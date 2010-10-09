#include "Video.h"

// std
#include <stdexcept>

// header from ffmpeg
#include <libavutil/avutil.h>

// headers from ffpp
#include <ffpp/Stream.h>
#include <ffpp/CodecContext.h>
#include <ffpp/InputFormatContext.h>
#include <ffpp/InputFormat.h>
#include <ffpp/Frame.h>
#include <ffpp/Packet.h>
#include <ffpp/ScalerContext.h>

// boost
#include <boost/optional.hpp>

// opencv
#include <opencv/cv.h>

// my stuff
#include <opencv/IplImageWrapper.h>
#include <opencv/functions.h>
#include <numeric/functions.hpp>

const bool DEBUGOUT = false;

/**
 * singleton class for instantiating ffmpeg
 */
class _AVInit
{
public:
	_AVInit() {
		av_register_all();
	}
	~_AVInit() {
		//av_free_static();
	}
};

static _AVInit __singleton;


Video::~Video()
{
	cleanup();
}

void Video::cleanup()
{
	// free all allocated objects
	if (_peekBuffer)
		delete _peekBuffer;
	if (_currentBuffer)
		delete _currentBuffer;
	if (_scalerCtx)
		delete _scalerCtx;
	delete _inCodecCtx;
	delete _inStream;
	delete _inFormatCtx;
	
	// reset internal variables
	_currentBuffer = 0;
	_peekBuffer = 0;
	_startPts = 0;
	_currentPts = 0;
	_peekPts = 0;
	_seekedSec = 0;
	_frameRate = 0;
	_invFrameRate = 0;
	_streamTimeBase = 0;
	_width = 0;
	_height = 0;
	_inStream = 0;
	_inCodecCtx = 0;
	_inFormatCtx = 0;
	_scalerCtx = 0;
}

void Video::init()
{
	// open inpute file and get the input format context
	_inFormatCtx = new ffpp::InputFormatContext(_fileName.c_str());

	// find the input video stream
	_inStream = new ffpp::Stream();
	*_inStream = _inFormatCtx->GetVideoStream();

	// get the input codec context
	_inCodecCtx = new ffpp::CodecContext(0);
	*_inCodecCtx = _inStream->GetCodecContext();

	// throw an error if something went wrong
	if (!(_inFormatCtx && _inStream && _inCodecCtx))
		throw std::runtime_error("Error initializing video!");

	// get the width and height of the video (take also into account the aspect ratio)
	_width = _inCodecCtx->GetWidth();
	_height = _inCodecCtx->GetHeight();
	if (_inCodecCtx->get()->sample_aspect_ratio.num > 0) {
		double aspectRatio = av_q2d(_inCodecCtx->get()->sample_aspect_ratio);
		if (DEBUGOUT) std::cout << "### aspectRatio: " << aspectRatio << std::endl;
		if (fuzzyGreater(aspectRatio, 1))
			_width = static_cast<int>(round(_width * aspectRatio));
		if (fuzzyLower(aspectRatio, 1))
			_width = static_cast<int>(round(_height / aspectRatio));
	}
	
	// get the frame rate
	_frameRate = av_q2d(_inStream->GetFrameRate());
	if (0.0 == _frameRate)
		throw std::runtime_error("Error video frame rate is 0!");
	_invFrameRate = 1 / _frameRate;
	_streamTimeBase = av_q2d(_inStream->GetTimeBase());
	if (DEBUGOUT) std::cout << "### invFrameRate: " << _invFrameRate << std::endl;
	if (DEBUGOUT) std::cout << "### frameRate: " << _frameRate << std::endl;

	// get the first frame, then we know the starting time of the first frame
	if (DEBUGOUT) std::cout << "### startPts: " << _startPts << std::endl;
	advance(0);
	_startPts = _currentPts;
	if (DEBUGOUT) std::cout << "### startPts: " << _startPts << std::endl;
	_seekedSec = 0;
}

int Video::getWidth() const
{
	return _width;
}

int Video::getHeight() const
{
	return _height;
}

double Video::getDuration() const
{
	return std::max(_probedDuration, std::max(_inStream->get()->duration * _streamTimeBase, _inFormatCtx->get()->duration / double(AV_TIME_BASE)));
}

Video::FrameIndex Video::numOfFrames() const
{
	return static_cast<FrameIndex>(round(getDuration() * _frameRate));
}

double Video::getFrameRate() const
{
	return _frameRate;
}

Video::FrameIndex Video::getFrameIndex() const
{
	return static_cast<FrameIndex>(round(_seekedSec * _frameRate));
}

double Video::getTime() const
{
	return _seekedSec;
}

const IplImageWrapper Video::getFrame() const
{
	if (_currentBuffer)
		return *_currentBuffer;
	else
		return IplImageWrapper();
}

bool Video::advance(double seekToSec, bool force)
{
	// check whether we need to retrieve the next frame
	bool foundFrame = false;
	double currentSec = (_currentPts - _startPts) * _streamTimeBase;
	double peekSec = (_peekPts - _startPts) * _streamTimeBase;
	if (DEBUGOUT) std::cout << "##### advance(" << seekToSec << ")" << std::endl;
	if (DEBUGOUT) std::cout << "# position: " << currentSec << "s/" << _currentPts << " " << seekToSec << "s " << peekSec << "s/" << _peekPts << std::endl;
	if (fuzzyLower(seekToSec, peekSec) && _currentBuffer && !force)
	//if ((seekToSec < 0.5 * (currentSec + peekSec) || (seekToSec < peekSec && seekToSec >= currentSec) && _currentBuffer)
		foundFrame = true;

	// get the next packet containing a frame
	boost::optional<ffpp::Packet> optInPacket;
	while (!(foundFrame && _currentBuffer) && (optInPacket = _inFormatCtx->ReadPacket())) {
		// check whether the packet belongs to our video stream
		if (DEBUGOUT) std::cout << "### new packet" << std::endl;
		ffpp::Packet& inPacket = *optInPacket;
		if (!_inStream->OwnsPacket(inPacket))
			continue;
		if (DEBUGOUT) std::cout << "# packet in stream: " << (inPacket.GetPTS() - _startPts) * _streamTimeBase << "s/" << inPacket.GetPTS() << std::endl;

		// try to decode frame
		boost::optional<ffpp::Frame> optInFrame = _inCodecCtx->DecodeFrame(inPacket);
		if (!optInFrame)
			continue;
		ffpp::Frame& inFrame = *optInFrame;
		if (DEBUGOUT) std::cout << "# new frame: " << (inFrame.GetPTS() - _startPts) * _streamTimeBase << "s/" << inFrame.GetPTS() << std::endl;
		
		// we need to initialize the ScalerContext in this loop
		// since it might be that we cannot initialize it earlier
		// set the resampling context with format BGR for OpenCV
		if (!_scalerCtx)
			_scalerCtx = new ffpp::ScalerContext(*_inCodecCtx, getWidth(), getHeight(), PIX_FMT_BGR24);

		// swap current buffer and peek buffer
		std::swap(_currentBuffer, _peekBuffer);
		std::swap(_currentPts, _peekPts);

		// create a buffer image if necessary
		if (!_peekBuffer)
			_peekBuffer = new IplImageWrapper(cvSize(getWidth(), getHeight()), IPL_DEPTH_8U, 3);

		// convert frame to BGR format and copy it into the IplImage structure
		ffpp::Frame bgrFrame = _scalerCtx->Rescale(inFrame);
		copyRawData(*_peekBuffer, bgrFrame.get()->data[0], bgrFrame.get()->linesize[0]);
		_peekPts = inFrame.GetPTS();

		// check whether we found the frames we are looking for
		currentSec = (_currentPts - _startPts) * _streamTimeBase;
		peekSec = (_peekPts - _startPts) * _streamTimeBase;
		if (fuzzyLower(seekToSec, peekSec) && _currentBuffer)
			// flag that we found a frame :)
			foundFrame = true;
		if (DEBUGOUT) std::cout << "# new position: " << currentSec << "s/" << _currentPts << " " << seekToSec << "s " << peekSec << "s/" << _peekPts << std::endl;
	}

	// update the current time we are virtually at
	// in case we are very close to the pts of the frame, this is our position
	_seekedSec = seekToSec;
	if (fuzzyLowerEqual(std::abs(currentSec - seekToSec), 0.25 * _invFrameRate))
		_seekedSec = currentSec;
	
	return foundFrame;
}

bool Video::nextFrame()
{
	return advance(getTime() + _invFrameRate);
}

bool Video::prevFrame()
{
	return seek(getTime() - _invFrameRate);
}

bool Video::seek(FrameIndex frame)
{
	if (frame - getFrameIndex() == 1)
		return nextFrame();
	return seek(frame * _invFrameRate);
}

bool Video::seek(double seconds)
{
	// get the destination in pts
	FrameIndex destination = static_cast<FrameIndex>(round(seconds / _streamTimeBase + _startPts));
	if (DEBUGOUT) std::cout << "##### seek(" << seconds << ") => pts: " << destination << std::endl;

	// if we are very close to the begin, we close and re-open the video sequence
	if (seconds <= 1.25 * ffpp::InputFormatContext::SAFE_SEEK_SECONDS) {
		if (DEBUGOUT) std::cout << "# re-opening video sequence" << std::endl;
		cleanup();
		init();
	}
	else {
		try {
			//_inFormatCtx->Seek(static_cast<float>(seconds + _startPts * _streamTimeBase));
			_inFormatCtx->Seek(*_inStream, destination);
		}
		catch (std::exception& e) {
			if (DEBUGOUT) std::cout << "# re-opening video sequence and seeking frame-by-frame" << std::endl;
			cleanup();
			init();
			return advance(seconds);
		}
	}

	// if we seeked to the first frame, we don't need to call advance :)
	if (fuzzyGreaterEqual(0.0, seconds))
		return true;

	// call advance to go frame by frame to the correct position
	return advance(seconds, true);
}

bool Video::probeFrame(Video::FrameIndex frame)	
{
	return probeFrame(frame * _invFrameRate);
}

bool Video::probeFrame(double seconds)	
{
	double oldPos = getTime();
	bool ret = seek(seconds);
	if (ret)
		_probedDuration = seconds;
	seek(oldPos);
	return ret;
}


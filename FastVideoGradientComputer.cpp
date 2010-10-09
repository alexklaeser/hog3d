#include "FastVideoGradientComputer.h"

#include <cmath>
#include <boost/typeof/typeof.hpp>
#include <opencv/cv.h>
//#include <opencv/highgui.h>

#include <opencv/Video.h>
#include <opencv/functions.h>


using std::cout;
using std::cerr;
using std::endl;


FastVideoGradientComputer::~FastVideoGradientComputer()
{
	// release convolution matrices
	cvReleaseMat(&_hDevMat);
	cvReleaseMat(&_vDevMat);
}

void FastVideoGradientComputer::init()
{
//	cvNamedWindow("image", CV_WINDOW_AUTOSIZE);
	if (!_video)
		return;
	
	// initiate the image buffer
	_intImgs.resize(_nFramesBuffer + 1); // allocate one position more for integration purposes
	CvSize size = cvSize(getWidth(), getHeight());
	for (std::size_t i = 0; i < _intImgs.size(); ++i) {
		_intImgs[i].resize(3);
		for (std::size_t j = 0; j < 3; ++j) {
			_intImgs[i][j] = IplImageWrapper(size, IPL_DEPTH_64F, 1);
			cvZero(_intImgs[i][j]);
		}
	}

	// initiate convolution matrices
	if (!_hDevMat || !_vDevMat) {
		_hDevMat = cvCreateMat(1, 2, CV_32FC1);
		cvmSet(_hDevMat, 0, 0, -1);
		cvmSet(_hDevMat, 0, 1, 1);
		_vDevMat = cvCreateMat(2, 1, CV_32FC1);
		cvmSet(_vDevMat, 0, 0, -1);
		cvmSet(_vDevMat, 1, 0, 1);
	}

	// initiate the temp frame
	_tmpFrame = IplImageWrapper(size, IPL_DEPTH_8U, _video->getFrame()->nChannels);
	_imgGrayFloat = IplImageWrapper(size, IPL_DEPTH_32F, 1);
	_imgGrayFloat2 = IplImageWrapper(size, IPL_DEPTH_32F, 1);
}

bool FastVideoGradientComputer::getFrame(long frame, IplImageWrapper img)
{
	bool ok = true;
	BOOST_AUTO(iBufferedFrame, _bufferedFrames.find(frame));
	if (iBufferedFrame != _bufferedFrames.end()) {
		// frame is buffered :), we can reuse it
		if (img)
			cvCopy(iBufferedFrame->second, img);
	}
	else {
		// frame needs to be obtained from the video .. check whether the frame should exist
		if (frame < 0 || frame >= _video->numOfFrames())
			return false;
		
		// set the frame for the video
//		cout << "_video->setFrame(" << frame << ")" << endl;
		if (ok = _video->seek(static_cast<Video::FrameIndex>(frame))) {
			cvResize(_video->getFrame(), _tmpFrame, CV_INTER_AREA);
			convert2GrayFloatImg(_tmpFrame, img);
			_bufferedFrames[frame] = img.clone();
		}
	}
	return ok;
}

void FastVideoGradientComputer::cleanBufferedFrames(long frame)
{
	// we only keep two frames before and after the current frame
	for (BOOST_AUTO(iBufferedFrame, _bufferedFrames.begin()); iBufferedFrame != _bufferedFrames.end();)
		if (std::abs(iBufferedFrame->first - frame) > 1)
			_bufferedFrames.erase(iBufferedFrame++);
		else
			++iBufferedFrame;
}

void FastVideoGradientComputer::setIntegralGradientImage(std::size_t iPos, long frame)
{
//	cout << "FastVideoGradientComputer::setIntegralGradientImage(" << iPos << ", " << frame << ")" << endl;
//	cout << "# setGradImg (" << iPos << ", " << frame << ")" << endl;
	assert(iPos < _intImgs.size());

	// check whether the given frame lies inside the video
	if (frame < 0 || frame >= _video->numOfFrames()) {
		for (std::size_t j = 0; j < 3; ++j)
			cvZero(_intImgs[iPos][j]);
	}
	// get the frame and compute gradient integral images
	else {
		// ensure that whatever happens, we start at (frame-1) and go then 
		// consecutively to the next frames  
		getFrame(frame - 1);
		
		// check whether we can access the current frame
		if (!getFrame(frame, _imgGrayFloat)) {
			for (std::size_t j = 0; j < 3; ++j)
				cvZero(_intImgs[iPos][j]);
		}
		else {
			assert(_tmpFrame->width == getWidth() && _tmpFrame->height == getHeight());
			
			// compute gradients in x direction [note: _imgGrayFloat has already been copied by getFrame()]
			// note: use the [-1, 1] kernel instead of Sobel since smoothing will
			//       produce gradient value that are not invariant to scale changes
			//cvSobel(_imgGrayFloat, _imgGrayFloat2, 1, 0, 1);
			cvFilter2D(_imgGrayFloat, _imgGrayFloat2, _hDevMat, cvPoint(0, 0));
//			cout << "frame: " << frame << endl;
//			cvShowImage("image", _imgGrayFloat);
//			cvWaitKey(0);
//			cvShowImage("image", _imgGrayFloat2);
//			cvWaitKey(0);
			computeIntegralImage(_imgGrayFloat2, _intImgs[iPos][0]);
			
			// compute gradients in y direction
			//cvSobel(_imgGrayFloat, _imgGrayFloat2, 0, 1, 1);
			cvFilter2D(_imgGrayFloat, _imgGrayFloat2, _vDevMat, cvPoint(0, 0));
//			cvShowImage("image", _imgGrayFloat2);
//			cvWaitKey(0);
			computeIntegralImage(_imgGrayFloat2, _intImgs[iPos][1]);
			
			// compute gradients in t direction (type [-1, 1])
			getFrame(frame + 1, _imgGrayFloat2);
			cvSub(_imgGrayFloat2, _imgGrayFloat, _imgGrayFloat);
//			cvShowImage("image", _imgGrayFloat);
//			cvWaitKey(0);
			computeIntegralImage(_imgGrayFloat, _intImgs[iPos][2]);
			
			// clear buffers that are too old/young
			cleanBufferedFrames(frame);
		}
	}
}

FastVideoGradientComputer::VectorType
FastVideoGradientComputer::getGradientVector(const Box3D& box) const
{
//	cout << "FastVideoGradientComputer::getGradientVector([" << box.x << ", " << box.y << ", " << box.t << ", " << box.width << ", " << box.height << ", " << box.length << "])" << endl; 

	// get the begining/end img buffer positions
	std::size_t t1 = (std::min(_intImgs.size() - 1, static_cast<std::size_t>(std::max(0.0, round(box.t - _firstFrame)))) + _iFirstPos) % _intImgs.size();
	std::size_t t2 = (std::min(_intImgs.size() - 1, static_cast<std::size_t>(std::max(0.0, round(box.t + box.length - _firstFrame)))) + _iFirstPos) % _intImgs.size();
	
	// compute the feature vector
	Box<int> box2D = static_cast<Box<int> >(Box<double>(box.x, box.y, box.width, box.height));
//	cout << "#   _iFirstPos: " << _iFirstPos << " _firstFrame: " << _firstFrame << endl;
//	cout << "#   t1: " << box.t << " t2: " << box.t + box.length << " -> t1: " << t1 << " t2: " << t2 << " box2D: " << box2D.toStr() << endl;
	VectorType vec(3);
	for (std::size_t i = 0; i < 3; ++i)
		vec[i] = getIntegralRegion(_intImgs[t2][i], box2D) - getIntegralRegion(_intImgs[t1][i], box2D);
	
	return vec;
}

void FastVideoGradientComputer::gotoFrame(std::size_t frame)
{
//	cout << "FastVideoGradientComputer::gotoFrame(" << frame << ")" << endl; 
	std::size_t currentFrame = _firstFrame + ICENTER;
	std::size_t nNewFrames = frame > currentFrame ? frame - currentFrame : currentFrame - frame;
	std::size_t nKeepFrames(_nFramesBuffer);
	std::size_t iNewFirstPos(_iFirstPos);
	long newFirstFrame = static_cast<long>(frame) - static_cast<long>(ICENTER);
//	cout << "# gotoFrame(" << frame << ")" << endl;
//	cout << "# -> _firstFrame: " << _firstFrame << " _iFirstPos: " << _iFirstPos << endl;

	// check whether we have to initiate the whole buffer from the scratch
	// or whether we might use existing frames
	if (nNewFrames >= _nFramesBuffer || !_hasBeenLoaded) {
//		cout << "# new initialization" << endl;
		iNewFirstPos = 0;
			
		// clear the first image in the vector
		for (std::size_t j = 0; j < 3; ++j)
			cvZero(_intImgs[0][j]);

		// update the integral images
		for (std::size_t i = 0; i < _nFramesBuffer; ++i) {
			setIntegralGradientImage(i + 1, newFirstFrame + i);
			for (std::size_t j = 0; j < 3; ++j) {
				cvAdd(_intImgs[i][j], _intImgs[i + 1][j], _intImgs[i + 1][j]);
			}
		}

		
		_hasBeenLoaded = true;
	}
	// check whether we go forward 
	else if (frame > currentFrame) {
		nKeepFrames = nNewFrames > _nFramesBuffer ? std::size_t(0) : _nFramesBuffer - nNewFrames;
		iNewFirstPos = (_iFirstPos + nNewFrames) % _intImgs.size();
		assert (nNewFrames + nKeepFrames == _nFramesBuffer);
//		cout << "# -> currentFrame: " << currentFrame << " nNewFrames: " << nNewFrames << " nKeepFrames: " << nKeepFrames << " iNewFirstPos: " << iNewFirstPos << " newFirstFrame: " << newFirstFrame << endl;

		// update the integral images
//		cout << "# update forward" << endl;
		for (std::size_t i = nNewFrames; i > 0; --i) {
			std::size_t iPos = ((iNewFirstPos + _intImgs.size()) - i) % _intImgs.size(); // position in the image vector
			std::size_t iPosBefore = ((iPos + _intImgs.size()) - 1) % _intImgs.size(); // the position before in the image vector
			long newFrame = (nNewFrames - i) + nKeepFrames + newFirstFrame;
//			cout << "#   i: " << i << " iPos: " << iPos << " iPosBefore: " << iPosBefore << " newFrame: " << newFrame << endl;
			setIntegralGradientImage(iPos, newFrame);
			for (std::size_t j = 0; j < 3; ++j)
				cvAdd(_intImgs[iPos][j], _intImgs[iPosBefore][j], _intImgs[iPos][j]);
		}
	}
	// check whether we go backward
	else if (frame < currentFrame) {
		nKeepFrames = nNewFrames > _nFramesBuffer ? std::size_t(0) : _nFramesBuffer - nNewFrames;
		iNewFirstPos = ((_iFirstPos + _intImgs.size()) - nNewFrames) % _intImgs.size();
		assert (nNewFrames + nKeepFrames == _nFramesBuffer);
//		cout << "# -> currentFrame: " << currentFrame << " nNewFrames: " << nNewFrames << " nKeepFrames: " << nKeepFrames << " iNewFirstPos: " << iNewFirstPos << " newFirstFrame: " << newFirstFrame << endl;
		
		// update the integral images
//		cout << "# update backward" << endl;
		for (std::size_t i = 1; i <= nNewFrames; ++i) {
			std::size_t iPos = ((iNewFirstPos + nNewFrames + _intImgs.size()) - i) % _intImgs.size(); // position in the image vector
			std::size_t iPosAfter = (iPos + 1) % _intImgs.size(); // the position after in the image vector
			long newFrame = (newFirstFrame - i) + nNewFrames;
//			cout << "#   i: " << i << " iPos: " << iPos << " iPosAfter: " << iPosAfter << " newFrame: " << newFrame << endl;
			setIntegralGradientImage(iPos, newFrame);
			for (std::size_t j = 0; j < 3; ++j)
				cvSub(_intImgs[iPosAfter][j], _intImgs[iPos][j], _intImgs[iPos][j]);
		}
	}

	_iFirstPos = iNewFirstPos;
	_firstFrame = newFirstFrame;
}


std::size_t FastVideoGradientComputer::currentFrame() const
{
	return static_cast<std::size_t>(std::max(std::size_t(0), static_cast<std::size_t>(_firstFrame + ICENTER)));
}

std::size_t FastVideoGradientComputer::getVideoLength() const
{
	return static_cast<std::size_t>(_video->numOfFrames());
}

std::size_t FastVideoGradientComputer::getBufferLength() const
{
	return _nFramesBuffer;
}

int FastVideoGradientComputer::getWidth() const
{
	return static_cast<int>(round(_video->getWidth() * _scaleFactor));
}

int FastVideoGradientComputer::getHeight() const
{
	return static_cast<int>(round(_video->getHeight() * _scaleFactor));
}


















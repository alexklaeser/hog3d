#ifndef _VIDEO_H 
#define _VIDEO_H

// std 
#include <string>

// forward declarations
namespace ffpp {
	class ScalerContext;
	class InputFormatContext;
	class Stream;
	class CodecContext;
}
class IplImageWrapper;


class Video
{
public:
	typedef long long FrameIndex;

protected:
	// stuff
	std::string _fileName;
	IplImageWrapper *_currentBuffer;
	IplImageWrapper *_peekBuffer;
	long long _startPts;
	long long _currentPts;
	long long _peekPts;
	double _seekedSec;
	double _frameRate;
	double _invFrameRate;
	double _streamTimeBase;
	int _width;
	int _height;
	double _probedDuration;

	// ffpp variables
	ffpp::Stream* _inStream;
	ffpp::CodecContext* _inCodecCtx;
	ffpp::InputFormatContext* _inFormatCtx;
	ffpp::ScalerContext* _scalerCtx;

	void init();
	void cleanup();
	bool advance(double sec, bool force = false);

public:
	Video(const std::string& fileName)
		: _fileName(fileName), _currentBuffer(0), _peekBuffer(0), 
		_startPts(0), _currentPts(0), _peekPts(0),
		_frameRate(0), _invFrameRate(0),
		_streamTimeBase(0), _width(0), _height(0), _probedDuration(0),
		_inStream(0), _inCodecCtx(0), _inFormatCtx(0), _scalerCtx(0)
	{
		init();
	}

	~Video();

	// queries
	const std::string& getFileName() const
	{
		return _fileName;
	}
	int getWidth() const;
	int getHeight() const;
	double getDuration() const;
	FrameIndex numOfFrames() const;
	FrameIndex getFrameIndex() const;
	double getFrameRate() const;
	double getTime() const;

	/**
	 * returns the internal offset for the first frame
	 */
	double getStartTime() const
	{
		return _startPts * _streamTimeBase;
	}

	// frame access
	const IplImageWrapper getFrame() const;

	// seeking functionality
	bool nextFrame();
	bool prevFrame();
	bool seek(FrameIndex frame);
	bool seek(double seconds);

	// other functionalities
	bool probeFrame(FrameIndex frame);
	bool probeFrame(double seconds);
};

#endif

#ifndef FASTVIDEOGRADIENTCOMPUTER_H_
#define FASTVIDEOGRADIENTCOMPUTER_H_

#include <vector>
#include <cmath>
#include <map>

#include <boost/numeric/ublas/vector.hpp>

#include <opencv/IplImageWrapper.h>
#include <geometry/Box.hpp>
#include "Box3D.h"

class Video;

class FastVideoGradientComputer
{
public:
	typedef boost::numeric::ublas::vector<double> VectorType;
	
protected:
	Video* _video;
	std::size_t _nFramesBuffer;
	const std::size_t ICENTER;
	std::vector<std::vector<IplImageWrapper> > _intImgs;
	long _firstFrame;
	std::size_t _iFirstPos;
	IplImageWrapper _tmpFrame, _imgGrayFloat, _imgGrayFloat2;
	bool _hasBeenLoaded;
	double _scaleFactor;
	CvMat* _hDevMat;
	CvMat* _vDevMat;

	// little buffer to avoid jumping backwards with 
	std::map<long, IplImageWrapper> _bufferedFrames;

protected:
	void init();
	
	void setIntegralGradientImage(std::size_t iPos, long frame);
	
	void cleanBufferedFrames(long frame);
	
public:
	FastVideoGradientComputer(Video* video, std::size_t nFramesBuffer = 100, double scaleFactor = 1)
		: _video(video), _nFramesBuffer(nFramesBuffer), ICENTER(nFramesBuffer / 2),
		_firstFrame(0), _iFirstPos(0), _hasBeenLoaded(false), _scaleFactor(scaleFactor),
		_hDevMat(0), _vDevMat(0)
	{
		init();
	}
	
	virtual ~FastVideoGradientComputer();
	
	VectorType getGradientVector(const Box3D& box) const;
	
	void gotoFrame(std::size_t frame);

	bool getFrame(long frame, IplImageWrapper img = IplImageWrapper());
	
	bool isInBuffer(const Box3D& box) const
	{
		return round(box.t) >= _firstFrame && round(box.t + box.length) < (_firstFrame + _nFramesBuffer)
				&& isInVideo(box);
	}
	
	bool isInVideo(const Box3D& box) const
	{
		if (box.t < 0 || (box.t + box.length) >= getVideoLength())
			return false;
		Box<double> box2DVideo(0, 0, getWidth(), getHeight());
		Box<double> box2D(box.x, box.y, box.width, box.height);
		return box2DVideo.contains(box2D);
	}
	
	std::size_t currentFrame() const;
	
	std::size_t getVideoLength() const;
	
	std::size_t getBufferLength() const;
	
	int getWidth() const;
	
	int getHeight() const;
	
	Video* getVideo()
	{
		return _video;
	}
	
	const Video* getVideo() const
	{
		return _video;
	}
	
	void setVideo(Video* v)
	{
		_video = v;
		init();
	}
	
};

#endif /*FASTVIDEOGRADIENTCOMPUTER_H_*/

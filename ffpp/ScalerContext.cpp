#include "ScalerContext.h"

#include <stdexcept>

namespace ffpp
{

void ScalerContext::Construct(int inWidth, int inHeight, enum PixelFormat inPixelFormat,
                              int outWidth, int outHeight, enum PixelFormat outPixelFormat)
{
	this->inHeight = inHeight;
	
	struct SwsContext* _avp;

	_avp = sws_getContext(inWidth, inHeight, inPixelFormat,
			              outWidth, outHeight, outPixelFormat,
	                      SWS_BICUBIC, NULL, NULL, NULL);
	if (!_avp) throw(std::runtime_error("Could not create resampling context"));
	   
	avp = boost::shared_ptr<SwsContext>(_avp, &ScalerContext::DeconstructContext);
	
	AVPicture* _pict_tmp = new AVPicture();
	_pict_tmp->data[0] = 0;

	pict_tmp = boost::shared_ptr<AVPicture>(_pict_tmp, &ScalerContext::DeconstructPicture);

	if (avpicture_alloc(_pict_tmp, outPixelFormat, outWidth, outHeight))
	    throw(std::runtime_error("Could not allocate picture buffer"));
}

void ScalerContext::DeconstructContext(SwsContext* _avp)
{
    sws_freeContext(_avp);
}

void ScalerContext::DeconstructPicture(AVPicture* _pict_tmp)
{
    if (_pict_tmp->data[0]) av_free(_pict_tmp->data[0]);
    delete _pict_tmp;
}

ScalerContext::ScalerContext(int inWidth, int inHeight, enum PixelFormat inPixelFormat,
		                     int outWidth, int outHeight, enum PixelFormat outPixelFormat) 
{
	Construct(inWidth, inHeight, inPixelFormat,
			  outWidth, outHeight, outPixelFormat);
}

ScalerContext::ScalerContext(const CodecContext& inCodecCtx, const CodecContext& outCodecCtx)
{
	Construct(inCodecCtx.GetWidth(), inCodecCtx.GetHeight(), inCodecCtx.GetPixelFormat(),
			  outCodecCtx.GetWidth(), outCodecCtx.GetHeight(), outCodecCtx.GetPixelFormat());
}

ScalerContext::ScalerContext(const CodecContext& inCodecCtx, int outWidth, int outHeight, enum PixelFormat outPixelFormat)
{
	Construct(inCodecCtx.GetWidth(), inCodecCtx.GetHeight(), inCodecCtx.GetPixelFormat(),
			  outWidth, outHeight, outPixelFormat);
}

ScalerContext::ScalerContext(int inWidth, int inHeight, enum PixelFormat inPixelFormat, const CodecContext& outCodecCtx)
{
	Construct(inWidth, inHeight, inPixelFormat,
			  outCodecCtx.GetWidth(), outCodecCtx.GetHeight(), outCodecCtx.GetPixelFormat());	
}

Frame ScalerContext::Rescale(const Frame& inFrame) const
{
    Frame outFrame;
    
    *(AVPicture*)outFrame.get() = *pict_tmp.get();
    
    if (sws_scale(avp.get(), const_cast<Frame&>(inFrame).get()->data, const_cast<Frame&>(inFrame).get()->linesize,
                  0, inHeight, outFrame.get()->data, outFrame.get()->linesize) < 0)
	throw(std::runtime_error("Resampling failed"));
	
    return outFrame;
}

}

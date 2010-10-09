#ifndef SCALERCONTEXT_H_
#define SCALERCONTEXT_H_

#include <boost/shared_ptr.hpp>

extern "C" {
	#include <libswscale/swscale.h>
}

#include "CodecContext.h"

namespace ffpp
{

class ScalerContext
{
	protected:
		boost::shared_ptr<SwsContext> avp;
		boost::shared_ptr<AVPicture> pict_tmp;
		int inHeight;
		
		void Construct(int inWidth, int inHeight, enum PixelFormat inPixelFormat,
		               int outWidth, int outHeight, enum PixelFormat outPixelFormat);
		static void DeconstructContext(SwsContext* _avp);
		static void DeconstructPicture(AVPicture* _pict_tmp);
	
	public:
		ScalerContext(int inWidth, int inHeight, enum PixelFormat inPixelFormat,
                      int outWidth, int outHeight, enum PixelFormat outPixelFormat); 
		ScalerContext(const CodecContext& inCodecCtx, const CodecContext& outCodecCtx);
		ScalerContext(const CodecContext& inCodecCtx, int outWidth, int outHeight, enum PixelFormat outPixelFormat);
		ScalerContext(int inWidth, int inHeight, enum PixelFormat inPixelFormat, const CodecContext& outCodecCtx);
		
		Frame Rescale(const Frame& inFrame) const;
};

}

#endif /*SCALERCONTEXT_H_*/

#ifndef CODEC_H_
#define CODEC_H_

#include <boost/logic/tribool.hpp>

extern "C" {
	#include <libavformat/avformat.h>
}

namespace ffpp
{

class Codec
{
	protected:
		const AVCodec* avp;
	
	public:
		Codec(const AVCodec* avp = 0);
		Codec(const char* name);
		
		const AVCodec* get() const;
		
		boost::tribool IsSupportedPixelFormat(enum PixelFormat pixelFormat) const;
		enum PixelFormat GetDefaultPixelFormat() const;
        
        AVRational GetSupportedFrameRate(const AVRational& frame_rate) const;
        
        enum CodecID GetID() const;
};

}

#endif /*CODEC_H_*/

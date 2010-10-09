#include "Codec.h"

#include <stdexcept>

namespace ffpp
{

Codec::Codec(const AVCodec* avp): avp(avp)
{}

Codec::Codec(const char* name): avp(avcodec_find_encoder_by_name(name))
{
    if (!avp) throw(std::runtime_error("Could not find codec"));
}

const AVCodec* Codec::get() const
{
	return avp;
}

boost::tribool Codec::IsSupportedPixelFormat(enum PixelFormat pixelFormat) const
{
	if (!get()->pix_fmts) return boost::logic::indeterminate;

	const enum PixelFormat *p;
	for (p = get()->pix_fmts; *p != -1; ++p)
	{
		if (*p == pixelFormat) return true;
	}
	return false;
}

enum PixelFormat Codec::GetDefaultPixelFormat() const
{
	if (!get()->pix_fmts) throw(std::runtime_error("No pixel formats specified for a codec"));
	return get()->pix_fmts[0];
}

AVRational Codec::GetSupportedFrameRate(const AVRational& frame_rate) const
{
    AVRational fps(frame_rate);

    while (get()->id == CODEC_ID_MPEG4 && fps.num > (1<<16)-1)
    {
        fps.num /= 2;
        fps.den /= 2;
    }

    if (!get()->supported_framerates) return fps;
    
    return get()->supported_framerates[av_find_nearest_q_idx(fps, get()->supported_framerates)];
}

enum CodecID Codec::GetID() const
{
    return get()->id;
}

}

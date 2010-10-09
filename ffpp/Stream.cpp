#include "Stream.h"

namespace ffpp
{

Stream::Stream(AVStream *avp): avp(avp)
{}

int Stream::GetIndex() const
{
	return get()->index;
}

bool Stream::OwnsPacket(const Packet& pkt) const
{
	return pkt.GetStreamIndex() == GetIndex();
}

void Stream::OwnPacket(Packet& pkt) const
{
	pkt.SetStreamIndex(GetIndex());
}

CodecContext Stream::GetCodecContext()
{
	return CodecContext(get()->codec);
}

const CodecContext Stream::GetCodecContext() const
{
	return CodecContext(get()->codec);
}

AVStream* Stream::get()
{
	return avp;	
}

const AVStream* Stream::get() const
{
	return avp;
}

AVRational Stream::GetAspectRatio() const
{
	return get()->codec->sample_aspect_ratio;
}

void Stream::SetAspectRatio(const AVRational& aspect_ratio)
{
	get()->sample_aspect_ratio = get()->codec->sample_aspect_ratio = aspect_ratio;
}

AVRational Stream::GetFrameRate() const
{
	return get()->r_frame_rate;
}

AVRational Stream::GetTimeBase() const
{
	return get()->time_base;
    //return get()->codec->time_base;
}

void Stream::SetFrameRate(const AVRational& frame_rate)
{
    AVRational fps = GetCodecContext().GetEncoder().GetSupportedFrameRate(frame_rate);
    get()->codec->time_base.num = fps.den;
    get()->codec->time_base.den = fps.num;
}

/*
void Stream::SetQuality(float qscale)
{
	get()->codec->global_quality = get()->quality = FF_QP2LAMBDA * qscale;
}
*/

}

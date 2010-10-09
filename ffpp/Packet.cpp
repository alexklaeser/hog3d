#define __STDC_CONSTANT_MACROS 1
#include "Packet.h"

#include <stdint.h>

#include "Stream.h"

namespace ffpp
{

Packet::Packet(): avp(new AVPacket(), &Packet::Destruct)
{}

AVPacket* Packet::get()
{
	return avp.get();	
}

const AVPacket* Packet::get() const
{
	return avp.get();
}

void Packet::Destruct(AVPacket* _avp)
{
	delete _avp;
}

int Packet::GetStreamIndex() const
{
	return get()->stream_index;
}

void Packet::SetStreamIndex(int index)
{
	get()->stream_index = index;
}

long long Packet::GetPTS() const
{
	return get()->dts;
}

double Packet::GetSeconds(const Stream& stream) const
{
	return get()->dts * av_q2d(stream.GetTimeBase());
}

void Packet::SetPTS(long long pts)
{
    get()->dts = pts;
    if (get()->pts != AV_NOPTS_VALUE) get()->pts = get()->dts;
}

}

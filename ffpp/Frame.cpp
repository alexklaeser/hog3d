#include "Frame.h"

#include <stdexcept>

namespace ffpp
{

Frame::Frame()
	: pts(0)
{
	AVFrame* _avp = avcodec_alloc_frame();
	if (!_avp) throw(std::runtime_error("Could not allocate frame"));
	avp = boost::shared_ptr<AVFrame>(_avp, &Frame::Destruct);
}

AVFrame* Frame::get()
{
	return avp.get();	
}

const AVFrame* Frame::get() const
{
	return avp.get();
}

void Frame::Destruct(AVFrame* _avp)
{
	av_free(_avp);
}

long long Frame::GetPTS()
{
	return pts;
}

void Frame::SetPTS(long long pts)
{
	this->pts = pts;
}

float Frame::GetQuality() const
{
	return get()->quality / FF_QP2LAMBDA;
}

void Frame::SetQuality(float qscale)
{
	get()->quality = (int)(qscale * FF_QP2LAMBDA);
}

unsigned char Frame::Get8Pixel(int x, int y) const
{
    return *(get()->data[0]+x + get()->linesize[0]*y);
}

}

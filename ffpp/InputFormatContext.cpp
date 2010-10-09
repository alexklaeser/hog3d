#include "InputFormatContext.h"

#include <stdexcept>
#include <boost/format.hpp>
#include <boost/none.hpp>
#include <cstdio>
#include <iostream>

namespace ffpp
{

const float InputFormatContext::SAFE_SEEK_SECONDS = 1.0;

InputFormatContext::InputFormatContext(const char* filename)
{
	AVFormatContext* _avp;
	
	if (av_open_input_file(&_avp, filename, NULL, 0, NULL) != 0)
		throw(std::runtime_error(boost::str(boost::format("Could not open input file '%1%'") % filename)));
	
	avp = boost::shared_ptr<AVFormatContext>(_avp, &InputFormatContext::Destruct);
	if (av_find_stream_info(get()) < 0)
		throw(std::runtime_error(boost::str(boost::format("Could not retrieve stream information for file '%1%'") % filename)));
		
	/*
	int i;
	
	for (i=0; i<avp->nb_streams; ++i)
    {
		if (avp->streams[i] && avp->streams[i]->codec && avp->streams[i]->codec->codec)
		{
			avcodec_close(avp->streams[i]->codec);
		}
    }
    */	
}

void InputFormatContext::Destruct(AVFormatContext* _avp)
{
	av_close_input_file(_avp);
}

InputFormat InputFormatContext::GetFormat() const
{
	return get()->iformat;
}

boost::optional<Packet> InputFormatContext::ReadPacket()
{
	Packet pkt;
	if (av_read_frame(get(), pkt.get()) >= 0) return pkt;
	return boost::none;
}

void InputFormatContext::Seek(float seconds)
{
	if (seconds > SAFE_SEEK_SECONDS) seconds -= SAFE_SEEK_SECONDS;
	long long destination = (long long)(seconds * AV_TIME_BASE);

	if (av_seek_frame(get(), -1, destination, AVSEEK_FLAG_BACKWARD) < 0)
		throw(std::runtime_error("Could not seek default stream"));
}

void InputFormatContext::Seek(const Stream& stream, long long destination)
{
	long long margin = (long long)(SAFE_SEEK_SECONDS / av_q2d(stream.get()->time_base));
	if (destination > margin) destination -= margin;
	
	if (av_seek_frame(get(), stream.GetIndex(), destination, AVSEEK_FLAG_BACKWARD) < 0)
		throw(std::runtime_error(boost::str(boost::format("Could not seek stream #%1%") % stream.GetIndex())));
}

}

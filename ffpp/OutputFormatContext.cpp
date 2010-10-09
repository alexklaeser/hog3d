#include "OutputFormatContext.h"

#include <stdexcept>
#include <cstdio>
#include <cstring>

namespace ffpp
{

OutputFormatContext::OutputFormatContext(const char* filename, const OutputFormat& format): header_written(false)
{
	AVFormatContext* _avp = av_alloc_format_context();
	if (_avp == NULL)
		throw(std::runtime_error("Could not allocate output context"));
	
	avp = boost::shared_ptr<AVFormatContext>(_avp, &OutputFormatContext::Destruct);

	std::snprintf(get()->filename, sizeof(get()->filename), "%s", filename);
	
	get()->oformat = const_cast<AVOutputFormat*>(format.get());
	
    AVFormatParameters parameters;
    memset(&parameters, 0, sizeof(parameters));
    if (av_set_parameters(get(), &parameters) < 0)
	    throw(std::runtime_error("Could not set output format parameters"));
}

void OutputFormatContext::Destruct(AVFormatContext* _avp)
{
	unsigned int i;

	if (av_write_trailer(_avp) != 0) throw(std::runtime_error("Could not write trailer"));
	
	for (i=0; i<_avp->nb_streams; ++i)
	{
		av_freep(&_avp->streams[i]->codec);
		av_freep(&_avp->streams[i]);
	}
	if (!(_avp->oformat->flags & AVFMT_NOFILE)) url_fclose(_avp->pb);
	av_free(_avp);
}

OutputFormat OutputFormatContext::GetFormat() const
{
	return get()->oformat;
}

Stream OutputFormatContext::AddStream(enum CodecType type)
{
	return AddStream(type, av_guess_codec(get()->oformat, NULL, get()->filename, NULL, type));
}

Stream OutputFormatContext::AddStream(enum CodecType type, enum CodecID codec_id)
{
	AVStream *pStream = av_new_stream(get(), get()->nb_streams);
	if (!pStream) throw(std::runtime_error("Could not create the output stream"));
	
	avcodec_get_context_defaults2(pStream->codec, type);
	pStream->codec->codec_id = codec_id;
	pStream->codec->flags |= CODEC_FLAG_QSCALE;

	return pStream;
}

Stream OutputFormatContext::AddVideoStream()
{
	return AddStream(CODEC_TYPE_VIDEO);
}

Stream OutputFormatContext::AddVideoStream(const Codec& codec)
{
	return AddStream(CODEC_TYPE_VIDEO, codec.GetID());
}

void OutputFormatContext::WriteHeader()
{
    if (av_write_header(get()) < 0) throw(std::runtime_error("Could not write header"));
	header_written = true;
}

void OutputFormatContext::WritePacket(const Packet& pkt)
{
	if (!header_written) WriteHeader();
	
	if (av_interleaved_write_frame(get(), const_cast<Packet&>(pkt).get()) < 0)
	   throw(std::runtime_error("Could not write packet"));
}

}

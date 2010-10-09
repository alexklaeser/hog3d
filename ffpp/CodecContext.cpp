#include "CodecContext.h"

#include <stdexcept>
#include <cstdio>
#include <boost/none.hpp>

namespace ffpp
{

CodecContext::CodecContext(AVCodecContext* _avp): avp(_avp, &CodecContext::Close)
{}

void CodecContext::OpenDecoder()
{
	Codec decoder = GetDecoder();
	if (avcodec_open(get(), const_cast<AVCodec*>(decoder.get())) < 0) throw(std::runtime_error("Could not open input codec"));
}

void CodecContext::OpenEncoder()
{
	Codec encoder = GetEncoder();
	if (avcodec_open(get(), const_cast<AVCodec*>(encoder.get())) < 0) throw(std::runtime_error("Could not open output codec"));
}

void CodecContext::Close(AVCodecContext* _avp)
{
	if (_avp && _avp->codec) avcodec_close(_avp);
}

boost::optional<Frame> CodecContext::DecodeFrame(const Packet& pkt)
{
	if (!get()->codec) OpenDecoder();

	Frame frame;
	int frame_ready;

	if (avcodec_decode_video(get(), frame.get(), &frame_ready, pkt.get()->data, pkt.get()->size) < 0)
		throw(std::runtime_error("Could not decode packet"));

	// return nothing if we could not yet decode a frame
	if (!frame_ready)
		return boost::none;

	// set the pts of the frame as the dts of the packet (seems to work fine)
	frame.SetPTS(pkt.get()->dts);
	return frame;
}

boost::optional<Packet> CodecContext::EncodeFrame(const Frame& frame)
{
    if (!get()->codec) OpenEncoder();

    Packet pkt;
    
    int bufsize = FFMAX(1024*256, avpicture_get_size(GetPixelFormat(), GetWidth(), GetHeight()))*sizeof(uint8_t);
    
    av_init_packet(pkt.get());
    pkt.get()->data = (uint8_t*)av_malloc(bufsize);
    if (!pkt.get()->data) throw(std::runtime_error("Could not allocate packet"));
    pkt.get()->destruct = av_destruct_packet;
    
    pkt.get()->size = avcodec_encode_video(get(), pkt.get()->data, bufsize, frame.get());
    if (pkt.get()->size < 0) throw(std::runtime_error("Could not encode frame"));
    
    if (pkt.get()->size) return pkt;
    return boost::none;
}

AVCodecContext* CodecContext::get()
{
	return avp.get();	
}

const AVCodecContext* CodecContext::get() const
{
	return avp.get();
}

int CodecContext::GetWidth() const { return get()->width; }
void CodecContext::SetWidth(int width) { get()->width = width; }

int CodecContext::GetHeight() const { return get()->height; }
void CodecContext::SetHeight(int height) { get()->height = height; }

enum PixelFormat CodecContext::GetPixelFormat() const { return get()->pix_fmt; }
void CodecContext::SetPixelFormat(enum PixelFormat pix_fmt) { get()->pix_fmt = pix_fmt; }

int CodecContext::GetBitrate() const { return get()->bit_rate; }
void CodecContext::SetBitrate(int bitrate) { get()->bit_rate = bitrate; }

Codec CodecContext::GetEncoder() const
{
	AVCodec *pCodec = avcodec_find_encoder(get()->codec_id);
	if (!pCodec) throw(std::runtime_error("Could not find output codec"));
        return pCodec;
}

Codec CodecContext::GetDecoder() const
{
	AVCodec *pCodec = avcodec_find_decoder(get()->codec_id);
	if (!pCodec) throw(std::runtime_error("Could not find input codec"));
	return pCodec;
}

}

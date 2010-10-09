#ifndef STREAM_H_
#define STREAM_H_

#include <boost/shared_ptr.hpp>

extern "C" {
	#include <libavformat/avformat.h>
}

#include "CodecContext.h"
#include "Packet.h"

namespace ffpp
{

class Stream
{
	protected:
		AVStream *avp;
	
	public:
		Stream(AVStream *avp = 0);
		int GetIndex() const;
		bool OwnsPacket(const Packet& pkt) const;
		void OwnPacket(Packet& pkt) const;

	    CodecContext GetCodecContext();
	    const CodecContext GetCodecContext() const;
	    
	    AVStream* get();
	    const AVStream* get() const;
	    
	    AVRational GetAspectRatio() const;
	    AVRational GetFrameRate() const;
	    AVRational GetTimeBase() const;

	    void SetAspectRatio(const AVRational& aspect_ratio);
	    void SetFrameRate(const AVRational& frame_rate);
};

}

#endif /*STREAM_H_*/

#ifndef FORMATCONTEXT_H_
#define FORMATCONTEXT_H_

#include <boost/shared_ptr.hpp>

extern "C" {
	#include <libavformat/avformat.h>
}

#include "OutputFormat.h"
#include "Stream.h"

namespace ffpp
{

class FormatContext 
{
	protected:
	   boost::shared_ptr<AVFormatContext> avp;
	   
	   int FindStream(enum CodecType type) const;
	
	public:
	   FormatContext();
	   
	   Stream GetVideoStream();
	   const Stream GetVideoStream() const;
	   
	   AVFormatContext* get();
	   const AVFormatContext* get() const;
};

}

#endif /*FORMATCONTEXT_H_*/

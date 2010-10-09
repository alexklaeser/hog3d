#include "FormatContext.h"

#include <stdexcept>
#include <boost/format.hpp>

namespace ffpp
{

FormatContext::FormatContext()
{}

int FormatContext::FindStream(enum CodecType type) const
{
	   unsigned int i, stream, count;

	   count = 0;
	   stream = -1;

	   if (avp)
	   {
		   for (i=0; i<get()->nb_streams; ++i)
		   {
		      if (get()->streams[i] && get()->streams[i]->codec->codec_type == type)
		      {
		         if (count == 0) stream = i;
		         count++;
		      }
		   }
	   }

	   if (count == 0) throw(std::runtime_error(boost::str(boost::format("Could not find any stream of type %1%") % type)));
	   if (count > 1) throw(std::runtime_error(boost::str(boost::format("More than one video streams of type %1% found (%2%)") % type % count)));
		   
	   return stream;
}

Stream FormatContext::GetVideoStream()
{
	int stream = FindStream(CODEC_TYPE_VIDEO);
	return get()->streams[stream];
}

const Stream FormatContext::GetVideoStream() const
{
	int stream = FindStream(CODEC_TYPE_VIDEO);
	return get()->streams[stream];
}

AVFormatContext* FormatContext::get()
{
	return avp.get();
}

const AVFormatContext* FormatContext::get() const
{
	return avp.get();	
}

}

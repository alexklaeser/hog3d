#ifndef PACKET_H_
#define PACKET_H_

#include <boost/shared_ptr.hpp>

extern "C" {
	#include <libavformat/avformat.h>
}

namespace ffpp
{

class Stream;

class Packet
{
	protected:
		boost::shared_ptr<AVPacket> avp;
		
		static void Destruct(AVPacket* _avp);
	
	public:
		Packet();
		
		AVPacket* get();
		const AVPacket* get() const;
		
		int GetStreamIndex() const;
		void SetStreamIndex(int index);
		
		long long GetPTS() const;
		double GetSeconds(const Stream& stream) const;
		void SetPTS(long long pts);
};

}

#endif /*PACKET_H_*/

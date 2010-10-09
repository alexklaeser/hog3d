#ifndef INPUTFORMATCONTEXT_H_
#define INPUTFORMATCONTEXT_H_

#include <boost/optional.hpp>

#include "FormatContext.h"
#include "InputFormat.h"
#include "Packet.h"

namespace ffpp
{

class InputFormatContext: public FormatContext
{
	protected:
		static void Destruct(AVFormatContext* _avp);
		
	public:
		static const float SAFE_SEEK_SECONDS;

		InputFormatContext(const char* filename);
		
		InputFormat GetFormat() const;
		
		void Seek(float seconds);
		void Seek(const Stream& stream, long long destination);

		
		boost::optional<Packet> ReadPacket(); 
};

}

#endif /*INPUTFORMATCONTEXT_H_*/

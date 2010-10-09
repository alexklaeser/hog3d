#ifndef INPUTFORMAT_H_
#define INPUTFORMAT_H_

extern "C" {
	#include <libavformat/avformat.h>
}

namespace ffpp
{

class InputFormat
{
	protected:
		const AVInputFormat *avp;
	
	public:
		InputFormat(const AVInputFormat *avp = 0);
		
		const char* GetName() const;
		
		const AVInputFormat* get() const;
};

}

#endif /*INPUTFORMAT_H_*/

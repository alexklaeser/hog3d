#ifndef OUTPUTFORMAT_H_
#define OUTPUTFORMAT_H_

extern "C" {
	#include <libavformat/avformat.h>
}

namespace ffpp
{

class OutputFormat
{
	protected:
		const AVOutputFormat *avp;
	
	public:
		OutputFormat(const AVOutputFormat *avp = 0);
		OutputFormat(const char* name, const char* filename);

		const char* GetName() const;

		const AVOutputFormat* get() const;
};

}

#endif /*OUTPUTFORMAT_H_*/

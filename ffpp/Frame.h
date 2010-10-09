#ifndef FRAME_H_
#define FRAME_H_

#include <boost/shared_ptr.hpp>

extern "C" {
	#include <libavformat/avformat.h>
}

namespace ffpp
{

class Frame
{
	protected:
		boost::shared_ptr<AVFrame> avp;
		long long pts;
	
		static void Destruct(AVFrame* _avp);

	public:
		Frame();
	
		AVFrame* get();
		const AVFrame* get() const;

		float GetQuality() const;
		void SetQuality(float qscale);

		long long GetPTS();
		void SetPTS(long long pts);
		
		unsigned char Get8Pixel(int x, int y) const;
};

}

#endif /*FRAME_H_*/

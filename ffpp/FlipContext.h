#ifndef FLIPCONTEXT_H_
#define FLIPCONTEXT_H_

extern "C" {
	#include <libavutil/avutil.h>
}

#include "Frame.h"

namespace ffpp
{

class FlipContext
{
	protected:
		int _imgWidth;
		int _imgHeight;
		PixelFormat _format;
		bool _hFlip;
		bool _vFlip;
		
		void HFlip(Frame& frame) const;
		
		void VFlip(Frame& frame) const;

	public:
		FlipContext(int imgWidth, int imgHeight, PixelFormat format, bool hFlip = false, bool vFlip = false);

		void Flip(Frame& frame) const;
		
};

}

#endif /*FLIPCONTEXT_H_*/

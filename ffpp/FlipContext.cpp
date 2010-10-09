#include "FlipContext.h"
#include <stdexcept>
#include <iostream>
#include <cstring>

extern "C" {
#include "PixelFormat.h"
#include <libavutil/mem.h>
}

namespace ffpp
{

FlipContext::FlipContext(int imgWidth, int imgHeight, PixelFormat format, bool hFlip, bool vFlip)
	: _imgWidth(imgWidth), _imgHeight(imgHeight), _format(format), _hFlip(hFlip), _vFlip(vFlip)
{
}

void FlipContext::Flip(Frame& frame) const
{
	if (_hFlip)
		HFlip(frame);
	if (_vFlip)
		VFlip(frame);
}

void FlipContext::VFlip(Frame& frame) const
{
	// get the pixel format
	const PixFmtInfo* pinfo = &pix_fmt_info[_format];

	// ensure that we only handle formats that we can
	if (PIX_FMT_XVMC_MPEG2_MC == _format || PIX_FMT_XVMC_MPEG2_IDCT == _format)
		throw std::runtime_error("Cannot handle XVMC format!");

	// get the number of channels (depends on the pixel type)
	int nChannels = pinfo->pixel_type == FF_PIXEL_PLANAR ? pinfo->nb_channels : 1;
	
	// compute height for chroma channels
	int chromaHeight = _imgHeight;
	if (FF_COLOR_YUV == pinfo->color_type)
		chromaHeight = (_imgHeight + (1 << pinfo->y_chroma_shift) - 1) >> pinfo->y_chroma_shift;

	// iterate over channels and swap lines
	for (int iChannel = 0; iChannel < nChannels; ++iChannel) {
		// allocate swap memory for current channel
		int linesize = frame.get()->linesize[iChannel];
		uint8_t* ptmp = (uint8_t*)malloc(linesize);

		// get the height for the current channel
		int height = _imgHeight;
		if (iChannel > 0 && iChannel < 3)
			height = chromaHeight;

		// swap lines
		uint8_t* pTop = frame.get()->data[iChannel];
		uint8_t* pBottom = frame.get()->data[iChannel] + (height - 1) * linesize;
		for (; pTop < pBottom; pTop += linesize, pBottom -= linesize) {
			memcpy(ptmp, pTop, linesize);
			memcpy(pTop, pBottom, linesize);
			memcpy(pBottom, ptmp, linesize);
		}

		// free swap memory
		free(ptmp);
	}
}

void FlipContext::HFlip(Frame& frame) const
{
	// get the pixel format
	const PixFmtInfo* pinfo = &pix_fmt_info[_format];

	// ensure that we have formats with a pixel size based on 8
	int bbp = pinfo->color_type == FF_PIXEL_PLANAR ? pinfo->depth : pinfo->depth * pinfo->nb_channels;
	if ((bbp % 8) != 0 || PIX_FMT_XVMC_MPEG2_MC == _format || PIX_FMT_XVMC_MPEG2_IDCT == _format)
		throw std::runtime_error("pixel depth needs to be a multiple of 8 bit!");

	// all planar formats
	if (FF_PIXEL_PLANAR == pinfo->pixel_type) {
		// compute the width/height for chroma channels
		int chromaWidth = _imgWidth;
		int chromaHeight = _imgHeight;
		if (FF_COLOR_YUV == pinfo->color_type) {
			chromaWidth = (_imgWidth + (1 << pinfo->x_chroma_shift) - 1) >> pinfo->x_chroma_shift;
			chromaHeight = (_imgHeight + (1 << pinfo->y_chroma_shift) - 1) >> pinfo->y_chroma_shift;
		}

		// iterate over all channels
		for (int iChannel = 0; iChannel < pinfo->nb_channels; ++iChannel) {
			// get the correct height/width for the current channel 
			int height = _imgHeight;
			int width = _imgWidth;
			if (iChannel > 0 && iChannel < 3) {
				height = chromaHeight;
				width = chromaWidth;
			}

			// iterate over all lines
			uint8_t* pLine = frame.get()->data[iChannel];
			for (int iLine = 0; iLine < height; ++iLine, pLine += frame.get()->linesize[iChannel]) {
				// iterate over all pixels in the line
				uint8_t* pLeft = pLine + 0;
				uint8_t* pRight = pLine + width - 1;
				for (; pLeft < pRight; ++pLeft, --pRight)
					// swap pixels
					std::swap(*pLeft, *pRight);
			}
		}
	}
	// packed formats or palettes
	else {
		int height = _imgHeight;
		int width = _imgWidth;
		int stride = pinfo->pixel_type == FF_PIXEL_PALETTE ? 1 : pinfo->nb_channels;

		// iterate over all lines
		uint8_t* pLine = frame.get()->data[0];
		for (int iLine = 0; iLine < height; ++iLine, pLine += frame.get()->linesize[0]) {
			// iterate over all pixels in the line
			uint8_t* pLeft = pLine;
			uint8_t* pRight = pLine + width - stride;
			for (; pLeft < pRight; pLeft += stride, pRight -= stride)
				// iterate over all channels
				for (int iChannel = 0; iChannel < stride; ++iChannel) {
					// swap pixels
					std::swap(*(pLeft + iChannel), *(pRight + iChannel));
				}
		}
	}
}

}


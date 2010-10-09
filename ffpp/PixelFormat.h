#ifndef PIXELFORMAT_H_
#define PIXELFORMAT_H_

#include <libavutil/avutil.h>

/* copied from libavcodec/imgconvert.c .. this is useful for pixel access :) */

#define FF_COLOR_RGB	  0 /**< RGB color space */
#define FF_COLOR_GRAY	  1 /**< gray color space */
#define FF_COLOR_YUV	  2 /**< YUV color space. 16 <= Y <= 235, 16 <= U, V <= 240 */
#define FF_COLOR_YUV_JPEG 3 /**< YUV color space. 0 <= Y <= 255, 0 <= U, V <= 255 */

#define FF_PIXEL_PLANAR   0 /**< each channel has one component in AVPicture */
#define FF_PIXEL_PACKED   1 /**< only one components containing all the channels */
#define FF_PIXEL_PALETTE  2  /**< one components containing indexes for a palette */

typedef struct PixFmtInfo {
	const char *name;
	uint8_t nb_channels;      /**< number of channels (including alpha) */
	uint8_t color_type;       /**< color type (see FF_COLOR_xxx constants) */
	uint8_t pixel_type;       /**< pixel storage type (see FF_PIXEL_xxx constants) */
	uint8_t is_alpha : 1;     /**< true if alpha can be specified */
	uint8_t x_chroma_shift;   /**< X chroma subsampling factor is 2 ^ shift */
	uint8_t y_chroma_shift;   /**< Y chroma subsampling factor is 2 ^ shift */
	uint8_t depth;            /**< bit depth of the color components */
} PixFmtInfo;


/* this table gives more information about formats */
extern const PixFmtInfo pix_fmt_info[];


#endif


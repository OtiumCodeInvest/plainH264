#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/time.h"

namespace NewDec {

// Enumerate the type of video bitstream which is provided to decoder
typedef enum{
	VIDEO_BITSTREAM_AVC=0,
	VIDEO_BITSTREAM_SVC=1,
	VIDEO_BITSTREAM_DEFAULT=VIDEO_BITSTREAM_SVC
} VIDEO_BITSTREAM_TYPE;

// Define a new struct to show the property of video bitstream.
typedef struct{
	unsigned int          size;				// size of the struct
	VIDEO_BITSTREAM_TYPE  eVideoBsType;		// video stream type (AVC/SVC)
} SVideoProperty;

// Enumerate the type of error concealment methods
typedef enum{
	ERROR_CON_DISABLE=0,
	ERROR_CON_FRAME_COPY,
	ERROR_CON_SLICE_COPY,
	ERROR_CON_FRAME_COPY_CROSS_IDR,
	ERROR_CON_SLICE_COPY_CROSS_IDR,
	ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE,
	ERROR_CON_SLICE_MV_COPY_CROSS_IDR,
	ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE
} ERROR_CON_IDC;

// SVC Decoding Parameters, reserved here and potential applicable in the future
typedef struct TagSVCDecodingParam{
	char* pFileNameRestructed;				// file name of reconstructed frame used for PSNR calculation based debug
	unsigned int uiCpuLoad;  				// CPU load
	unsigned char uiTargetDqLayer;			// setting target dq layer id
	bool bParseOnly;          				// decoder for parse only, no reconstruction. When it is true, SPS/PPS size should not exceed SPS_PPS_BS_SIZE (128). Otherwise, it will return error info
	SVideoProperty sVideoProperty;			// video stream property
} SDecodingParam,* PDecodingParam;

// Enumerate the type of video format
typedef enum{
	videoFormatRGB=1,					  	// rgb color formats
	videoFormatRGBA=2,
	videoFormatRGB555=3,
	videoFormatRGB565=4,
	videoFormatBGR=5,
	videoFormatBGRA=6,
	videoFormatABGR=7,
	videoFormatARGB=8,
	videoFormatYUY2=20,						// yuv color formats
	videoFormatYVYU=21,
	videoFormatUYVY=22,
	videoFormatI420=23, 					// the same as IYUV
	videoFormatYV12=24,
	videoFormatInternal=25,					// only used in SVC decoder testbed
	videoFormatNV12=26, 					// new format for output by DXVA decoding
	videoFormatVFlip=0x80000000
} EVideoFormatType;

// Option types introduced in decoder application
typedef enum{
	DECODER_OPTION_END_OF_STREAM=1,			// end of stream flag
	DECODER_OPTION_VCL_NAL,    				// feedback whether or not have VCL NAL in current AU for application layer
	DECODER_OPTION_TEMPORAL_ID,				// feedback temporal id for application layer
	DECODER_OPTION_FRAME_NUM,				// feedback current decoded frame number
	DECODER_OPTION_IDR_PIC_ID, 				// feedback current frame belong to which IDR period
	DECODER_OPTION_LTR_MARKING_FLAG,		// feedback wether current frame mark a LTR
	DECODER_OPTION_LTR_MARKED_FRAME_NUM,	// feedback frame num marked by current Frame
	DECODER_OPTION_ERROR_CON_IDC,			// indicate decoder error concealment method
	DECODER_OPTION_TRACE_LEVEL,
	DECODER_OPTION_TRACE_CALLBACK,			// a void (*)(void* context, int level, const char* message) function which receives log messages
	DECODER_OPTION_TRACE_CALLBACK_CONTEXT,	// context info of trace callbac

	DECODER_OPTION_GET_STATISTICS,			// feedback decoder statistics
	DECODER_OPTION_GET_SAR_INFO,			// feedback decoder Sample Aspect Ratio info in Vui
	DECODER_OPTION_PROFILE,    				// get current AU profile info, only is used in GetOption
	DECODER_OPTION_LEVEL,      				// get current AU level info,only is used in GetOption
	DECODER_OPTION_STATISTICS_LOG_INTERVAL,	// set log output interval
	DECODER_OPTION_IS_REF_PIC,  			// feedback current frame is ref pic or not
	DECODER_OPTION_NUM_OF_FRAMES_REMAINING_IN_BUFFER,// number of frames remaining in decoder buffer when pictures are required to re-ordered into display-order.
	DECODER_OPTION_NUM_OF_THREADS,			// number of decoding threads. The maximum thread count is equal or less than lesser of (cpu core counts and 16).
} DECODER_OPTION;

class ISVCDecoderBase{
	public:
		virtual ~ISVCDecoderBase(){}
		virtual bool DecodeFrame(const unsigned char* pSrc,const int iSrcLen,unsigned char** ppDst)=0;
		virtual bool Initialize(const SDecodingParam* pParam)=0;
		virtual void GetProfileDisplay(std::vector<DisplayTimer>* profilerDisplay)=0;
		virtual uint32_t Width()const=0;
		virtual uint32_t Height()const=0;
		virtual uint32_t StrideY()const=0;
		virtual uint32_t StrideUV()const=0;
};
long CreateDecoder(ISVCDecoderBase** ppDecoder);
};
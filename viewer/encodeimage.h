#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/time.h"

//Enumerate return type
enum CM_RETURN{
	cmResultSuccess, 			// successful
	cmInitParaError, 			// parameters are invalid
	cmUnknownReason,
	cmMallocMemeError,			// malloc a memory error
	cmInitExpected, 			// initial action is expected
	cmUnsupportedData
};

//Enumerate the type of video format
enum EVideoFormatType{
	videoFormatRGB=1, 			// rgb color formats
	videoFormatRGBA=2,
	videoFormatRGB555=3,
	videoFormatRGB565=4,
	videoFormatBGR=5,
	videoFormatBGRA=6,
	videoFormatABGR=7,
	videoFormatARGB=8,
	videoFormatYUY2=20, 		// yuv color formats
	videoFormatYVYU=21,
	videoFormatUYVY=22,
	videoFormatI420=23, 		// the same as IYUV
	videoFormatYV12=24,
	videoFormatInternal=25, 	// only used in SVC decoder testbed
	videoFormatNV12=26, 		// new format for output by DXVA decoding
	videoFormatVFlip=0x80000000
};

//brief Structure for source picture
struct SSourcePicture{
	int iStride[4]; 			// stride for each plane pData
	unsigned char* pData[4];	// plane pData
	int iPicWidth; 				// luma picture width in x coordinate
	int iPicHeight; 			// luma picture height in y coordinate
	long long uiTimeStamp; 		// timestamp of the source picture, unit: millisecond
};

//Enumerate video frame type
enum EVideoFrameType{
	videoFrameTypeInvalid=0,		// encoder not ready or parameters are invalidate
	videoFrameTypeIDR,			// IDR frame in H.264
	//videoFrameTypeI, 			// I frame type
	videoFrameTypeP, 			// P frame type
	//videoFrameTypeSkip,			// skip the frame based encoder kernel
	//videoFrameTypeIPMixed		// a frame where I and P slices are mixing, not supported yet
};

//Bitstream inforamtion of a layer being encoded
struct SLayerBSInfo{
	unsigned char uiTemporalId;
	unsigned char uiSpatialId;
	unsigned char uiQualityId;
	EVideoFrameType eFrameType;
// The sub sequence layers are ordered hierarchically based on their dependency on each other so that any picture in a layer shall not be predicted from any picture on any higher layer.
	//int iSubSeqId; 				// refer to D.2.11 Sub-sequence information SEI message semantics
	int iNalCount; 				// count number of NAL coded already
	int* pNalLengthInByte;		// length of NAL size in byte from 0 to iNalCount-1
	unsigned char* pBsBuf;		// buffer of bitstream contained
};

#define MAX_LAYER_NUM_OF_FRAME_ENC 4		//Was 128

// Frame bit stream info
struct SFrameBSInfo{
	int iLayerNum;
	SLayerBSInfo sLayerInfo[MAX_LAYER_NUM_OF_FRAME_ENC];
	EVideoFrameType eFrameType;
	long long uiTimeStamp;
};

// Enumerate the type of rate control mode
enum RC_MODES{
	RC_QUALITY_MODE=0,			// quality mode
	RC_BITRATE_MODE=1			//bitrate mode
};

// SVC Encoding Parameters
struct SEncParamBase{
	int iPicWidth;				// width of picture in luminance samples (the maximum of all layers if multiple spatial layers presents)
	int iPicHeight;				// height of picture in luminance samples((the maximum of all layers if multiple spatial layers presents)
	int iTargetBitrate;			// target bitrate desired, in unit of bps
	RC_MODES iRCMode;			// rate control mode
	float fMaxFrameRate;		// maximal input frame rate
};

class ISVCEncoder {
	public:
		virtual int Initialize(const SEncParamBase& param)=0;
		virtual const uint8_t* EncodeFrame(uint32_t* byteSize,const SSourcePicture* pSrcPic)=0;
		virtual void GetProfileDisplay(std::vector<DisplayTimer>* profilerDisplay)=0;
		virtual ~ISVCEncoder(){}
};

ISVCEncoder* CreateSVCEncoder();
void DestroySVCEncoder(ISVCEncoder* pEncoder);

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <psapi.h>
#define ALIGNED_DECLARE( type,var,n ) __declspec(align(n)) type var
#else
#define ALIGNED_DECLARE( type, var, n ) type var __attribute__((aligned(n)))
#endif

#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/time.h"
#include "shared/output.h"

#include "decodeimage.h"

namespace NewDec{

#define WELS_CHECK_SE_BOTH_WARNING(val,lower_bound,upper_bound,syntax_name) do {\
if ((val < lower_bound) || (val > upper_bound)) {\
	uprintf("invalid syntax " syntax_name " %d",val);\
}\
}while(0)

// define macros to check syntax elements
#define WELS_CHECK_SE_BOTH_ERROR(val,lower_bound,upper_bound,syntax_name,ret_code) do {\
if ((val < lower_bound) || (val > upper_bound)) {\
	 FATAL("invalid syntax " syntax_name " %d",val);\
	 return ret_code;\
}\
}while(0)

#define WELS_CHECK_SE_UPPER_ERROR_NOLOG(val,upper_bound,syntax_name,ret_code) do {\
if (val > upper_bound) {\
	return ret_code;\
}\
}while(0)

#define WELS_CHECK_SE_UPPER_ERROR(val,upper_bound,syntax_name,ret_code) do {\
if (val > upper_bound) {\
	FATAL("invalid syntax " syntax_name " %d",val);\
	return ret_code;\
}\
}while(0)

// for log2_max_frame_num_minus4
#define LOG2_MAX_FRAME_NUM_OFFSET 4
// for log2_max_pic_order_cnt_lsb_minus4
#define LOG2_MAX_PIC_ORDER_CNT_LSB_OFFSET 4
// for pic_width_in_mbs_minus1
#define PIC_WIDTH_IN_MBS_OFFSET 1
// for pic_height_in_map_units_minus1
#define PIC_HEIGHT_IN_MAP_UNITS_OFFSET 1
// for num_slice_groups_minus1
#define NUM_SLICE_GROUPS_OFFSET 1
// for run_length_minus1
#define RUN_LENGTH_OFFSET 1
// for num_ref_idx_l0_default_active_minus1 and num_ref_idx_l1_default_active_minus1
#define NUM_REF_IDX_L0_DEFAULT_ACTIVE_OFFSET 1
#define NUM_REF_IDX_L1_DEFAULT_ACTIVE_OFFSET 1
// for pic_init_qp_minus26 and pic_init_qs_minus26
#define PIC_INIT_QP_OFFSET 26
#define PIC_INIT_QS_OFFSET 26

// From Level 5.2
#define MAX_MB_SIZE 36864

#define GENERATE_ERROR_NO(iErrLevel,iErrInfo) ((iErrLevel << 16) | (iErrInfo & 0xFFFF))
#define ERR_INVALID_INTRA4X4_MODE -1

// ERR_LEVEL
// -----------------------------------------------------------------------------------------------------------
enum{
	ERR_LEVEL_ACCESS_UNIT=1,
	ERR_LEVEL_NAL_UNIT_HEADER,
	ERR_LEVEL_PREFIX_NAL,
	ERR_LEVEL_PARAM_SETS,
	ERR_LEVEL_SLICE_HEADER,
	ERR_LEVEL_SLICE_DATA,
	ERR_LEVEL_MB_DATA
};

enum{
	ERR_INFO_INVALID_ACCESS,
	ERR_INFO_READ_OVERFLOW,
	ERR_INFO_READ_LEADING_ZERO,
	ERR_INFO_NO_PARAM_SETS, 	// No SPS and/ PPS before sequence header
	ERR_INFO_SPS_ID_OVERFLOW,
	ERR_INFO_PPS_ID_OVERFLOW,
	ERR_INFO_INVALID_POC_TYPE,
	ERR_INFO_INVALID_SLICEGROUP,
	ERR_INFO_INVALID_IDR_PIC_ID,
	ERR_INFO_INVALID_MAX_NUM_REF_FRAMES,
	ERR_INFO_INVALID_MAX_MB_SIZE,
	ERR_INFO_INVALID_FIRST_MB_IN_SLICE,
	ERR_INFO_INVALID_SLICE_ALPHA_C0_OFFSET_DIV2,
	ERR_INFO_INVALID_SLICE_BETA_OFFSET_DIV2,
	ERR_INFO_INVALID_CABAC_INIT_IDC,
	ERR_INFO_INVALID_QP,
	ERR_INFO_INVALID_PIC_INIT_QS,
	ERR_INFO_INVALID_CHROMA_QP_INDEX_OFFSET,
	ERR_INFO_INVALID_PIC_INIT_QP,
	ERR_INFO_INVALID_LOG2_MAX_FRAME_NUM_MINUS4,
	ERR_INFO_INVALID_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4,
	ERR_INFO_INVALID_NUM_REF_FRAME_IN_PIC_ORDER_CNT_CYCLE,
	ERR_INFO_INVALID_SUB_MB_TYPE,
	ERR_INFO_INVALID_I4x4_PRED_MODE,
	ERR_INFO_INVALID_I16x16_PRED_MODE,
	ERR_INFO_INVALID_I_CHROMA_PRED_MODE,
	ERR_INFO_UNSUPPORTED_NON_BASELINE,
	ERR_INFO_UNSUPPORTED_FMOTYPE,
	ERR_INFO_UNSUPPORTED_MBAFF,
	ERR_INFO_REFERENCE_PIC_LOST,
	ERR_INFO_INVALID_SLICE_TYPE,
	ERR_INFO_INVALID_REF_REORDERING,
	// for CABAC
	ERR_CABAC_NO_BS_TO_READ,
	ERR_CABAC_UNEXPECTED_VALUE,
	ERR_INFO_MB_RECON_FAIL
};

#ifndef WELS_CEIL
#define WELS_CEIL(x) ceil(x)		// FIXME: low complexity instead of math library used
#endif// WELS_CEIL

#ifndef WELS_FLOOR
#define WELS_FLOOR(x) floor(x)		// FIXME: low complexity instead of math library used
#endif// WELS_FLOOR

#ifndef WELS_ROUND
#define WELS_ROUND(x) ((int32_t)(0.5+(x)))
#endif// WELS_ROUND

#ifndef WELS_ROUND64
#define WELS_ROUND64(x) ((int64_t)(0.5+(x)))
#endif// WELS_ROUND

#ifndef WELS_DIV_ROUND
#define WELS_DIV_ROUND(x,y) ((int32_t)((y)==0?((x)/((y)+1)):(((y)/2+(x))/(y))))
#endif// WELS_DIV_ROUND

#ifndef WELS_DIV_ROUND64
#define WELS_DIV_ROUND64(x,y) ((int64_t)((y)==0?((x)/((y)+1)):(((y)/2+(x))/(y))))
#endif// WELS_DIV_ROUND64

#ifndef WELS_ALIGN
#define WELS_ALIGN(x,n) (((x)+(n)-1)&~((n)-1))
#endif// WELS_ALIGN

#ifndef WELS_MAX
#define WELS_MAX(x,y) ((x) > (y) ? (x) : (y))
#endif// WELS_MAX

#ifndef WELS_MIN
#define WELS_MIN(x,y) ((x) < (y) ? (x) : (y))
#endif// WELS_MIN

#ifndef WELS_MIN_POSITIVE
#define WELS_MIN_POSITIVE(x,y) (x >=0 && y >=0) ? WELS_MIN(x,y) : WELS_MAX(x,y);
#endif// WELS_MIN_POSITIVE

#ifndef NEG_NUM
// #define NEG_NUM( num ) (-num)
#define NEG_NUM(iX) (1+(~(iX)))
#endif// NEG_NUM

#define WELS_NON_ZERO_COUNT_AVERAGE(nC,nA,nB) { \
 nC=nA+nB+1; \
 nC >>=(uint8_t)( nA !=-1 && nB !=-1); \
 nC+=(uint8_t)(nA==-1 && nB==-1); \
}

static inline uint8_t Clip1(int32_t iX){
	uint8_t uiTmp=(uint8_t)(((iX)&~255) ? (-(iX)>>31) : (iX));
	return uiTmp;
}

#ifndef WELS_SIGN
#define WELS_SIGN(iX) ((int32_t)(iX) >> 31)
#endif		// WELS_SIGN
#ifndef WELS_ABS
#if 1
#define WELS_ABS(iX) ((iX)>0 ? (iX) :-(iX))
#else
#define WELS_ABS(iX) ((WELS_SIGN(iX) ^ (int32_t)(iX))-WELS_SIGN(iX))
#endif
#endif		// WELS_ABS

// WELS_CLIP3
#ifndef WELS_CLIP3
#define WELS_CLIP3(iX,iY,iZ) ((iX) < (iY) ? (iY) : ((iX) > (iZ) ? (iZ) : (iX)))
#endif		// WELS_CLIP3


#define LD16(a) (*((uint16_t*)(a)))
#define LD32(a) (*((uint32_t*)(a)))
#define LD64(a) (*((uint64_t*)(a)))

#define ST16(a,b) *((uint16_t*)(a))=(b)
#define ST32(a,b) *((uint32_t*)(a))=(b)
#define ST64(a,b) *((uint64_t*)(a))=(b)
#define LD16A2 LD16
#define LD32A2 LD32
#define LD32A4 LD32
#define LD64A2 LD64
#define LD64A4 LD64
#define LD64A8 LD64
#define ST16A2 ST16
#define ST32A2 ST32
#define ST32A4 ST32
#define ST64A2 ST64
#define ST64A4 ST64
#define ST64A8 ST64

static inline int32_t Median(int32_t iX,int32_t iY,int32_t iZ) {
	int32_t iMin=iX,iMax=iX;
	if(iY<iMin)
		iMin=iY;
	else
		iMax=iY;
	if(iZ<iMin)
		iMin=iZ;
	else
	if(iZ>iMax)
		iMax=iZ;
	return (iX+iY+iZ)-(iMin+iMax);
}

// ENFORCE_STACK_ALIGN_1D: force 1 dimension local data aligned in stack
// _tp: type
// _nm: var name
// _sz: size
// _al: align bytes
// auxiliary var: _nm ## _tEmP
#define ENFORCE_STACK_ALIGN_1D(_tp,_nm,_sz,_al) \
	_tp _nm ## _tEmP[(_sz)+(_al)-1]; \
	_tp *_nm=_nm ## _tEmP+((_al)-1)-(((uintptr_t)(_nm ## _tEmP+((_al)-1)) & ((_al)-1))/sizeof(_tp));


#define ERR_NONE 0

struct SDataBuffer{
	uint8_t* pHead;
	uint8_t* pEnd;
	uint8_t* pCurPos;
};

// NAL Unit Type (5 Bits)
enum ENalUnitType{
	NAL_UNIT_UNSPEC_0=0,
	NAL_UNIT_CODED_SLICE=1,
	NAL_UNIT_CODED_SLICE_DPA=2,
	NAL_UNIT_CODED_SLICE_DPB=3,
	NAL_UNIT_CODED_SLICE_DPC=4,
	NAL_UNIT_CODED_SLICE_IDR=5,
	NAL_UNIT_SEI=6,
	NAL_UNIT_SPS=7,
	NAL_UNIT_PPS=8,
	NAL_UNIT_AU_DELIMITER=9,
	NAL_UNIT_END_OF_SEQ=10,
	NAL_UNIT_END_OF_STR=11,
	NAL_UNIT_FILLER_DATA=12,
	NAL_UNIT_SPS_EXT=13,
	NAL_UNIT_PREFIX=14,
	NAL_UNIT_SUBSET_SPS=15,
	NAL_UNIT_DEPTH_PARAM=16,// NAL_UNIT_RESV_16
	NAL_UNIT_RESV_17=17,
	NAL_UNIT_RESV_18=18,
	NAL_UNIT_AUX_CODED_SLICE=19,
	NAL_UNIT_CODED_SLICE_EXT=20,
	NAL_UNIT_MVC_SLICE_EXT=21,// NAL_UNIT_RESV_21
	NAL_UNIT_RESV_22=22,
	NAL_UNIT_RESV_23=23,
	NAL_UNIT_UNSPEC_24=24,
	NAL_UNIT_UNSPEC_25=25,
	NAL_UNIT_UNSPEC_26=26,
	NAL_UNIT_UNSPEC_27=27,
	NAL_UNIT_UNSPEC_28=28,
	NAL_UNIT_UNSPEC_29=29,
	NAL_UNIT_UNSPEC_30=30,
	NAL_UNIT_UNSPEC_31=31
};

enum ENalRefIdc{
	NRI_PRI_LOWEST=0,
	NRI_PRI_LOW=1,
	NRI_PRI_HIGH=2,
	NRI_PRI_HIGHEST=3
};

enum EVclType{
	NON_VCL=0,
	VCL=1,
	NOT_APP=2
};

const EVclType g_keTypeMap[32][2]={
	{NON_VCL,NON_VCL},		// 0: NAL_UNIT_UNSPEC_0
	{VCL,VCL,},				// 1: NAL_UNIT_CODED_SLICE
	{VCL,NOT_APP},			// 2: NAL_UNIT_CODED_SLICE_DPA
	{VCL,NOT_APP},			// 3: NAL_UNIT_CODED_SLICE_DPB
	{VCL,NOT_APP},			// 4: NAL_UNIT_CODED_SLICE_DPC
	{VCL,VCL},				// 5: NAL_UNIT_CODED_SLICE_IDR
	{NON_VCL,NON_VCL},		// 6: NAL_UNIT_SEI
	{NON_VCL,NON_VCL},		// 7: NAL_UNIT_SPS
	{NON_VCL,NON_VCL},		// 8: NAL_UNIT_PPS
	{NON_VCL,NON_VCL},		// 9: NAL_UNIT_AU_DELIMITER
	{NON_VCL,NON_VCL},		// 10: NAL_UNIT_END_OF_SEQ
	{NON_VCL,NON_VCL},		// 11: NAL_UNIT_END_OF_STR
	{NON_VCL,NON_VCL},		// 12: NAL_UNIT_FILLER_DATA
	{NON_VCL,NON_VCL},		// 13: NAL_UNIT_SPS_EXT
	{NON_VCL,NON_VCL},		// 14: NAL_UNIT_PREFIX,NEED associate succeeded NAL to make a VCL
	{NON_VCL,NON_VCL},		// 15: NAL_UNIT_SUBSET_SPS
	{NON_VCL,NON_VCL},		// 16: NAL_UNIT_DEPTH_PARAM
	{NON_VCL,NON_VCL},		// 17: NAL_UNIT_RESV_17
	{NON_VCL,NON_VCL},		// 18: NAL_UNIT_RESV_18
	{NON_VCL,NON_VCL},		// 19: NAL_UNIT_AUX_CODED_SLICE
	{NON_VCL,VCL},			// 20: NAL_UNIT_CODED_SLICE_EXT
	{NON_VCL,NON_VCL},		// 21: NAL_UNIT_MVC_SLICE_EXT
	{NON_VCL,NON_VCL},		// 22: NAL_UNIT_RESV_22
	{NON_VCL,NON_VCL},		// 23: NAL_UNIT_RESV_23
	{NON_VCL,NON_VCL},		// 24: NAL_UNIT_UNSPEC_24
	{NON_VCL,NON_VCL},		// 25: NAL_UNIT_UNSPEC_25
	{NON_VCL,NON_VCL},		// 26: NAL_UNIT_UNSPEC_26
	{NON_VCL,NON_VCL},		// 27: NAL_UNIT_UNSPEC_27
	{NON_VCL,NON_VCL},		// 28: NAL_UNIT_UNSPEC_28
	{NON_VCL,NON_VCL},		// 29: NAL_UNIT_UNSPEC_29
	{NON_VCL,NON_VCL},		// 30: NAL_UNIT_UNSPEC_30
	{NON_VCL,NON_VCL}		// 31: NAL_UNIT_UNSPEC_31
};

#define IS_VCL_NAL(t,ext_idx) (g_keTypeMap[t][ext_idx]==VCL)
#define IS_PARAM_SETS_NALS(t) ( (t)==NAL_UNIT_SPS || (t)==NAL_UNIT_PPS || (t)==NAL_UNIT_SUBSET_SPS )
#define IS_SPS_NAL(t) ( (t)==NAL_UNIT_SPS )
#define IS_SUBSET_SPS_NAL(t) ( (t)==NAL_UNIT_SUBSET_SPS )
#define IS_PPS_NAL(t) ( (t)==NAL_UNIT_PPS )
#define IS_SEI_NAL(t) ( (t)==NAL_UNIT_SEI )
#define IS_AU_DELIMITER_NAL(t) ( (t)==NAL_UNIT_AU_DELIMITER )
#define IS_PREFIX_NAL(t) ( (t)==NAL_UNIT_PREFIX )
#define IS_SUBSET_SPS_USED(t) ( (t)==NAL_UNIT_SUBSET_SPS || (t)==NAL_UNIT_CODED_SLICE_EXT )
#define IS_VCL_NAL_AVC_BASE(t) ( (t)==NAL_UNIT_CODED_SLICE || (t)==NAL_UNIT_CODED_SLICE_IDR )
#define IS_NEW_INTRODUCED_SVC_NAL(t) ( (t)==NAL_UNIT_PREFIX || (t)==NAL_UNIT_CODED_SLICE_EXT )

enum ESliceType{
	P_SLICE=0,
	I_SLICE=2
};

// List Index
enum EListIndex{
	LIST_0=0,
	LIST_1=1,
	LIST_A=2
};

// Motion Vector components
enum EMvComp{
	MV_X=0,
	MV_Y=1,
	MV_A=2
};

#define MB_SUB_PARTITION_SIZE 4						// Sub partition size in a 8x8 sub-block
#define MAX_PPS_COUNT 1								// Count number of PPS
#define MAX_REF_PIC_COUNT 2							// MAX Short+Long reference pictures
#define MAX_SLICEGROUP_IDS 8						// Count number of Slice Groups

// MB width in pixels for specified colorspace I420 usually used in codec
#define MB_WIDTH_LUMA 16
#define MB_WIDTH_CHROMA (MB_WIDTH_LUMA>>1)
// MB height in pixels for specified colorspace I420 usually used in codec
#define MB_HEIGHT_LUMA 16
#define MB_HEIGHT_CHROMA (MB_HEIGHT_LUMA>>1)
#define MB_COEFF_LIST_SIZE (256+((MB_WIDTH_CHROMA*MB_HEIGHT_CHROMA)<<1))
#define MB_PARTITION_SIZE 4							// Macroblock partition size in 8x8 sub-blocks
#define MB_BLOCK4x4_NUM 16
#define BASE_QUALITY_ID 0

struct SPicture{
	uint8_t* pBuffer[4];							// pointer to the first allocated byte,basical offset of buffer,dimension:
	uint8_t* pData[4];								// pointer to picture planes respectively
	int32_t iLinesize[4];							// linesize of picture planes respectively used currently
	ESliceType eSliceType;
	uint32_t* pMbType;								// mb type used for direct mode
	int16_t(*pMv[LIST_A])[MB_BLOCK4x4_NUM][MV_A];	// used for direct mode
	int8_t(*pRefIndex[LIST_A])[MB_BLOCK4x4_NUM];	// used for direct mode
};

typedef uint8_t ProfileIdc;

// brief Enumerate the type of profile id
typedef enum{
	PRO_UNKNOWN=0,
	PRO_BASELINE=66,
	PRO_MAIN=77,
	PRO_EXTENDED=88,
	PRO_HIGH=100,
	PRO_HIGH10=110,
	PRO_HIGH422=122,
	PRO_HIGH444=144,
	PRO_CAVLC444=244,
	PRO_SCALABLE_BASELINE=83,
	PRO_SCALABLE_HIGH=86
} EProfileIdc;

// brief Enumerate the type of level id
typedef enum{
	LEVEL_UNKNOWN=0,
	LEVEL_1_0=10,
	LEVEL_1_B=9,
	LEVEL_1_1=11,
	LEVEL_1_2=12,
	LEVEL_1_3=13,
	LEVEL_2_0=20,
	LEVEL_2_1=21,
	LEVEL_2_2=22,
	LEVEL_3_0=30,
	LEVEL_3_1=31,
	LEVEL_3_2=32,
	LEVEL_4_0=40,
	LEVEL_4_1=41,
	LEVEL_4_2=42,
	LEVEL_5_0=50,
	LEVEL_5_1=51,
	LEVEL_5_2=52
} ELevelIdc;

#define CTX_NA 0
#define CABAC_CONTEXT_COUNT 460

struct SLevelLimits{
	ELevelIdc uiLevelIdc;					// level idc
	uint32_t uiMaxMBPS;						// Max macroblock processing rate(MB/s)
	uint32_t uiMaxFS;						// Max frame sizea(MBs)
	uint32_t uiMaxDPBMbs;					// Max decoded picture buffer size(MBs)
	uint32_t uiMaxBR;						// Max video bit rate
	uint32_t uiMaxCPB;						// Max CPB size
	int16_t iMinVmv;						// Vertical MV component range upper bound
	int16_t iMaxVmv;						// Vertical MV component range lower bound
	uint16_t uiMinCR;						// Min compression ration
	int16_t iMaxMvsPer2Mb;					// Max number of motion vectors per two consecutive MBs
};

// Sequence Parameter Set,refer to Page 57 in JVT X201wcm
struct SSps{
	uint32_t iMbWidth;
	uint32_t iMbHeight;
	uint32_t uiTotalMbCount;					// used in decode_slice_data()
	uint32_t uiLog2MaxFrameNum;
	uint32_t uiPocType;
// POC type 0
	int32_t iLog2MaxPocLsb;
// POC type 1
	int32_t iOffsetForNonRefPic;
	int32_t iOffsetForTopToBottomField;
	int32_t iNumRefFramesInPocCycle;
	int8_t iOffsetForRefFrame[256];
	int32_t iNumRefFrames;
	ProfileIdc uiProfileIdc;
	uint8_t uiLevelIdc;
	uint8_t uiChromaFormatIdc;
	bool bDeltaPicOrderAlwaysZeroFlag;
	bool bGapsInFrameNumValueAllowedFlag;
	bool bFrameMbsOnlyFlag;
	bool bDirect8x8InferenceFlag;
	bool bQpPrimeYZeroTransfBypassFlag;
	const SLevelLimits* pSLevelLimits;
};

// Picture parameter set syntax,refer to Page 59 in JVT X201wcm
struct SPps{
	int32_t iPicInitQp;
	int32_t iPicInitQs;
	int32_t iChromaQpIndexOffset[2];	// cb,cr
	bool bPicOrderPresentFlag;
	bool bDeblockingFilterControlPresentFlag;
	bool bRedundantPicCntPresentFlag;
	bool bTransform8x8ModeFlag;
};

// AVC MB types
#define MB_TYPE_INTRA4x4 0x00000001
#define MB_TYPE_INTRA16x16 0x00000002
#define MB_TYPE_INTRA8x8 0x00000004
#define MB_TYPE_16x16 0x00000008
#define MB_TYPE_16x8 0x00000010
#define MB_TYPE_8x16 0x00000020
#define MB_TYPE_8x8 0x00000040
#define MB_TYPE_8x8_REF0 0x00000080
#define MB_TYPE_SKIP 0x00000100
#define MB_TYPE_INTRA_PCM 0x00000200
#define MB_TYPE_INTRA_BL 0x00000400
#define MB_TYPE_DIRECT 0x00000800
#define MB_TYPE_P0L0 0x00001000
#define MB_TYPE_P1L0 0x00002000
#define MB_TYPE_P0L1 0x00004000
#define MB_TYPE_P1L1 0x00008000
#define MB_TYPE_L0 (MB_TYPE_P0L0 | MB_TYPE_P1L0)
#define MB_TYPE_L1 (MB_TYPE_P0L1 | MB_TYPE_P1L1)

#define SUB_MB_TYPE_8x8 0x00000001
#define SUB_MB_TYPE_8x4 0x00000002
#define SUB_MB_TYPE_4x8 0x00000004
#define SUB_MB_TYPE_4x4 0x00000008

#define MB_TYPE_INTRA (MB_TYPE_INTRA4x4 | MB_TYPE_INTRA16x16 | MB_TYPE_INTRA8x8 | MB_TYPE_INTRA_PCM)
#define MB_TYPE_INTER (MB_TYPE_16x16 | MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8 | MB_TYPE_8x8_REF0 | MB_TYPE_SKIP | MB_TYPE_DIRECT)
#define IS_INTRA4x4(type) ( MB_TYPE_INTRA4x4==(type) )
#define IS_INTRA8x8(type) ( MB_TYPE_INTRA8x8==(type) )
#define IS_INTRANxN(type) ( MB_TYPE_INTRA4x4==(type) || MB_TYPE_INTRA8x8==(type) )
#define IS_INTRA16x16(type) ( MB_TYPE_INTRA16x16==(type) )
#define IS_INTRA(type) ( (type)&MB_TYPE_INTRA )
#define IS_INTER(type) ( (type)&MB_TYPE_INTER )
#define IS_INTER_16x16(type) ( (type)&MB_TYPE_16x16 )
#define IS_INTER_16x8(type) ( (type)&MB_TYPE_16x8 )
#define IS_INTER_8x16(type) ( (type)&MB_TYPE_8x16 )
#define IS_TYPE_L0(type) ( (type)&MB_TYPE_L0 )
#define IS_TYPE_L1(type) ( (type)&MB_TYPE_L1 )
#define IS_DIR(a,part,list) ((a) & (MB_TYPE_P0L0<<((part)+2*(list))))

#define IS_SKIP(type) ( ((type)&MB_TYPE_SKIP) !=0 )
#define IS_DIRECT(type) ( ((type)&MB_TYPE_DIRECT) !=0 )
#define IS_SVC_INTER(type) IS_INTER(type)
#define IS_I_BL(type) ( (type)==MB_TYPE_INTRA_BL )
#define IS_SVC_INTRA(type) ( IS_I_BL(type) || IS_INTRA(type) )
#define IS_Inter_8x8(type) ( ((type)&MB_TYPE_8x8) !=0)

#define REF_NOT_AVAIL -2
#define REF_NOT_IN_LIST -1			// intra

#define I16_PRED_INVALID -1
#define I16_PRED_V 0
#define I16_PRED_H 1
#define I16_PRED_DC 2
#define I16_PRED_P 3

#define I16_PRED_DC_L 4
#define I16_PRED_DC_T 5
#define I16_PRED_DC_128 6

#define I4_PRED_INVALID 0
#define I4_PRED_V 0
#define I4_PRED_H 1
#define I4_PRED_DC 2
#define I4_PRED_DDL 3				// diagonal_down_left
#define I4_PRED_DDR 4				// diagonal_down_right
#define I4_PRED_VR 5				// vertical_right
#define I4_PRED_HD 6				// horizon_down
#define I4_PRED_VL 7				// vertical_left
#define I4_PRED_HU 8				// horizon_up

#define I4_PRED_DC_L 9
#define I4_PRED_DC_T 10
#define I4_PRED_DC_128 11

#define I4_PRED_DDL_TOP 12			// right-top replacing by padding rightmost pixel of top
#define I4_PRED_VL_TOP 13			// right-top replacing by padding rightmost pixel of top

// intra Chroma
#define C_PRED_DC 0
#define C_PRED_H 1
#define C_PRED_V 2
#define C_PRED_P 3

#define C_PRED_DC_L 4
#define C_PRED_DC_T 5
#define C_PRED_DC_128 6

struct SSlice{
	int32_t iSliceQp;
	int32_t iSliceAlphaC0Offset;
	int32_t iSliceBetaOffset;
	int32_t iCabacInitIdc;
	int32_t iLastMbQp;									// stored qp for last mb coded,maybe more efficient for mb skip detection etc.
	int32_t iLastDeltaQp;
};

struct SNalUnit{
	uint8_t uiNalRefIdc;
	ENalUnitType eNalUnitType;
};

#define PICTURE_RESOLUTION_ALIGNMENT 32

// MB Type & Sub-MB Type
typedef uint32_t MbType;

#define I16_LUMA_DC 1
#define I16_LUMA_AC 2
#define LUMA_DC_AC 3
#define CHROMA_DC 4
#define CHROMA_AC 5
#define LUMA_DC_AC_8 6
#define CHROMA_DC_U 7
#define CHROMA_DC_V 8
#define CHROMA_AC_U 9
#define CHROMA_AC_V 10
#define LUMA_DC_AC_INTRA 11
#define LUMA_DC_AC_INTER 12
#define CHROMA_DC_U_INTER 13
#define CHROMA_DC_V_INTER 14
#define CHROMA_AC_U_INTER 15
#define CHROMA_AC_V_INTER 16
#define LUMA_DC_AC_INTRA_8 17
#define LUMA_DC_AC_INTER_8 18

struct SDqLayer{
	uint32_t* pMbType;
	int16_t(*pMv[LIST_A])[MB_BLOCK4x4_NUM][MV_A];
	int16_t(*pMvd[LIST_A])[MB_BLOCK4x4_NUM][MV_A];
	int8_t(*pRefIndex[LIST_A])[MB_BLOCK4x4_NUM];
	int8_t(*pDirect)[MB_BLOCK4x4_NUM];
	bool* pNoSubMbPartSizeLessThan8x8Flag;
	bool* pTransformSize8x8Flag;
	int8_t* pLumaQp;
	int8_t(*pChromaQp)[2];
	int8_t* pCbp;
	uint16_t* pCbfDc;
	int8_t(*pNzc)[24];
	int8_t(*pNzcRs)[24];
	int16_t(*pScaledTCoeff)[MB_COEFF_LIST_SIZE];
	int8_t(*pIntraPredMode)[8];	// 0~3 top4x4 ; 4~6 left 4x4; 7 intra16x16
	int8_t(*pIntra4x4FinalMode)[MB_BLOCK4x4_NUM];
	uint8_t* pIntraNxNAvailFlag;
	int8_t* pChromaPredMode;
	uint32_t(*pSubMbType)[MB_SUB_PARTITION_SIZE];
	int32_t iLumaStride;
	int32_t iChromaStride;
	uint8_t* pPred[3];
	int32_t iMbX;
	int32_t iMbY;
	int32_t iMbXyIndex;
	int32_t iMbWidth;						// MB width of this picture,equal to sSps.iMbWidth
	int32_t iMbHeight;						// MB height of this picture,equal to sSps.iMbHeight;
	SPicture* pDec;							// reconstruction picture pointer for layer
};

struct SDeblockingFilter{
	uint8_t* pCsData[3];						// pointer to reconstructed picture data
	int32_t iCsStride[2];						// Cs stride
	int8_t iSliceAlphaC0Offset;
	int8_t iSliceBetaOffset;
	int8_t iChromaQP[2];
	int8_t iLumaQP;
	SPicture** pRefPics[LIST_A];
};

const uint8_t g_kuiScan4[16]={						// for mb cache in sMb (only current element,without neighbor)
	// 4*4block scan mb cache order
	0,1,4,5,		// 0 1 | 4 5 0 1 | 2 3
	2,3,6,7,		// 2 3 | 6 7 4 5 | 6 7
	8,9,12,13,		// ----------------->-----------
	10,11,14,15		// 8 9 |12 13 8 9 |10 11
}; 	// 10 11 |14 15 12 13 |14 15

struct SNeighAvail{
	int32_t iTopAvail;
	int32_t iLeftAvail;
	int32_t iRightTopAvail;
	int32_t iLeftTopAvail;							// used for check intra_pred_mode avail or not		// 1: avail; 0: unavail

	int32_t iLeftType;
	int32_t iTopType;
	int32_t iLeftTopType;
	int32_t iRightTopType;

	int8_t iTopCbp;
	int8_t iLeftCbp;
	int8_t iDummy[2];		// for align
};

typedef void (*PMapNeighToSample) (SNeighAvail* pNeighAvail,int32_t* pSampleAvail);
typedef void (*PMap16NeighToSample) (SNeighAvail* pNeighAvail,uint8_t* pSampleAvail);

#define MAX_PRED_MODE_ID_I16x16 3
#define MAX_PRED_MODE_ID_CHROMA 3
#define MAX_PRED_MODE_ID_I4x4 8
#define WELS_QP_MAX 51

struct SCabacCtx{
	uint8_t uiState;
	uint8_t uiMPS;
};

#define NEW_CTX_OFFSET_MB_TYPE_I 3
#define NEW_CTX_OFFSET_SKIP 11
#define NEW_CTX_OFFSET_SUBMB_TYPE 21
#define NEW_CTX_OFFSET_B_SUBMB_TYPE 36
#define NEW_CTX_OFFSET_MVD 40
#define NEW_CTX_OFFSET_REF_NO 54
#define NEW_CTX_OFFSET_DELTA_QP 60
#define NEW_CTX_OFFSET_IPR 68
#define NEW_CTX_OFFSET_CIPR 64
#define NEW_CTX_OFFSET_CBP 73
#define NEW_CTX_OFFSET_CBF 85
#define NEW_CTX_OFFSET_MAP 105
#define NEW_CTX_OFFSET_LAST 166
#define NEW_CTX_OFFSET_ONE 227
#define NEW_CTX_OFFSET_ABS 232
#define NEW_CTX_OFFSET_TS_8x8_FLAG 399
#define CTX_NUM_MVD 7
#define CTX_NUM_CBP 4
// Table 9-34 in Page 270
#define NEW_CTX_OFFSET_TRANSFORM_SIZE_8X8_FLAG 399
#define NEW_CTX_OFFSET_MAP_8x8 402
#define NEW_CTX_OFFSET_LAST_8x8 417
#define NEW_CTX_OFFSET_ONE_8x8 426
#define NEW_CTX_OFFSET_ABS_8x8 431

typedef void (*PGetIntraPredFunc) (uint8_t* pPred,const int32_t kiLumaStride);
typedef void (*PGetIntraPred8x8Func) (uint8_t* pPred,const int32_t kiLumaStride,bool bTLAvail,bool bTRAvail);

#define READ_VERIFY(uiRet) do{\
	uint32_t uiRetTmp=(uint32_t)uiRet;\
	if(uiRetTmp!=ERR_NONE)\
		return uiRetTmp;\
	}while(0)


// Bit-stream auxiliary reading / writing
struct SBitStringAux{
	uint8_t* pStartBuf;							// buffer to start position
	uint8_t* pEndBuf;							// buffer+length
	int32_t iBits;								// count bits of overall bitstreaming input
	uint8_t* pCurBuf;							// current reading position
	uint32_t uiCurBits;
	int32_t iLeftBits;							// count number of available bits left ([1,8]),
	// need pointer to next byte start position in case 0 bit left then 8 instead
};

class SCabacDecEngine{
	public:
		bool InitCabacDecEngineFromBS(SBitStringAux* bitstream);
		void InitBsFromCabacDecEngine(SBitStringAux* bitstream);
		void InitCabac(int32_t iQp,int32_t iModel);
		uint32_t DecodeUEGLevelCabac(uint32_t uiBin,uint32_t& uiCode);
		int32_t DecodeBinCabac(uint32_t uiBin,uint32_t& uiBinVal);
		int32_t DecodeUEGMvCabac(uint32_t uiBin,uint32_t iMaxBin,uint32_t& uiCode);
		int32_t DecodeUnaryBinCabac(uint32_t uiBin,int32_t iCtxOffset,uint32_t& uiSymVal);
		int32_t DecodeBypassCabac(uint32_t& uiBinVal);
		bool DecodeTerminateCabac();
	protected:
		int32_t DecodeUnaryBinCabac(SCabacCtx* pBinCtx,int32_t iCtxOffset,uint32_t& uiSymVal);
		uint32_t DecodeUEGLevelCabac(SCabacCtx* pBinCtx,uint32_t& uiCode);
		int32_t DecodeBinCabac(SCabacCtx* pBinCtx,uint32_t& uiBinVal);
		int32_t DecodeUEGMvCabac(SCabacCtx* pBinCtx,uint32_t iMaxBin,uint32_t& uiCode);
		inline int32_t Read32BitsCabac(uint32_t& uiValue,int32_t& iNumBitsRead);
		int32_t DecodeExpBypassCabac(int32_t iCount,uint32_t& uiSymVal);
		uint64_t m_uiRange=0;
		uint64_t m_uiOffset=0;
		uint8_t* m_pBuffEnd=0;
		uint8_t* m_pBuffCurr=0;
		int32_t m_iBitsLeft=0;
		uint8_t* m_pBuffStart=0;
		SCabacCtx m_cabacCtx[CABAC_CONTEXT_COUNT];
};

struct SDecoderContext{
	SDataBuffer sRawData;
	ESliceType eSliceType;							// Slice type
	int32_t	iDecBlockOffsetArray[24];				// address talbe for sub 4x4 block in intra4x4_mb,so no need to caculta the address every time.
	SSps m_sps;
	SPps m_pps;
	SPicture* pDec; 								// pointer to current picture being reconstructed
	SPicture* pRef; 								// pointer to prev picture
	SPicture* m_pics[2];
	SNalUnit m_nalUnit;
	SSlice m_slice;
	SDqLayer m_layer;								// current DQ layer representation,also carry reference base layer if applicable
	PGetIntraPredFunc pGetI16x16LumaPredFunc[7];	// h264_predict_copy_16x16;
	PGetIntraPredFunc pGetI4x4LumaPredFunc[14];		// h264_predict_4x4_t
	PGetIntraPredFunc pGetIChromaPredFunc[7];		// h264_predict_8x8_t
	PGetIntraPred8x8Func pGetI8x8LumaPredFunc[14];	// Transform8x8
};

#define GET_WORD(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes) {\
	if(iReadBytes>iAllowedBytes+1) {\
		return ERR_INFO_READ_OVERFLOW;\
	}\
	iCurBits|=((uint32_t)((pBufPtr[0]<<8)|pBufPtr[1]))<<(iLeftBits);\
	iLeftBits-=16;\
	pBufPtr+=2;\
}
#define NEED_BITS(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes) {\
	if (iLeftBits>0) { \
		GET_WORD(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes);\
	}\
}
#define UBITS(iCurBits,iNumBits) (iCurBits>>(32-(iNumBits)))
#define DUMP_BITS(iCurBits,pBufPtr,iLeftBits,iNumBits,iAllowedBytes,iReadBytes) {\
	iCurBits<<=(iNumBits);\
	iLeftBits+=(iNumBits);\
	NEED_BITS(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes);\
}

const uint8_t g_kuiLeadingZeroTable[256]={
	8,7,6,6,5,5,5,5,4,4,4,4,4,4,4,4,
	3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static inline int32_t GetLeadingZeroBits(uint32_t iCurBits){		// <=32 bits
	uint32_t uiValue;
	uiValue=UBITS(iCurBits,8);		// ShowBits( bs,8 );
	if(uiValue){
		return g_kuiLeadingZeroTable[uiValue];
	}
	uiValue=UBITS(iCurBits,16);		// ShowBits( bs,16 );
	if(uiValue){
		return (g_kuiLeadingZeroTable[uiValue]+8);
	}
	uiValue=UBITS(iCurBits,24);		// ShowBits( bs,24 );
	if(uiValue){
		return (g_kuiLeadingZeroTable[uiValue]+16);
	}
	uiValue=iCurBits;				// ShowBits( bs,32 );
	if(uiValue){
		return (g_kuiLeadingZeroTable[uiValue]+24);
	}
	FATAL("TF");					// should not go here
	return -1;
}

// Check whether there is more rbsp data for processing
static inline bool CheckMoreRBSPData(SBitStringAux* pBsAux){
	if((pBsAux->iBits-((pBsAux->pCurBuf-pBsAux->pStartBuf-2)<<3)-pBsAux->iLeftBits)>1){
		return true;
	}else{
		return false;
	}
}

static inline int32_t BsGetBits(SBitStringAux* pBs,int32_t iNumBits,uint32_t* pCode){
	int32_t iRc=UBITS(pBs->uiCurBits,iNumBits);
	int32_t iAllowedBytes=(int32_t)(pBs->pEndBuf-pBs->pStartBuf);
	int32_t iReadBytes=(int32_t)(pBs->pCurBuf-pBs->pStartBuf);
	DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iNumBits,iAllowedBytes,iReadBytes);
	*pCode=(uint32_t)iRc;
	return ERR_NONE;
}

static inline uint32_t BsGetOneBit(SBitStringAux* pBs,uint32_t* pCode){
	return BsGetBits(pBs,1,pCode);
}

inline uint32_t GetValue4Bytes(uint8_t* pDstNal){
	uint32_t uiValue=0;
	uiValue=(pDstNal[0]<<24)|(pDstNal[1]<<16)|(pDstNal[2]<<8)|(pDstNal[3]);
	return uiValue;
}

int32_t InitReadBits(SBitStringAux* pBitString,int32_t iEndOffset){
	if(pBitString->pCurBuf>=(pBitString->pEndBuf-iEndOffset)){
		return ERR_INFO_INVALID_ACCESS;
	}
	pBitString->uiCurBits=GetValue4Bytes(pBitString->pCurBuf);
	pBitString->pCurBuf+=4;
	pBitString->iLeftBits=-16;
	return ERR_NONE;
}

// brief input bits for decoder or initialize bitstream writing in encoder
// param pBitString Bit string auxiliary pointer
// param kpBuf bit-stream buffer
// param kiSize size in bits for decoder
void DecInitBits(SBitStringAux* pBitString,const uint8_t* kpBuf,const int32_t kiSize){
	const int32_t kiSizeBuf=(kiSize+7)>>3;
	uint8_t* pTmp=(uint8_t*)kpBuf;
	pBitString->pStartBuf=pTmp;				// buffer to start position
	pBitString->pEndBuf=pTmp+kiSizeBuf;		// buffer+length
	pBitString->iBits=kiSize;				// count bits of overall bitstreaming inputindex;
	pBitString->pCurBuf=pBitString->pStartBuf;
	//int32_t iErr=
	InitReadBits(pBitString,0);
}

static inline uint32_t BsGetUe(SBitStringAux* pBs,uint32_t* pCode){
	uint32_t iValue=0;
	int32_t iLeadingZeroBits=GetLeadingZeroBits(pBs->uiCurBits);
	int32_t iAllowedBytes,iReadBytes;
	iAllowedBytes=(int32_t)(pBs->pEndBuf-pBs->pStartBuf);		// actual stream bytes
	if(iLeadingZeroBits==-1){						// bistream error
		return ERR_INFO_READ_LEADING_ZERO;			// -1
	}else
	if(iLeadingZeroBits>16){						// rarely into this condition (even may be bitstream error),prevent from 16-bit reading overflow
// using two-step reading instead of one time reading of >16 bits.
		iReadBytes=(int32_t)(pBs->pCurBuf-pBs->pStartBuf);
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,16,iAllowedBytes,iReadBytes);
		iReadBytes=(int32_t)(pBs->pCurBuf-pBs->pStartBuf);
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iLeadingZeroBits+1-16,iAllowedBytes,iReadBytes);
	}else{
		iReadBytes=(int32_t)(pBs->pCurBuf-pBs->pStartBuf);
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iLeadingZeroBits+1,iAllowedBytes,iReadBytes);
	}
	if(iLeadingZeroBits){
		iValue=UBITS(pBs->uiCurBits,iLeadingZeroBits);
		iReadBytes=(int32_t)(pBs->pCurBuf-pBs->pStartBuf);
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iLeadingZeroBits,iAllowedBytes,iReadBytes);
	}
	*pCode=((1u<<iLeadingZeroBits)-1+iValue);
	return ERR_NONE;
}

// Read signed exp golomb codes
static inline int32_t BsGetSe(SBitStringAux* pBs,int32_t* pCode){
	uint32_t uiCodeNum;
	READ_VERIFY(BsGetUe(pBs,&uiCodeNum));
	if(uiCodeNum&0x01){
		*pCode=(int32_t)((uiCodeNum+1)>>1);
	}else{
		*pCode=NEG_NUM((int32_t)(uiCodeNum>>1));
	}
	return ERR_NONE;
}

// Get number of trailing bits
static inline int32_t BsGetTrailingBits(const uint8_t* pBuf){
	uint32_t uiValue=*pBuf;
	int32_t iRetNum=0;
	do{
		if(uiValue&1)
			return iRetNum;
		uiValue>>=1;
		++iRetNum;
	} while(iRetNum<9);
	return 0;
}

struct sMCRefMember{
	uint8_t* pDstY;
	uint8_t* pDstU;
	uint8_t* pDstV;
	uint8_t* pSrcY;
	uint8_t* pSrcU;
	uint8_t* pSrcV;
	int32_t iSrcLineLuma;
	int32_t iSrcLineChroma;
	int32_t iDstLineLuma;
	int32_t iDstLineChroma;
};

class PlainH264Decoder: public ISVCDecoderBase{
	public:
		PlainH264Decoder(void);
		virtual ~PlainH264Decoder();
		virtual bool Initialize(const SDecodingParam* pParam);
		virtual bool DecodeFrame(const unsigned char* kpSrc,const int kiSrcLen,unsigned char** ppDst);
		virtual void GetProfileDisplay(std::vector<DisplayTimer>* profilerDisplay) {
			*profilerDisplay=m_profilerDisplay;
		}
		virtual uint32_t Width()const{return m_videoWidth;}
		virtual uint32_t Height()const{return m_videoHeight;}
		virtual uint32_t StrideY()const{return m_stride[0];}
		virtual uint32_t StrideUV()const{return m_stride[1];}
	private:
		int32_t DecodeSliceNal(SDecoderContext* pCtx,const uint8_t* payload,int32_t payloadBitsize,uint8_t** ppDst);
		int32_t DecodeMbCabacPSliceBaseMode0(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SBitStringAux* pBsAux,const SNeighAvail* pNeighAvail);
		void DecodeMbCabacPSlice(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SBitStringAux* pBsAux);
		int32_t ParseInterPMotionInfoCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,int16_t pMotionVector[LIST_A][30][MV_A],int16_t pMvdCache[LIST_A][30][MV_A],int8_t pRefIndex[LIST_A][30]);
		const uint8_t* DecodeBsInit(int32_t* piNalBitSize,SDecoderContext* pCtx,const uint8_t* kpBsBuf,const int32_t kiBsLen);
		uint8_t* DetectStartCodePrefix(const uint8_t* kpBuf,int32_t* pOffset,int32_t iBufSize);
		int32_t ParsePps(SPps* pPps,SBitStringAux* pBsAux);
		int32_t ParseSps(SSps* pSps,SBitStringAux* pBsAux);
		const uint8_t* ParseNalHeader(int32_t* iNalSize,SNalUnit* pNalUnitHeader,const uint8_t* pSrcRbsp,int32_t iSrcRbspLen,const uint8_t* pSrcNal,int32_t iSrcNalLen,int32_t* pConsumedBytes);
		int32_t ParseSliceHeaderSyntaxs(SDecoderContext* pCtx,SBitStringAux* pBs);
		int32_t ParseRefPicListReordering(SDecoderContext* pCtx,SBitStringAux* pBs,SSlice* pSh);
		void InitPredFunc(SDecoderContext* pCtx);
		void TargetSliceConstruction(SDecoderContext* pCtx);
		int32_t MbInterConstruction(SDecoderContext* pCtx,SDqLayer* layer);
		int32_t MbInterSampleConstruction(SDecoderContext* pCtx,SDqLayer* layer,uint8_t* pDstY,uint8_t* pDstU,uint8_t* pDstV,int32_t iStrideL,int32_t iStrideC);
		int32_t MbInterPrediction(SDecoderContext* pCtx,SDqLayer* layer);
		int32_t GetInterPred(uint8_t* pPredY,uint8_t* pPredCb,uint8_t* pPredCr,SDecoderContext* pCtx);
		void BaseMC(SDecoderContext* pCtx,sMCRefMember* pMCRefMem,const int32_t& listIdx,const int8_t& iRefIdx,int32_t iXOffset,int32_t iYOffset,int32_t iBlkWidth,int32_t iBlkHeight,int16_t iMVs[2]);
		int32_t MbIntraPredictionConstruction(SDecoderContext* pCtx,SDqLayer* layer);
		int32_t RecI4x4Chroma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer);
		int32_t RecI8x8Mb(int32_t iMbXy,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer);
		int32_t RecI4x4Luma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer);
		int32_t RecI4x4Mb(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer);
		void RecI8x8Luma(int32_t iMbXy,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer);
		void RecI16x16Mb(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer);
		void RecChroma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer);
		void FillRecNeededMbInfo(SDecoderContext* pCtx,SDqLayer* layer);
		void DeblockingMb(SDecoderContext* pCtx,SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag);
		void DeblockingInterMb(SDqLayer* layer,SDeblockingFilter* pFilter,uint8_t nBS[2][4][4],int32_t iBoundryFlag);
		void FilteringEdgeChromaH(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS);
		void FilteringEdgeLumaH(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS);
		void FilteringEdgeLumaHV(SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag);
		void FilteringEdgeChromaHV(SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag);
		void FilteringEdgeChromaIntraH(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS);
		void FilteringEdgeChromaIntraV(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS);
		void DeblockingIntraMb(SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag);
		uint32_t DeblockingBsMarginalMBAvcbase(SDeblockingFilter* pFilter,SDqLayer* layer,int32_t iEdge,int32_t iNeighMb,int32_t iMbXy);
		void FilteringEdgeChromaV(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS);
		void DeblockingBSInsideMBAvsbase(int8_t* pNnzTab,uint8_t nBS[2][4][4],int32_t iLShiftFactor);
		void DeblockingBSInsideMBAvsbase8x8(int8_t* pNnzTab,uint8_t nBS[2][4][4],int32_t iLShiftFactor);
		void DeblockingBSliceBSInsideMBNormal(SDeblockingFilter* pFilter,SDqLayer* layer,uint8_t nBS[2][4][4],int8_t* pNnzTab,int32_t iMbXy);
		void FilteringEdgeLumaV(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS);
		void FilteringEdgeLumaIntraV(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS);
		void FilteringEdgeLumaIntraH(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS);
		static void MapNxNNeighToSampleNormal(const SNeighAvail* pNeighAvail,int32_t* pSampleAvail);
		static void FillCacheConstrain0IntraNxN(const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,int8_t* pIntraPredMode,SDqLayer* layer);		// no matter slice type
		int32_t DecodeMbCabacISlice(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SBitStringAux* pBsAux);
		int32_t ParseMBTypeISliceCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SNeighAvail* pNeighAvail,uint32_t& uiBinVal);
		int32_t ParseResidualBlockCabac8x8(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCountCache,SBitStringAux* pBsAux,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* pScanTable,int32_t iResProperty,short* sTCoeff,uint8_t uiQp,SDecoderContext* pCtx);
		int32_t ParseResidualBlockCabac(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCountCache,SBitStringAux* pBsAux,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* pScanTable,int32_t iResProperty,short* sTCoeff,uint8_t uiQp,SDecoderContext* pCtx);
		void LumaDcDequantIdct(int16_t* pBlock,int32_t iQp,SDecoderContext* pCtx);
		int32_t ParseSignificantCoeffCabac(SCabacDecEngine* cabacDecEngine,int32_t* pSignificant,int32_t iResProperty,SDecoderContext* pCtx);
		int32_t ParseSignificantMapCabac(SCabacDecEngine* cabacDecEngine,int32_t* pSignificantMap,int32_t iResProperty,SDecoderContext* pCtx,uint32_t& uiCoeffNum);
		int32_t ParseCbfInfoCabac(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail,uint8_t* pNzcCache,int32_t iZIndex,int32_t iResProperty,SDecoderContext* pCtx,uint32_t& uiCbfBit);
		int32_t ParseDeltaQpCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,int32_t& iQpDelta);
		int32_t ParseCbpInfoCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint32_t& uiCbp);
		int32_t ParseIntra16x16Mode(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,SBitStringAux* pBs,SDqLayer* layer);
		int32_t CheckIntra16x16PredMode(uint8_t uiSampleAvail,int8_t* pMode);
		int32_t ParseIntra4x4Mode(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,int8_t* pIntraPredMode,SBitStringAux* pBs,SDqLayer* layer);
		int32_t ParseIntra8x8Mode(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,int8_t* pIntraPredMode,SBitStringAux* pBs,SDqLayer* layer);
		int32_t CheckIntraChromaPredMode(uint8_t uiSampleAvail,int8_t* pMode);
		int32_t ParseIntraPredModeChromaCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,uint8_t uiNeighAvail,int32_t& iBinVal);
		int32_t CheckIntraNxNPredMode(int32_t* pSampleAvail,int8_t* pMode,int32_t iIndex,bool b8x8);
		int32_t ParseIntraPredModeLumaCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,int32_t& iBinVal);
		int32_t ParseTransformSize8x8FlagCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,bool& bTransformSize8x8Flag);
		int32_t ParseSubMBTypeCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint32_t& uiSubMbType);
		void UpdateP8x16MvdCabac(SDqLayer* layer,int16_t pMvdCache[LIST_A][30][MV_A],int32_t iPartIdx,int16_t pMvd[2],const int8_t iListIdx);
		void UpdateP8x16MotionInfo(SDqLayer* layer,int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVs[2]);
		void PredInter8x16Mv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVP[2]);
		void UpdateP8x16RefIdxCabac(SDqLayer* layer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx);
		void UpdateP16x8MvdCabac(SDqLayer* layer,int16_t pMvdCache[LIST_A][30][MV_A],int32_t iPartIdx,int16_t pMvd[2],const int8_t iListIdx);
		void UpdateP16x8MotionInfo(SDqLayer* layer,int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVs[2]);
		void PredInter16x8Mv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVP[2]);
		void UpdateP16x8RefIdxCabac(SDqLayer* layer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx);
		void UpdateP16x16MvdCabac(SDqLayer* layer,int16_t pMvd[2],const int8_t iListIdx);
		void UpdateP16x16MotionInfo(SDqLayer* layer,int32_t listIdx,int8_t iRef,int16_t iMVs[2]);
		int32_t ParseMvdInfoCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,int8_t pRefIndex[LIST_A][30],int16_t pMvdCache[LIST_A][30][2],int32_t index,int8_t iListIdx,int8_t iMvComp,int16_t& iMvdVal);
		void PredMv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int32_t iPartWidth,int8_t iRef,int16_t iMVP[2]);
		void FillCacheInterCabac(SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,int16_t iMvArray[LIST_A][30][MV_A],int16_t iMvdCache[LIST_A][30][MV_A],int8_t iRefIdxArray[LIST_A][30],SDqLayer* layer);
		static void FillCacheNonZeroCount(const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,SDqLayer* layer);
		uint32_t ParseMBTypePSliceCabac(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail);
		void PredPSkipMvFromNeighbor(SDqLayer* layer,int16_t iMvp[2]);
		int32_t ParseSkipFlagCabac(SCabacDecEngine* cabacDecEngine,SNeighAvail* pNeighAvail,uint32_t& uiSkip);
		void GetNeighborAvailMbType(SNeighAvail* pNeighAvail,SDqLayer* layer);
		void ExpandReferencingPicture(uint8_t* pData[3],int32_t iWidth,int32_t iHeight,int32_t iStride[3]);
		void InitDqLayerInfo(SDecoderContext* pCtx,SDqLayer* pDqLayer,SNalUnit* pNalUnit,SPicture* pPicDec);
		void InitCurDqLayerData(SDecoderContext* pCtx,SDqLayer* pCurDq);
		SPicture* AllocPicture(SDecoderContext* pCtx,const int32_t kiPicWidth,const int32_t kiPicHeight);
		void FreePicture(SPicture* pPic);
		void FreeStaticMemory(SDecoderContext* pCtx);
		void FreeDynamicMemory(SDecoderContext* pCtx);
		void CloseDecoder(SDecoderContext* pCtx);

		int m_iFrameNumber=-1;
		int m_frameNumber=0;
		int m_intraFrameIndex=0;											// Count since last I frame
		int m_videoWidth;
		int m_videoHeight;
		int m_blockWidth;													// m_videoWidth/16
		int m_blockHeight;													// m_videoHeight/16
		int m_stride[2];
		std::vector<DisplayTimer> m_profilerDisplay;
		SDecoderContext* m_pCtx;
		Profiler m_profiler;

		void CreateMalloc();
		void DestroyMalloc();
		void* Malloc(uint32_t byteSize);
		void* Mallocz(uint32_t byteSize);
		void Free(void* p);
		void* m_memoryBlock=0;
		uint32_t m_memoryBlockByteSize;
		uint32_t m_memoryBlockPos;
};

#if defined(__linux__) || defined(__APPLE__)

void* AllocSystemAligned(int lAlignment,int lSize) {
	void* pMemory=aligned_alloc(lAlignment,lSize);
	return pMemory;
}
void FreeSystemAligned(void* pMemory) {
	free(pMemory);
}
void* AllocSystemAligned(int lSize) {
	return AllocSystemAligned(64,lSize);
}
#else

void* AllocSystemAligned(int lAlignment,int lSize) {
	void* pMemory=_aligned_malloc(lSize,lAlignment);
	return pMemory;
}
void FreeSystemAligned(void* pMemory) {
	_aligned_free(pMemory);
}
void* AllocSystemAligned(int lSize) {
	return AllocSystemAligned(64,lSize);
}

#endif


void PlainH264Decoder::CreateMalloc() {
	m_memoryBlockByteSize=1024*1024*20;
	m_memoryBlock=AllocSystemAligned(0x10,m_memoryBlockByteSize);
	m_memoryBlockPos=0;
}
void PlainH264Decoder::DestroyMalloc() {
	FreeSystemAligned(m_memoryBlock);
}

void* PlainH264Decoder::Malloc(uint32_t byteSize) {
	uint32_t byteSizeRoundup=(byteSize+15)&-16;
	if(m_memoryBlockPos+byteSizeRoundup>m_memoryBlockByteSize)
		FATAL("Out of memory");
	void* p=(uint8_t*)m_memoryBlock+m_memoryBlockPos;
	m_memoryBlockPos+=byteSizeRoundup;
	return p;
	//return _aligned_malloc(byteSize,0x10);
}
void* PlainH264Decoder::Mallocz(uint32_t byteSize) {
	void* p=Malloc(byteSize);
	memset(p,0,byteSize);
	return p;
}
void PlainH264Decoder::Free(void* p){
	//_aligned_free(p);
}

long CreateDecoder(ISVCDecoderBase** ppDecoder){
	*ppDecoder=new PlainH264Decoder();
	return ERR_NONE;
}

#define PADDING_LENGTH 32		// reference extension

void PlainH264Decoder::FreePicture(SPicture* pPic){
	if(NULL!=pPic){
		if(pPic->pBuffer[0]){
			Free(pPic->pBuffer[0]);
			pPic->pBuffer[0]=NULL;
		}
		if(pPic->pMbType){
			Free(pPic->pMbType);
			pPic->pMbType=NULL;
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(pPic->pMv[listIdx]){
				Free(pPic->pMv[listIdx]);
				pPic->pMv[listIdx]=NULL;
			}
			if(pPic->pRefIndex[listIdx]){
				Free(pPic->pRefIndex[listIdx]);
				pPic->pRefIndex[listIdx]=NULL;
			}
		}
		Free(pPic);
		pPic=NULL;
	}
}

SPicture* PlainH264Decoder::AllocPicture(SDecoderContext* pCtx,const int32_t kiPicWidth,const int32_t kiPicHeight){
	SPicture* pPic=(SPicture*)Mallocz(sizeof(SPicture));
	memset(pPic,0,sizeof(SPicture));
	int32_t iPicWidth=WELS_ALIGN(kiPicWidth+(PADDING_LENGTH<<1),PICTURE_RESOLUTION_ALIGNMENT);
	int32_t iPicHeight=WELS_ALIGN(kiPicHeight+(PADDING_LENGTH<<1),PICTURE_RESOLUTION_ALIGNMENT);
	int32_t iPicChromaWidth=iPicWidth>>1;
	int32_t iPicChromaHeight=iPicHeight>>1;
	int32_t iLumaSize=iPicWidth*iPicHeight;
	int32_t iChromaSize=iPicChromaWidth*iPicChromaHeight;
	pPic->pBuffer[0]=(uint8_t*)Mallocz(iLumaSize+(iChromaSize<<1));
	memset(pPic->pBuffer[0],128,(iLumaSize+(iChromaSize<<1)));
	pPic->iLinesize[0]=iPicWidth;
	pPic->iLinesize[1]=pPic->iLinesize[2]=iPicChromaWidth;
	pPic->pBuffer[1]=pPic->pBuffer[0]+iLumaSize;
	pPic->pBuffer[2]=pPic->pBuffer[1]+iChromaSize;
	pPic->pData[0]=pPic->pBuffer[0]+(1+pPic->iLinesize[0])*PADDING_LENGTH;
	pPic->pData[1]=pPic->pBuffer[1]+(((1+pPic->iLinesize[1])*PADDING_LENGTH)>>1);
	pPic->pData[2]=pPic->pBuffer[2]+(((1+pPic->iLinesize[2])*PADDING_LENGTH)>>1);
	uint32_t uiMbWidth=(kiPicWidth+15)>>4;
	uint32_t uiMbHeight=(kiPicHeight+15)>>4;
	uint32_t uiMbCount=uiMbWidth*uiMbHeight;
	pPic->pMbType=(uint32_t*)Mallocz(uiMbCount*sizeof(uint32_t));
	pPic->pMv[LIST_0]=(int16_t(*)[16][2])Mallocz(uiMbCount*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pPic->pMv[LIST_1]=(int16_t(*)[16][2])Mallocz(uiMbCount*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pPic->pRefIndex[LIST_0]=(int8_t(*)[16])Mallocz(uiMbCount*sizeof(int8_t)*MB_BLOCK4x4_NUM);
	pPic->pRefIndex[LIST_1]=(int8_t(*)[16])Mallocz(uiMbCount*sizeof(int8_t)*MB_BLOCK4x4_NUM);
	return pPic;
}

void PlainH264Decoder::InitCurDqLayerData(SDecoderContext* pCtx,SDqLayer* pCurDq){
	pCurDq->pMv[LIST_0]=(int16_t(*)[16][2])Mallocz(m_blockWidth*m_blockHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pCurDq->pMv[LIST_1]=(int16_t(*)[16][2])Mallocz(m_blockWidth*m_blockHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pCurDq->pRefIndex[LIST_0]=(int8_t(*)[MB_BLOCK4x4_NUM])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
	pCurDq->pRefIndex[LIST_1]=(int8_t(*)[MB_BLOCK4x4_NUM])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
	pCurDq->pDirect=(int8_t(*)[MB_BLOCK4x4_NUM])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
	pCurDq->pLumaQp=(int8_t*)Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t));
	pCurDq->pNoSubMbPartSizeLessThan8x8Flag=(bool*)Mallocz(m_blockWidth*m_blockHeight*sizeof(bool));
	pCurDq->pTransformSize8x8Flag=(bool*)Mallocz(m_blockWidth*m_blockHeight*sizeof(bool));
	pCurDq->pChromaQp=(int8_t(*)[2])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*2);
	pCurDq->pMvd[LIST_0]=(int16_t(*)[16][2])Mallocz(m_blockWidth*m_blockHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pCurDq->pMvd[LIST_1]=(int16_t(*)[16][2])Mallocz(m_blockWidth*m_blockHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pCurDq->pCbfDc=(uint16_t*)Mallocz(m_blockWidth*m_blockHeight*sizeof(uint16_t));
	pCurDq->pNzc=(int8_t(*)[24])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*24);
	pCurDq->pNzcRs=(int8_t(*)[24])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*24);
	pCurDq->pScaledTCoeff=(int16_t(*)[MB_COEFF_LIST_SIZE])Mallocz(m_blockWidth*m_blockHeight*sizeof(int16_t)*MB_COEFF_LIST_SIZE);
	pCurDq->pIntraPredMode=(int8_t(*)[8])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*8);
	pCurDq->pIntra4x4FinalMode=(int8_t(*)[MB_BLOCK4x4_NUM])Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
	pCurDq->pIntraNxNAvailFlag=(uint8_t(*))Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t));
	pCurDq->pChromaPredMode=(int8_t*)Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t));
	pCurDq->pCbp=(int8_t*)Mallocz(m_blockWidth*m_blockHeight*sizeof(int8_t));
	pCurDq->pSubMbType=(uint32_t(*)[MB_PARTITION_SIZE])Mallocz(m_blockWidth*m_blockHeight*sizeof(uint32_t)*MB_PARTITION_SIZE);
}

const uint8_t g_kuiScan8[24]={	// [16+2*4]
	9,10,17,18,					// 1+1*8,2+1*8,1+2*8,2+2*8,
	11,12,19,20,				// 3+1*8,4+1*8,3+2*8,4+2*8,
	25,26,33,34,				// 1+3*8,2+3*8,1+4*8,2+4*8,
	27,28,35,36,				// 3+3*8,4+3*8,3+4*8,4+4*8,
	14,15,						// 6+1*8,7+1*8,
	22,23,						// 6+2*8,7+2*8,
	38,39,						// 6+4*8,7+4*8,
	46,47,						// 6+5*8,7+5*8,
};


void GetI4LumaIChromaAddrTable(int32_t* pBlockOffset,const int32_t kiYStride,const int32_t kiUVStride){
	int32_t* pOffset=pBlockOffset;
	int32_t i;
	const uint8_t kuiScan0=g_kuiScan8[0];
	for(i=0; i<16; i++){
		const uint32_t kuiA=g_kuiScan8[i]-kuiScan0;
		const uint32_t kuiX=kuiA&0x07;
		const uint32_t kuiY=kuiA>>3;
		pOffset[i]=(kuiX+kiYStride*kuiY)<<2;
	}
	for(i=0; i<4; i++){
		const uint32_t kuiA=g_kuiScan8[i]-kuiScan0;
		pOffset[16+i]=pOffset[20+i]=((kuiA&0x07)+(kiUVStride)*(kuiA>>3))<<2;
	}
}

void PlainH264Decoder::InitDqLayerInfo(SDecoderContext* pCtx,SDqLayer* pDqLayer,SNalUnit* pNalUnit,SPicture* pPicDec){
	//SSlice* pSh=&pCtx->m_slice;
	pDqLayer->pDec=pPicDec;
	pDqLayer->iMbWidth=m_blockWidth;
	pDqLayer->iMbHeight=m_blockHeight;
}

static inline void ExpandPictureChroma_c(uint8_t* pDst,const int32_t kiStride,const int32_t kiPicW,const int32_t kiPicH){
	uint8_t* pTmp=pDst;
	uint8_t* pDstLastLine=pTmp+(kiPicH-1)*kiStride;
	const int32_t kiPaddingLen=(PADDING_LENGTH>>1);
	const uint8_t kuiTL=pTmp[0];
	const uint8_t kuiTR=pTmp[kiPicW-1];
	const uint8_t kuiBL=pDstLastLine[0];
	const uint8_t kuiBR=pDstLastLine[kiPicW-1];
	int32_t i=0;
	do{
		const int32_t kiStrides=(1+i)*kiStride;
		uint8_t* pTop=pTmp-kiStrides;
		uint8_t* pBottom=pDstLastLine+kiStrides;
// pad pTop and pBottom
		memcpy(pTop,pTmp,kiPicW);
		memcpy(pBottom,pDstLastLine,kiPicW);
// pad corners
		memset(pTop-kiPaddingLen,kuiTL,kiPaddingLen);		// pTop left
		memset(pTop+kiPicW,kuiTR,kiPaddingLen);				// pTop right
		memset(pBottom-kiPaddingLen,kuiBL,kiPaddingLen);	// pBottom left
		memset(pBottom+kiPicW,kuiBR,kiPaddingLen);			// pBottom right
		++i;
	} while(i<kiPaddingLen);
// pad left and right
	i=0;
	do{
		memset(pTmp-kiPaddingLen,pTmp[0],kiPaddingLen);
		memset(pTmp+kiPicW,pTmp[kiPicW-1],kiPaddingLen);
		pTmp+=kiStride;
		++i;
	} while(i<kiPicH);
}

// rewrite it (split into luma & chroma) that is helpful for mmx/sse2 optimization perform,9/27/2009
static inline void ExpandPictureLuma_c(uint8_t* pDst,const int32_t kiStride,const int32_t kiPicW,const int32_t kiPicH){
	uint8_t* pTmp=pDst;
	uint8_t* pDstLastLine=pTmp+(kiPicH-1)*kiStride;
	const int32_t kiPaddingLen=PADDING_LENGTH;
	const uint8_t kuiTL=pTmp[0];
	const uint8_t kuiTR=pTmp[kiPicW-1];
	const uint8_t kuiBL=pDstLastLine[0];
	const uint8_t kuiBR=pDstLastLine[kiPicW-1];
	int32_t i=0;
	do{
		const int32_t kiStrides=(1+i)*kiStride;
		uint8_t* pTop=pTmp-kiStrides;
		uint8_t* pBottom=pDstLastLine+kiStrides;
// pad pTop and pBottom
		memcpy(pTop,pTmp,kiPicW);
		memcpy(pBottom,pDstLastLine,kiPicW);
// pad corners
		memset(pTop-kiPaddingLen,kuiTL,kiPaddingLen);			// pTop left
		memset(pTop+kiPicW,kuiTR,kiPaddingLen);					// pTop right
		memset(pBottom-kiPaddingLen,kuiBL,kiPaddingLen);		// pBottom left
		memset(pBottom+kiPicW,kuiBR,kiPaddingLen);				// pBottom right

		++i;
	} while(i<kiPaddingLen);
// pad left and right
	i=0;
	do{
		memset(pTmp-kiPaddingLen,pTmp[0],kiPaddingLen);
		memset(pTmp+kiPicW,pTmp[kiPicW-1],kiPaddingLen);
		pTmp+=kiStride;
		++i;
	} while(i<kiPicH);
}

void PlainH264Decoder::ExpandReferencingPicture(uint8_t* pData[3],int32_t iWidth,int32_t iHeight,int32_t iStride[3]){
	ExpandPictureLuma_c(pData[0],iStride[0],iWidth,iHeight);
	ExpandPictureChroma_c(pData[1],iStride[1],iWidth>>1,iHeight>>1);
	ExpandPictureChroma_c(pData[2],iStride[2],iWidth>>1,iHeight>>1);
}

void PlainH264Decoder::GetNeighborAvailMbType(SNeighAvail* pNeighAvail,SDqLayer* layer){
	int32_t iCurSliceIdc,iTopSliceIdc,iLeftTopSliceIdc,iRightTopSliceIdc,iLeftSliceIdc;
	int32_t iCurXy,iTopXy=0,iLeftXy=0,iLeftTopXy=0,iRightTopXy=0;
	int32_t iCurX,iCurY;
	iCurXy=layer->iMbXyIndex;
	iCurX=layer->iMbX;
	iCurY=layer->iMbY;
	iCurSliceIdc=0;//layer->pSliceIdc[iCurXy];
	if(iCurX!=0){
		iLeftXy=iCurXy-1;
		iLeftSliceIdc=0;//layer->pSliceIdc[iLeftXy];
		pNeighAvail->iLeftAvail=(iLeftSliceIdc==iCurSliceIdc);
		pNeighAvail->iLeftCbp=pNeighAvail->iLeftAvail ? layer->pCbp[iLeftXy] : 0;
	}else{
		pNeighAvail->iLeftAvail=0;
		pNeighAvail->iLeftTopAvail=0;
		pNeighAvail->iLeftCbp=0;
	}
	if(iCurY!=0){
		iTopXy=iCurXy-layer->iMbWidth;
		iTopSliceIdc=0;//layer->pSliceIdc[iTopXy];
		pNeighAvail->iTopAvail=(iTopSliceIdc==iCurSliceIdc);
		pNeighAvail->iTopCbp=pNeighAvail->iTopAvail ? layer->pCbp[iTopXy] : 0;
		if(iCurX!=0){
			iLeftTopXy=iTopXy-1;
			iLeftTopSliceIdc=0;//layer->pSliceIdc[iLeftTopXy];
			pNeighAvail->iLeftTopAvail=(iLeftTopSliceIdc==iCurSliceIdc);
		}else{
			pNeighAvail->iLeftTopAvail=0;
		}
		if(iCurX!=(layer->iMbWidth-1)){
			iRightTopXy=iTopXy+1;
			iRightTopSliceIdc=0;//layer->pSliceIdc[iRightTopXy];
			pNeighAvail->iRightTopAvail=(iRightTopSliceIdc==iCurSliceIdc);
		}else{
			pNeighAvail->iRightTopAvail=0;
		}
	}else{
		pNeighAvail->iTopAvail=0;
		pNeighAvail->iLeftTopAvail=0;
		pNeighAvail->iRightTopAvail=0;
		pNeighAvail->iTopCbp=0;
	}
	pNeighAvail->iLeftType=(pNeighAvail->iLeftAvail ? layer->pDec->pMbType[iLeftXy] : 0);
	pNeighAvail->iTopType=(pNeighAvail->iTopAvail ? layer->pDec->pMbType[iTopXy] : 0);
	pNeighAvail->iLeftTopType=(pNeighAvail->iLeftTopAvail ? layer->pDec->pMbType[iLeftTopXy] : 0);
	pNeighAvail->iRightTopType=(pNeighAvail->iRightTopAvail ? layer->pDec->pMbType[iRightTopXy] : 0);
}


int32_t PlainH264Decoder::ParseSkipFlagCabac(SCabacDecEngine* cabacDecEngine,SNeighAvail* pNeighAvail,uint32_t& uiSkip){
	uiSkip=0;
	int32_t iCtxInc=NEW_CTX_OFFSET_SKIP;
	iCtxInc+=(pNeighAvail->iLeftAvail && !IS_SKIP(pNeighAvail->iLeftType))+(pNeighAvail->iTopAvail && !IS_SKIP(pNeighAvail->iTopType));
	//SCabacCtx* pBinCtx=pCabacCtx+iCtxInc;
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(iCtxInc,uiSkip));
	return ERR_NONE;
}

inline uint32_t* GetMbType(SDqLayer*& layer){
	if(layer->pDec!=NULL){
		return layer->pDec->pMbType;
	}else{
		return layer->pMbType;
	}
}

void PlainH264Decoder::PredPSkipMvFromNeighbor(SDqLayer* layer,int16_t iMvp[2]){
	bool bTopAvail,bLeftTopAvail,bRightTopAvail,bLeftAvail;
	int32_t iCurSliceIdc,iTopSliceIdc,iLeftTopSliceIdc,iRightTopSliceIdc,iLeftSliceIdc;
	int32_t iLeftTopType,iRightTopType,iTopType,iLeftType;
	int32_t iCurX,iCurY,iCurXy,iLeftXy,iTopXy=0,iLeftTopXy=0,iRightTopXy=0;
	int8_t iLeftRef;
	int8_t iTopRef;
	int8_t iRightTopRef;
	int8_t iLeftTopRef;
	int8_t iDiagonalRef;
	int8_t iMatchRef;
	int16_t iMvA[2],iMvB[2],iMvC[2],iMvD[2];
	iCurXy=layer->iMbXyIndex;
	iCurX=layer->iMbX;
	iCurY=layer->iMbY;
	iCurSliceIdc=0;//layer->pSliceIdc[iCurXy];
	if(iCurX!=0){
		iLeftXy=iCurXy-1;
		iLeftSliceIdc=0;//layer->pSliceIdc[iLeftXy];
		bLeftAvail=(iLeftSliceIdc==iCurSliceIdc);
	}else{
		bLeftAvail=0;
		bLeftTopAvail=0;
	}
	if(iCurY!=0){
		iTopXy=iCurXy-layer->iMbWidth;
		iTopSliceIdc=0;//layer->pSliceIdc[iTopXy];
		bTopAvail=(iTopSliceIdc==iCurSliceIdc);
		if(iCurX!=0){
			iLeftTopXy=iTopXy-1;
			iLeftTopSliceIdc=0;//layer->pSliceIdc[iLeftTopXy];
			bLeftTopAvail=(iLeftTopSliceIdc==iCurSliceIdc);
		}else{
			bLeftTopAvail=0;
		}
		if(iCurX!=(layer->iMbWidth-1)){
			iRightTopXy=iTopXy+1;
			iRightTopSliceIdc=0;//layer->pSliceIdc[iRightTopXy];
			bRightTopAvail=(iRightTopSliceIdc==iCurSliceIdc);
		}else{
			bRightTopAvail=0;
		}
	}else{
		bTopAvail=0;
		bLeftTopAvail=0;
		bRightTopAvail=0;
	}
	iLeftType=((iCurX!=0 && bLeftAvail) ? GetMbType(layer)[iLeftXy] : 0);
	iTopType=((iCurY!=0 && bTopAvail) ? GetMbType(layer)[iTopXy] : 0);
	iLeftTopType=((iCurX!=0 && iCurY!=0 && bLeftTopAvail) ? GetMbType(layer)[iLeftTopXy] : 0);
	iRightTopType=((iCurX!=layer->iMbWidth-1 && iCurY!=0 && bRightTopAvail) ? GetMbType(layer)[iRightTopXy] : 0);
	// get neb mv&iRefIdxArray
	// left
	if(bLeftAvail && IS_INTER(iLeftType)){
		ST32(iMvA,LD32(layer->pDec ? layer->pDec->pMv[0][iLeftXy][3] : layer->pMv[0][iLeftXy][3]));
		iLeftRef=layer->pDec ? layer->pDec->pRefIndex[0][iLeftXy][3] : layer->pRefIndex[0][iLeftXy][3];
	}else{
		ST32(iMvA,0);
		if(0==bLeftAvail){						// not available
			iLeftRef=REF_NOT_AVAIL;
		}else{									// available but is intra mb type
			iLeftRef=REF_NOT_IN_LIST;
		}
	}
	if(REF_NOT_AVAIL==iLeftRef || 
		(0==iLeftRef && 0==*(int32_t*)iMvA)){
		ST32(iMvp,0);
		return;
	}
	// top
	if(bTopAvail && IS_INTER(iTopType)){
		ST32(iMvB,LD32(layer->pDec ? layer->pDec->pMv[0][iTopXy][12] : layer->pMv[0][iTopXy][12]));
		iTopRef=layer->pDec ? layer->pDec->pRefIndex[0][iTopXy][12] : layer->pRefIndex[0][iTopXy][12];
	}else{
		ST32(iMvB,0);
		if(0==bTopAvail){						// not available
			iTopRef=REF_NOT_AVAIL;
		}else{									// available but is intra mb type
			iTopRef=REF_NOT_IN_LIST;
		}
	}
	if(REF_NOT_AVAIL==iTopRef || 
		(0==iTopRef && 0==*(int32_t*)iMvB)){
		ST32(iMvp,0);
		return;
	}
	// right_top
	if(bRightTopAvail && IS_INTER(iRightTopType)){
		ST32(iMvC,LD32(layer->pDec ? layer->pDec->pMv[0][iRightTopXy][12] :
			layer->pMv[0][iRightTopXy][12]));
		iRightTopRef=layer->pDec ? layer->pDec->pRefIndex[0][iRightTopXy][12] :
			layer->pRefIndex[0][iRightTopXy][12];
	}else{
		ST32(iMvC,0);
		if(0==bRightTopAvail){					// not available
			iRightTopRef=REF_NOT_AVAIL;
		}else{									// available but is intra mb type
			iRightTopRef=REF_NOT_IN_LIST;
		}
	}
	// left_top
	if(bLeftTopAvail && IS_INTER(iLeftTopType)){
		ST32(iMvD,LD32(layer->pDec ? layer->pDec->pMv[0][iLeftTopXy][15] : layer->pMv[0][iLeftTopXy][15]));
		iLeftTopRef=layer->pDec ? layer->pDec->pRefIndex[0][iLeftTopXy][15] :
			layer->pRefIndex[0][iLeftTopXy][15];
	}else{
		ST32(iMvD,0);
		if(0==bLeftTopAvail){					// not available
			iLeftTopRef=REF_NOT_AVAIL;
		}else{									// available but is intra mb type
			iLeftTopRef=REF_NOT_IN_LIST;
		}
	}
	iDiagonalRef=iRightTopRef;
	if(REF_NOT_AVAIL==iDiagonalRef){
		iDiagonalRef=iLeftTopRef;
		*(int32_t*)iMvC=*(int32_t*)iMvD;
	}
	if(REF_NOT_AVAIL==iTopRef && REF_NOT_AVAIL==iDiagonalRef && iLeftRef>=REF_NOT_IN_LIST){
		ST32(iMvp,LD32(iMvA));
		return;
	}
	iMatchRef=(0==iLeftRef)+(0==iTopRef)+(0==iDiagonalRef);
	if(1==iMatchRef){
		if(0==iLeftRef){
			ST32(iMvp,LD32(iMvA));
		}else
		if(0==iTopRef){
			ST32(iMvp,LD32(iMvB));
		}else{
			ST32(iMvp,LD32(iMvC));
		}
	}else{
		iMvp[0]=Median(iMvA[0],iMvB[0],iMvC[0]);
		iMvp[1]=Median(iMvA[1],iMvB[1],iMvC[1]);
	}
}
// extern at wels_common_defs.h
const uint8_t g_kuiChromaQpTable[52]={
	0,1,2,3,4,5,6,7,8,9,10,11,
	12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,
	28,29,29,30,31,32,32,33,34,34,35,35,36,36,37,37,
	37,38,38,38,39,39,39,39
};

uint32_t PlainH264Decoder::ParseMBTypePSliceCabac(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail){
	uint32_t uiCode;
	uint32_t uiMbType=0;
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+3,uiCode));
	if(uiCode){
		// Intra MB
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+6,uiCode));
		if(uiCode){		// Intra 16x16
			if(cabacDecEngine->DecodeTerminateCabac()){
				uiMbType=30;
				return uiMbType;	// MB_TYPE_INTRA_PCM;
			}
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+7,uiCode));
			uiMbType=6+uiCode*12;
			// uiCbp: 0,1,2
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+8,uiCode));
			if(uiCode){
				uiMbType+=4;
				READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+8,uiCode));
				if(uiCode)
					uiMbType+=4;
			}
			// IPredMode: 0,1,2,3
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+9,uiCode));
			uiMbType+=(uiCode<<1);
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+9,uiCode));
			uiMbType+=uiCode;
		}else
			// Intra 4x4
			uiMbType=5;
	}else{		// P MB
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+4,uiCode));
		if(uiCode){		// second bit
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+6,uiCode));
			if(uiCode)
				uiMbType=1;
			else
				uiMbType=2;
		}else{
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SKIP+5,uiCode));
			if(uiCode)
				uiMbType=3;
			else
				uiMbType=0;
		}
	}
	return uiMbType;
}

typedef struct TagPartMbInfo{
	MbType iType;
	int8_t iPartCount;		// P_16*16,P_16*8,P_8*16,P_8*8 based on 8*8 block; P_8*4,P_4*8,P_4*4 based on 4*4 block
	int8_t iPartWidth;		// based on 4*4 block
} SPartMbInfo;

// Table 7.13. Macroblock type values 0 to 4 for P slices.
static const SPartMbInfo g_ksInterPMbTypeInfo[5]={
	{MB_TYPE_16x16,1,4},
	{MB_TYPE_16x8,2,4},
	{MB_TYPE_8x16,2,2},
	{MB_TYPE_8x8,4,4},
	{MB_TYPE_8x8_REF0,4,4},// ref0--ref_idx not present in bit-stream and default as 0
};

void PlainH264Decoder::FillCacheNonZeroCount(const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,SDqLayer* layer){		// no matter slice type,intra_pred_constrained_flag
	int32_t iCurXy=layer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;
	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-layer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}
	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	if(pNeighAvail->iTopAvail){
		ST32(&pNonZeroCount[1],LD32(&layer->pNzc[iTopXy][12]));
		pNonZeroCount[0]=pNonZeroCount[5]=pNonZeroCount[29]=0;
		ST16(&pNonZeroCount[6],LD16(&layer->pNzc[iTopXy][20]));
		ST16(&pNonZeroCount[30],LD16(&layer->pNzc[iTopXy][22]));
	}else{
		ST32(&pNonZeroCount[1],0xFFFFFFFFU);
		pNonZeroCount[0]=pNonZeroCount[5]=pNonZeroCount[29]=0xFF;
		ST16(&pNonZeroCount[6],0xFFFF);
		ST16(&pNonZeroCount[30],0xFFFF);
	}
	if(pNeighAvail->iLeftAvail){
		pNonZeroCount[8*1]=layer->pNzc[iLeftXy][3];
		pNonZeroCount[8*2]=layer->pNzc[iLeftXy][7];
		pNonZeroCount[8*3]=layer->pNzc[iLeftXy][11];
		pNonZeroCount[8*4]=layer->pNzc[iLeftXy][15];

		pNonZeroCount[5+8*1]=layer->pNzc[iLeftXy][17];
		pNonZeroCount[5+8*2]=layer->pNzc[iLeftXy][21];
		pNonZeroCount[5+8*4]=layer->pNzc[iLeftXy][19];
		pNonZeroCount[5+8*5]=layer->pNzc[iLeftXy][23];
	}else{
		pNonZeroCount[8*1]=pNonZeroCount[8*2]=pNonZeroCount[8*3]=pNonZeroCount[8*4]=-1;	// unavailable
		pNonZeroCount[5+8*1]=pNonZeroCount[5+8*2]=-1;	// unavailable
		pNonZeroCount[5+8*4]=pNonZeroCount[5+8*5]=-1;	// unavailable
	}
}

void PlainH264Decoder::FillCacheInterCabac(SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,int16_t iMvArray[LIST_A][30][MV_A],int16_t iMvdCache[LIST_A][30][MV_A],int8_t iRefIdxArray[LIST_A][30],SDqLayer* layer){
	int32_t iCurXy=layer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;
	int32_t iLeftTopXy=0;
	int32_t iRightTopXy=0;

	//SSlice* slice=&pCtx->m_slice;
	int32_t listCount=1;
	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	FillCacheNonZeroCount(pNeighAvail,pNonZeroCount,layer);

	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-layer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}
	if(pNeighAvail->iLeftTopAvail){
		iLeftTopXy=iCurXy-1-layer->iMbWidth;
	}
	if(pNeighAvail->iRightTopAvail){
		iRightTopXy=iCurXy+1-layer->iMbWidth;
	}

	for(int32_t listIdx=0; listIdx<listCount;++listIdx){
		// stuff mv_cache and iRefIdxArray from left and top (inter)
		if(pNeighAvail->iLeftAvail && IS_INTER(pNeighAvail->iLeftType)){
			ST32(iMvArray[listIdx][6],LD32(layer->pDec->pMv[listIdx][iLeftXy][3]));
			ST32(iMvArray[listIdx][12],LD32(layer->pDec->pMv[listIdx][iLeftXy][7]));
			ST32(iMvArray[listIdx][18],LD32(layer->pDec->pMv[listIdx][iLeftXy][11]));
			ST32(iMvArray[listIdx][24],LD32(layer->pDec->pMv[listIdx][iLeftXy][15]));

			ST32(iMvdCache[listIdx][6],LD32(layer->pMvd[listIdx][iLeftXy][3]));
			ST32(iMvdCache[listIdx][12],LD32(layer->pMvd[listIdx][iLeftXy][7]));
			ST32(iMvdCache[listIdx][18],LD32(layer->pMvd[listIdx][iLeftXy][11]));
			ST32(iMvdCache[listIdx][24],LD32(layer->pMvd[listIdx][iLeftXy][15]));

			iRefIdxArray[listIdx][6]=layer->pDec->pRefIndex[listIdx][iLeftXy][3];
			iRefIdxArray[listIdx][12]=layer->pDec->pRefIndex[listIdx][iLeftXy][7];
			iRefIdxArray[listIdx][18]=layer->pDec->pRefIndex[listIdx][iLeftXy][11];
			iRefIdxArray[listIdx][24]=layer->pDec->pRefIndex[listIdx][iLeftXy][15];
		}else{
			ST32(iMvArray[listIdx][6],0);
			ST32(iMvArray[listIdx][12],0);
			ST32(iMvArray[listIdx][18],0);
			ST32(iMvArray[listIdx][24],0);

			ST32(iMvdCache[listIdx][6],0);
			ST32(iMvdCache[listIdx][12],0);
			ST32(iMvdCache[listIdx][18],0);
			ST32(iMvdCache[listIdx][24],0);
			if(0==pNeighAvail->iLeftAvail){		// not available
				iRefIdxArray[listIdx][6]=iRefIdxArray[listIdx][12]=iRefIdxArray[listIdx][18]=iRefIdxArray[listIdx][24]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][6]=iRefIdxArray[listIdx][12]=iRefIdxArray[listIdx][18]=iRefIdxArray[listIdx][24]=REF_NOT_IN_LIST;
			}
		}
		if(pNeighAvail->iLeftTopAvail && IS_INTER(pNeighAvail->iLeftTopType)){
			ST32(iMvArray[listIdx][0],LD32(layer->pDec->pMv[listIdx][iLeftTopXy][15]));
			ST32(iMvdCache[listIdx][0],LD32(layer->pMvd[listIdx][iLeftTopXy][15]));
			iRefIdxArray[listIdx][0]=layer->pDec->pRefIndex[listIdx][iLeftTopXy][15];
		}else{
			ST32(iMvArray[listIdx][0],0);
			ST32(iMvdCache[listIdx][0],0);
			if(0==pNeighAvail->iLeftTopAvail){		// not available
				iRefIdxArray[listIdx][0]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][0]=REF_NOT_IN_LIST;
			}
		}

		if(pNeighAvail->iTopAvail && IS_INTER(pNeighAvail->iTopType)){
			ST64(iMvArray[listIdx][1],LD64(layer->pDec->pMv[listIdx][iTopXy][12]));
			ST64(iMvArray[listIdx][3],LD64(layer->pDec->pMv[listIdx][iTopXy][14]));
			ST64(iMvdCache[listIdx][1],LD64(layer->pMvd[listIdx][iTopXy][12]));
			ST64(iMvdCache[listIdx][3],LD64(layer->pMvd[listIdx][iTopXy][14]));
			ST32(&iRefIdxArray[listIdx][1],LD32(&layer->pDec->pRefIndex[listIdx][iTopXy][12]));
		}else{
			ST64(iMvArray[listIdx][1],0);
			ST64(iMvArray[listIdx][3],0);
			ST64(iMvdCache[listIdx][1],0);
			ST64(iMvdCache[listIdx][3],0);
			if(0==pNeighAvail->iTopAvail){			// not available
				iRefIdxArray[listIdx][1]=iRefIdxArray[listIdx][2]=iRefIdxArray[listIdx][3]=iRefIdxArray[listIdx][4]=REF_NOT_AVAIL;
			}else{									// available but is intra mb type
				iRefIdxArray[listIdx][1]=iRefIdxArray[listIdx][2]=iRefIdxArray[listIdx][3]=iRefIdxArray[listIdx][4]=REF_NOT_IN_LIST;
			}
		}

		if(pNeighAvail->iRightTopAvail && IS_INTER(pNeighAvail->iRightTopType)){
			ST32(iMvArray[listIdx][5],LD32(layer->pDec->pMv[listIdx][iRightTopXy][12]));
			ST32(iMvdCache[listIdx][5],LD32(layer->pMvd[listIdx][iRightTopXy][12]));
			iRefIdxArray[listIdx][5]=layer->pDec->pRefIndex[listIdx][iRightTopXy][12];
		}else{
			ST32(iMvArray[listIdx][5],0);
			if(0==pNeighAvail->iRightTopAvail){		// not available
				iRefIdxArray[listIdx][5]=REF_NOT_AVAIL;
			}else{									// available but is intra mb type
				iRefIdxArray[listIdx][5]=REF_NOT_IN_LIST;
			}
		}
// right-top 4*4 block unavailable
		ST32(iMvArray[listIdx][9],0);
		ST32(iMvArray[listIdx][21],0);
		ST32(iMvArray[listIdx][11],0);
		ST32(iMvArray[listIdx][17],0);
		ST32(iMvArray[listIdx][23],0);
		ST32(iMvdCache[listIdx][9],0);
		ST32(iMvdCache[listIdx][21],0);
		ST32(iMvdCache[listIdx][11],0);
		ST32(iMvdCache[listIdx][17],0);
		ST32(iMvdCache[listIdx][23],0);
		iRefIdxArray[listIdx][9]=iRefIdxArray[listIdx][21]=iRefIdxArray[listIdx][11]=iRefIdxArray[listIdx][17]=iRefIdxArray[listIdx][23]=REF_NOT_AVAIL;
	}
}

// cache element equal to 30
const uint8_t g_kuiCache30ScanIdx[16]={		// mv or uiRefIndex cache scan index,4*4 block as basic unit
	7,8,13,14,
	9,10,15,16,
	19,20,25,26,
	21,22,27,28
};

// basic iMVs prediction unit for iMVs partition width (4,2,1)
void PlainH264Decoder::PredMv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int32_t iPartWidth,int8_t iRef,int16_t iMVP[2]){
	const uint8_t kuiLeftIdx=g_kuiCache30ScanIdx[iPartIdx]-1;
	const uint8_t kuiTopIdx=g_kuiCache30ScanIdx[iPartIdx]-6;
	const uint8_t kuiRightTopIdx=kuiTopIdx+iPartWidth;
	const uint8_t kuiLeftTopIdx=kuiTopIdx-1;
	const int8_t kiLeftRef=iRefIndex[listIdx][kuiLeftIdx];
	const int8_t kiTopRef=iRefIndex[listIdx][kuiTopIdx];
	const int8_t kiRightTopRef=iRefIndex[listIdx][kuiRightTopIdx];
	const int8_t kiLeftTopRef=iRefIndex[listIdx][kuiLeftTopIdx];
	int8_t iDiagonalRef=kiRightTopRef;
	int8_t iMatchRef=0;
	int16_t iAMV[2],iBMV[2],iCMV[2];
	ST32(iAMV,LD32(iMotionVector[listIdx][kuiLeftIdx]));
	ST32(iBMV,LD32(iMotionVector[listIdx][kuiTopIdx]));
	ST32(iCMV,LD32(iMotionVector[listIdx][kuiRightTopIdx]));
	if(REF_NOT_AVAIL==iDiagonalRef){
		iDiagonalRef=kiLeftTopRef;
		ST32(iCMV,LD32(iMotionVector[listIdx][kuiLeftTopIdx]));
	}
	iMatchRef=(iRef==kiLeftRef)+(iRef==kiTopRef)+(iRef==iDiagonalRef);
	if(REF_NOT_AVAIL==kiTopRef && REF_NOT_AVAIL==iDiagonalRef && kiLeftRef>=REF_NOT_IN_LIST){
		ST32(iMVP,LD32(iAMV));
		return;
	}
	if(1==iMatchRef){
		if(iRef==kiLeftRef){
			ST32(iMVP,LD32(iAMV));
		}else
		if(iRef==kiTopRef){
			ST32(iMVP,LD32(iBMV));
		}else{
			ST32(iMVP,LD32(iCMV));
		}
	}else{
		iMVP[0]=Median(iAMV[0],iBMV[0],iCMV[0]);
		iMVP[1]=Median(iAMV[1],iBMV[1],iCMV[1]);
	}
}

int32_t PlainH264Decoder::ParseMvdInfoCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,int8_t pRefIndex[LIST_A][30],int16_t pMvdCache[LIST_A][30][2],int32_t index,int8_t iListIdx,int8_t iMvComp,int16_t& iMvdVal){
	uint32_t uiCode;
	int32_t iIdxA=0;
	//SCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_MVD+iMvComp*CTX_NUM_MVD;
	iMvdVal=0;
	if(pRefIndex[iListIdx][g_kuiCache30ScanIdx[index]-6]>=0)
		iIdxA=WELS_ABS(pMvdCache[iListIdx][g_kuiCache30ScanIdx[index]-6][iMvComp]);
	if(pRefIndex[iListIdx][g_kuiCache30ScanIdx[index]-1]>=0)
		iIdxA+=WELS_ABS(pMvdCache[iListIdx][g_kuiCache30ScanIdx[index]-1][iMvComp]);
	int32_t iCtxInc=0;
	if(iIdxA>=3)
		iCtxInc=1+(iIdxA>32);
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_MVD+(iMvComp*CTX_NUM_MVD)+iCtxInc,uiCode));
	if(uiCode){
		READ_VERIFY(cabacDecEngine->DecodeUEGMvCabac(NEW_CTX_OFFSET_MVD+(iMvComp*CTX_NUM_MVD)+3,3,uiCode));
		iMvdVal=(int16_t)(uiCode+1);
		READ_VERIFY(cabacDecEngine->DecodeBypassCabac(uiCode));
		if(uiCode){
			iMvdVal=-iMvdVal;
		}
	}else{
		iMvdVal=0;
	}
	return ERR_NONE;
}

// update iMVs and iRefIndex cache for current MB,only for P_16*16 (SKIP inclusive)
void PlainH264Decoder::UpdateP16x16MotionInfo(SDqLayer* layer,int32_t listIdx,int8_t iRef,int16_t iMVs[2]){
	const int16_t kiRef2=((uint8_t)iRef<<8)|(uint8_t)iRef;
	const int32_t kiMV32=LD32(iMVs);
	int32_t i;
	int32_t iMbXy=layer->iMbXyIndex;

	for(i=0; i<16; i+=4){
		// mb
		const uint8_t kuiScan4Idx=g_kuiScan4[i];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		if(layer->pDec!=NULL){
			ST16(&layer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&layer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);

			ST32(layer->pDec->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}else{
			ST16(&layer->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&layer->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);

			ST32(layer->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}
	}
}

void PlainH264Decoder::UpdateP16x16MvdCabac(SDqLayer* layer,int16_t pMvd[2],const int8_t iListIdx){
	int32_t pMvd32[2];
	ST32(&pMvd32[0],LD32(pMvd));
	ST32(&pMvd32[1],LD32(pMvd));
	int32_t i;
	int32_t iMbXy=layer->iMbXyIndex;
	for(i=0; i<16; i+=2){
		ST64(layer->pMvd[iListIdx][iMbXy][i],LD64(pMvd32));
	}
}

void PlainH264Decoder::UpdateP16x8RefIdxCabac(SDqLayer* layer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx){
	uint32_t iRef32Bit=(uint32_t)iRef;
	const int32_t iRef4Bytes=(iRef32Bit<<24)|(iRef32Bit<<16)|(iRef32Bit<<8)|iRef32Bit;
	int32_t iMbXy=layer->iMbXyIndex;
	const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
	const uint8_t iScan4Idx4=4+iScan4Idx;
	const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
	const uint8_t iCacheIdx6=6+iCacheIdx;
	// mb
	ST32(&layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx],iRef4Bytes);
	ST32(&layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx4],iRef4Bytes);
	// cache
	ST32(&pRefIndex[iListIdx][iCacheIdx],iRef4Bytes);
	ST32(&pRefIndex[iListIdx][iCacheIdx6],iRef4Bytes);
}

void PlainH264Decoder::PredInter16x8Mv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVP[2]){
	if(0==iPartIdx){
		const int8_t kiTopRef=iRefIndex[listIdx][1];
		if(iRef==kiTopRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][1][0]));
			return;
		}
	}else{		// 8==iPartIdx
		const int8_t kiLeftRef=iRefIndex[listIdx][18];
		if(iRef==kiLeftRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][18][0]));
			return;
		}
	}

	PredMv(iMotionVector,iRefIndex,listIdx,iPartIdx,4,iRef,iMVP);
}

// update iRefIndex and iMVs of Mb,only for P16x8
// need further optimization,mb_cache not work
void PlainH264Decoder::UpdateP16x8MotionInfo(SDqLayer* layer,int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVs[2]){
	const int16_t kiRef2=((uint8_t)iRef<<8)|(uint8_t)iRef;
	const int32_t kiMV32=LD32(iMVs);
	int32_t i;
	int32_t iMbXy=layer->iMbXyIndex;
	for(i=0; i<2; i++,iPartIdx+=4){
		const uint8_t kuiScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		const uint8_t kuiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t kuiCacheIdxPlus6=6+kuiCacheIdx;

		// mb
		if(layer->pDec!=NULL){
			ST16(&layer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&layer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(layer->pDec->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}else{
			ST16(&layer->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&layer->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(layer->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}
		// cache
		ST16(&iRefIndex[listIdx][kuiCacheIdx],kiRef2);
		ST16(&iRefIndex[listIdx][kuiCacheIdxPlus6],kiRef2);
		ST32(iMotionVector[listIdx][kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][kuiCacheIdxPlus6],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdxPlus6],kiMV32);
	}
}

void PlainH264Decoder::UpdateP16x8MvdCabac(SDqLayer* layer,int16_t pMvdCache[LIST_A][30][MV_A],int32_t iPartIdx,int16_t pMvd[2],const int8_t iListIdx){
	int32_t pMvd32[2];
	ST32(&pMvd32[0],LD32(pMvd));
	ST32(&pMvd32[1],LD32(pMvd));
	int32_t i;
	int32_t iMbXy=layer->iMbXyIndex;
	for(i=0; i<2; i++,iPartIdx+=4){
		const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t iScan4Idx4=4+iScan4Idx;
		const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t iCacheIdx6=6+iCacheIdx;
		// mb
		ST64(layer->pMvd[iListIdx][iMbXy][iScan4Idx],LD64(pMvd32));
		ST64(layer->pMvd[iListIdx][iMbXy][iScan4Idx4],LD64(pMvd32));
		// cache
		ST64(pMvdCache[iListIdx][iCacheIdx],LD64(pMvd32));
		ST64(pMvdCache[iListIdx][iCacheIdx6],LD64(pMvd32));
	}
}

void PlainH264Decoder::UpdateP8x16RefIdxCabac(SDqLayer* layer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx){
	uint16_t iRef16Bit=(uint16_t)iRef;
	const int16_t iRef2Bytes=(iRef16Bit<<8)|iRef16Bit;
	int32_t i;
	int32_t iMbXy=layer->iMbXyIndex;
	for(i=0; i<2; i++,iPartIdx+=8){
		const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t iScan4Idx4=4+iScan4Idx;
		const uint8_t iCacheIdx6=6+iCacheIdx;
		// mb
		ST16(&layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx],iRef2Bytes);
		ST16(&layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx4],iRef2Bytes);
		// cache
		ST16(&pRefIndex[iListIdx][iCacheIdx],iRef2Bytes);
		ST16(&pRefIndex[iListIdx][iCacheIdx6],iRef2Bytes);
	}
}

void PlainH264Decoder::PredInter8x16Mv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVP[2]){
	if(0==iPartIdx){
		const int8_t kiLeftRef=iRefIndex[listIdx][6];
		if(iRef==kiLeftRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][6][0]));
			return;
		}
	}else{		// 1==iPartIdx
		int8_t iDiagonalRef=iRefIndex[listIdx][5];		// top-right
		int8_t index=5;
		if(REF_NOT_AVAIL==iDiagonalRef){
			iDiagonalRef=iRefIndex[listIdx][2];		// top-left for 8*8 block(index 1)
			index=2;
		}
		if(iRef==iDiagonalRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][index][0]));
			return;
		}
	}

	PredMv(iMotionVector,iRefIndex,listIdx,iPartIdx,2,iRef,iMVP);
}

// update iRefIndex and iMVs of both Mb and Mb_cache,only for P8x16
void PlainH264Decoder::UpdateP8x16MotionInfo(SDqLayer* layer,int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVs[2]){
	const int16_t kiRef2=((uint8_t)iRef<<8)|(uint8_t)iRef;
	const int32_t kiMV32=LD32(iMVs);
	int32_t i;
	int32_t iMbXy=layer->iMbXyIndex;

	for(i=0; i<2; i++,iPartIdx+=8){
		const uint8_t kuiScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t kuiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		const uint8_t kuiCacheIdxPlus6=6+kuiCacheIdx;

		// mb
		if(layer->pDec!=NULL){
			ST16(&layer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&layer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(layer->pDec->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(layer->pDec->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}else{
			ST16(&layer->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&layer->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(layer->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(layer->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}
		// cache
		ST16(&iRefIndex[listIdx][kuiCacheIdx],kiRef2);
		ST16(&iRefIndex[listIdx][kuiCacheIdxPlus6],kiRef2);
		ST32(iMotionVector[listIdx][kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][kuiCacheIdxPlus6],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdxPlus6],kiMV32);
	}
}

void PlainH264Decoder::UpdateP8x16MvdCabac(SDqLayer* layer,int16_t pMvdCache[LIST_A][30][MV_A],int32_t iPartIdx,int16_t pMvd[2],const int8_t iListIdx){
	int32_t pMvd32[2];
	ST32(&pMvd32[0],LD32(pMvd));
	ST32(&pMvd32[1],LD32(pMvd));
	int32_t i;
	int32_t iMbXy=layer->iMbXyIndex;

	for(i=0; i<2; i++,iPartIdx+=8){
		const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t iScan4Idx4=4+iScan4Idx;
		const uint8_t iCacheIdx6=6+iCacheIdx;
		// mb
		ST64(layer->pMvd[iListIdx][iMbXy][iScan4Idx],LD64(pMvd32));
		ST64(layer->pMvd[iListIdx][iMbXy][iScan4Idx4],LD64(pMvd32));
		// cache
		ST64(pMvdCache[iListIdx][iCacheIdx],LD64(pMvd32));
		ST64(pMvdCache[iListIdx][iCacheIdx6],LD64(pMvd32));
	}
}

int32_t PlainH264Decoder::ParseSubMBTypeCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint32_t& uiSubMbType){
	uint32_t uiCode;
	//SCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_SUBMB_TYPE;
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SUBMB_TYPE,uiCode));
	if(uiCode)
		uiSubMbType=0;
	else{
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SUBMB_TYPE+1,uiCode));
		if(uiCode){
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_SUBMB_TYPE+2,uiCode));
			uiSubMbType=3-uiCode;
		}else{
			uiSubMbType=1;
		}
	}
	return ERR_NONE;
}

// Table 7.17 Sub-macroblock types in B macroblocks.
static const SPartMbInfo g_ksInterPSubMbTypeInfo[4]={
	{SUB_MB_TYPE_8x8,1,2},
	{SUB_MB_TYPE_8x4,2,2},
	{SUB_MB_TYPE_4x8,2,1},
	{SUB_MB_TYPE_4x4,4,1},
};

void UpdateP8x8RefIdxCabac(SDqLayer* layer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx){
	int32_t iMbXy=layer->iMbXyIndex;
	const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
	layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx]=layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx+1]=layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx+4]=layer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx+5]=iRef;
}

int32_t PlainH264Decoder::ParseInterPMotionInfoCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,int16_t pMotionVector[LIST_A][30][MV_A],int16_t pMvdCache[LIST_A][30][MV_A],int8_t pRefIndex[LIST_A][30]){
	//SSlice* slice=&pCtx->m_slice;
	SDqLayer* layer=&pCtx->m_layer;
	int32_t i,j;
	int32_t iMbXy=layer->iMbXyIndex;
	int16_t pMv[4]={0};
	int16_t pMvd[4]={0};
	int32_t iPartIdx;
	int16_t iMinVmv=pCtx->m_sps.pSLevelLimits->iMinVmv;
	int16_t iMaxVmv=pCtx->m_sps.pSLevelLimits->iMaxVmv;
	int8_t iRef=0;		//Always prev ref
	switch(layer->pDec->pMbType[iMbXy]){
		case MB_TYPE_16x16:
		{
			iPartIdx=0;
			PredMv(pMotionVector,pRefIndex,LIST_0,0,4,iRef,pMv);
			READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
			READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
			pMv[0]+=pMvd[0];
			pMv[1]+=pMvd[1];
			WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
			UpdateP16x16MotionInfo(layer,LIST_0,iRef,pMv);
			UpdateP16x16MvdCabac(layer,pMvd,LIST_0);
		}
		break;
		case MB_TYPE_16x8:
			for(i=0; i<2; i++){
				iPartIdx=i<<3;
				UpdateP16x8RefIdxCabac(layer,pRefIndex,iPartIdx,iRef,LIST_0);
			}
			for(i=0; i<2; i++){
				iPartIdx=i<<3;
				PredInter16x8Mv(pMotionVector,pRefIndex,LIST_0,iPartIdx,iRef,pMv);
				READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
				READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
				pMv[0]+=pMvd[0];
				pMv[1]+=pMvd[1];
				WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
				UpdateP16x8MotionInfo(layer,pMotionVector,pRefIndex,LIST_0,iPartIdx,iRef,pMv);
				UpdateP16x8MvdCabac(layer,pMvdCache,iPartIdx,pMvd,LIST_0);
			}
			break;
		case MB_TYPE_8x16:
			for(i=0; i<2; i++){
				iPartIdx=i<<2;
				UpdateP8x16RefIdxCabac(layer,pRefIndex,iPartIdx,iRef,LIST_0);
			}
			for(i=0; i<2; i++){
				iPartIdx=i<<2;
				PredInter8x16Mv(pMotionVector,pRefIndex,LIST_0,i<<2,iRef,pMv);
				READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
				READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
				pMv[0]+=pMvd[0];
				pMv[1]+=pMvd[1];
				WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
				UpdateP8x16MotionInfo(layer,pMotionVector,pRefIndex,LIST_0,iPartIdx,iRef,pMv);
				UpdateP8x16MvdCabac(layer,pMvdCache,iPartIdx,pMvd,LIST_0);
			}
			break;
		case MB_TYPE_8x8:
		case MB_TYPE_8x8_REF0:
		{
			int8_t pRefIdx[4]={0},pSubPartCount[4],pPartW[4];
			uint32_t uiSubMbType;
// sub_mb_type,partition
			for(i=0; i<4; i++){
				READ_VERIFY(ParseSubMBTypeCabac(cabacDecEngine,pCtx,pNeighAvail,uiSubMbType));
				if(uiSubMbType>=4){		// invalid sub_mb_type
					return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_SUB_MB_TYPE);
				}
				layer->pSubMbType[iMbXy][i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iType;
				pSubPartCount[i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iPartCount;
				pPartW[i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iPartWidth;
// Need modification when B picture add in,reference to 7.3.5
				layer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]&=(uiSubMbType==0);
			}
			for(i=0; i<4; i++){
				int16_t iIdx8=i<<2;
				UpdateP8x8RefIdxCabac(layer,pRefIndex,iIdx8,pRefIdx[i],LIST_0);
			}
			// mv
			for(i=0; i<4; i++){
				int8_t iPartCount=pSubPartCount[i];
				uiSubMbType=layer->pSubMbType[iMbXy][i];
				int16_t iPartIdx,iBlockW=pPartW[i];
				uint8_t iScan4Idx,iCacheIdx;
				iCacheIdx=g_kuiCache30ScanIdx[i<<2];
				pRefIndex[0][iCacheIdx]=pRefIndex[0][iCacheIdx+1]=pRefIndex[0][iCacheIdx+6]=pRefIndex[0][iCacheIdx+7]=pRefIdx[i];
				for(j=0; j<iPartCount; j++){
					iPartIdx=(i<<2)+j*iBlockW;
					iScan4Idx=g_kuiScan4[iPartIdx];
					iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
					PredMv(pMotionVector,pRefIndex,LIST_0,iPartIdx,iBlockW,pRefIdx[i],pMv);
					READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
					READ_VERIFY(ParseMvdInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
					pMv[0]+=pMvd[0];
					pMv[1]+=pMvd[1];
					WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
					if(SUB_MB_TYPE_8x8==uiSubMbType){
						ST32((pMv+2),LD32(pMv));
						ST32((pMvd+2),LD32(pMvd));
						ST64(layer->pDec->pMv[0][iMbXy][iScan4Idx],LD64(pMv));
						ST64(layer->pDec->pMv[0][iMbXy][iScan4Idx+4],LD64(pMv));
						ST64(layer->pMvd[0][iMbXy][iScan4Idx],LD64(pMvd));
						ST64(layer->pMvd[0][iMbXy][iScan4Idx+4],LD64(pMvd));
						ST64(pMotionVector[0][iCacheIdx],LD64(pMv));
						ST64(pMotionVector[0][iCacheIdx+6],LD64(pMv));
						ST64(pMvdCache[0][iCacheIdx],LD64(pMvd));
						ST64(pMvdCache[0][iCacheIdx+6],LD64(pMvd));
					}else
					if(SUB_MB_TYPE_8x4==uiSubMbType){
						ST32((pMv+2),LD32(pMv));
						ST32((pMvd+2),LD32(pMvd));
						ST64(layer->pDec->pMv[0][iMbXy][iScan4Idx],LD64(pMv));
						ST64(layer->pMvd[0][iMbXy][iScan4Idx],LD64(pMvd));
						ST64(pMotionVector[0][iCacheIdx],LD64(pMv));
						ST64(pMvdCache[0][iCacheIdx],LD64(pMvd));
					}else
					if(SUB_MB_TYPE_4x8==uiSubMbType){
						ST32(layer->pDec->pMv[0][iMbXy][iScan4Idx],LD32(pMv));
						ST32(layer->pDec->pMv[0][iMbXy][iScan4Idx+4],LD32(pMv));
						ST32(layer->pMvd[0][iMbXy][iScan4Idx],LD32(pMvd));
						ST32(layer->pMvd[0][iMbXy][iScan4Idx+4],LD32(pMvd));
						ST32(pMotionVector[0][iCacheIdx],LD32(pMv));
						ST32(pMotionVector[0][iCacheIdx+6],LD32(pMv));
						ST32(pMvdCache[0][iCacheIdx],LD32(pMvd));
						ST32(pMvdCache[0][iCacheIdx+6],LD32(pMvd));
					}else{		// SUB_MB_TYPE_4x4
						ST32(layer->pDec->pMv[0][iMbXy][iScan4Idx],LD32(pMv));
						ST32(layer->pMvd[0][iMbXy][iScan4Idx],LD32(pMvd));
						ST32(pMotionVector[0][iCacheIdx],LD32(pMv));
						ST32(pMvdCache[0][iCacheIdx],LD32(pMvd));
					}
				}
			}
		}
		break;
		default:
			break;
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseTransformSize8x8FlagCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,bool& bTransformSize8x8Flag){
	uint32_t uiCode;
	int32_t iIdxA,iIdxB;
	int32_t iCtxInc;
	//SCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_TS_8x8_FLAG;
	iIdxA=(pNeighAvail->iLeftAvail) && (pCtx->m_layer.pTransformSize8x8Flag[pCtx->m_layer.iMbXyIndex-1]);
	iIdxB=(pNeighAvail->iTopAvail) && (pCtx->m_layer.pTransformSize8x8Flag[pCtx->m_layer.iMbXyIndex-pCtx->m_layer.iMbWidth]);
	iCtxInc=iIdxA+iIdxB;
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_TS_8x8_FLAG+iCtxInc,uiCode));
	bTransformSize8x8Flag=!!uiCode;
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseIntraPredModeLumaCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,int32_t& iBinVal){
	uint32_t uiCode;
	iBinVal=0;
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_IPR,uiCode));
	if(uiCode==1)
		iBinVal=-1;
	else{
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_IPR+1,uiCode));
		iBinVal|=uiCode;
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_IPR+1,uiCode));
		iBinVal|=(uiCode<<1);
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_IPR+1,uiCode));
		iBinVal|=(uiCode<<2);
	}
	return ERR_NONE;
}

int32_t PredIntra4x4Mode(int8_t* pIntraPredMode,int32_t iIdx4){
	int8_t iTopMode=pIntraPredMode[g_kuiScan8[iIdx4]-8];
	int8_t iLeftMode=pIntraPredMode[g_kuiScan8[iIdx4]-1];
	int8_t iBestMode;
	if(-1==iLeftMode || -1==iTopMode){
		iBestMode=2;
	}else{
		iBestMode=WELS_MIN(iLeftMode,iTopMode);
	}
	return iBestMode;
}


typedef struct TagI16PredInfo{
	int8_t iPredMode;
	int8_t iLeftAvail;
	int8_t iTopAvail;
	int8_t iLeftTopAvail;
} SI16PredInfo;
static const SI16PredInfo g_ksI16PredInfo[4]={
	{I16_PRED_V,0,1,0},
	{I16_PRED_H,1,0,0},
	{0,0,0,0},
	{I16_PRED_P,1,1,1},
};

static const SI16PredInfo g_ksChromaPredInfo[4]={
	{0,0,0,0},
	{C_PRED_H,1,0,0},
	{C_PRED_V,0,1,0},
	{C_PRED_P,1,1,1},
};

typedef struct TagI4PredInfo{
	int8_t iPredMode;
	int8_t iLeftAvail;
	int8_t iTopAvail;
	int8_t iLeftTopAvail;
} SI4PredInfo;
static const SI4PredInfo g_ksI4PredInfo[9]={
	{I4_PRED_V,0,1,0},
	{I4_PRED_H,1,0,0},
	{0,0,0,0},
	{I4_PRED_DDL,0,1,0},
	{I4_PRED_DDR,1,1,1},
	{I4_PRED_VR,1,1,1},
	{I4_PRED_HD,1,1,1},
	{I4_PRED_VL,0,1,0},
	{I4_PRED_HU,1,0,0},
};

#define CHECK_I16_MODE(a,b,c,d) \
 ((a==g_ksI16PredInfo[a].iPredMode) && \
 (b >=g_ksI16PredInfo[a].iLeftAvail) && \
 (c >=g_ksI16PredInfo[a].iTopAvail) && \
 (d >=g_ksI16PredInfo[a].iLeftTopAvail));
#define CHECK_CHROMA_MODE(a,b,c,d) \
 ((a==g_ksChromaPredInfo[a].iPredMode) && \
 (b >=g_ksChromaPredInfo[a].iLeftAvail) && \
 (c >=g_ksChromaPredInfo[a].iTopAvail) && \
 (d >=g_ksChromaPredInfo[a].iLeftTopAvail));
#define CHECK_I4_MODE(a,b,c,d) \
 ((a==g_ksI4PredInfo[a].iPredMode) && \
 (b >=g_ksI4PredInfo[a].iLeftAvail) && \
 (c >=g_ksI4PredInfo[a].iTopAvail) && \
 (d >=g_ksI4PredInfo[a].iLeftTopAvail));


int32_t PlainH264Decoder::CheckIntraNxNPredMode(int32_t* pSampleAvail,int8_t* pMode,int32_t iIndex,bool b8x8){
	int8_t iIdx=g_kuiCache30ScanIdx[iIndex];
	int32_t iLeftAvail=pSampleAvail[iIdx-1];
	int32_t iTopAvail=pSampleAvail[iIdx-6];
	int32_t bLeftTopAvail=pSampleAvail[iIdx-7];
	int32_t bRightTopAvail=pSampleAvail[iIdx-(b8x8 ? 4 : 5)];		// Diff with 4x4 Pred
	int8_t iFinalMode;
	if((*pMode<0) || (*pMode>MAX_PRED_MODE_ID_I4x4)){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE);
	}
	if(I4_PRED_DC==*pMode){
		if(iLeftAvail && iTopAvail){
			return *pMode;
		}else
		if(iLeftAvail){
			iFinalMode=I4_PRED_DC_L;
		}else
		if(iTopAvail){
			iFinalMode=I4_PRED_DC_T;
		}else{
			iFinalMode=I4_PRED_DC_128;
		}
	}else{
		bool bModeAvail=CHECK_I4_MODE(*pMode,iLeftAvail,iTopAvail,bLeftTopAvail);
		if(0==bModeAvail){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE);
		}
		iFinalMode=*pMode;
		// if right-top unavailable,modify mode DDL and VL (padding rightmost pixel of top)
		if(I4_PRED_DDL==iFinalMode && 0==bRightTopAvail){
			iFinalMode=I4_PRED_DDL_TOP;
		}else
		if(I4_PRED_VL==iFinalMode && 0==bRightTopAvail){
			iFinalMode=I4_PRED_VL_TOP;
		}
	}
	return iFinalMode;
}

int32_t PlainH264Decoder::ParseIntraPredModeChromaCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,uint8_t uiNeighAvail,int32_t& iBinVal){
	uint32_t uiCode;
	int32_t iIdxA,iIdxB,iCtxInc;
	int8_t* pChromaPredMode=pCtx->m_layer.pChromaPredMode;
	uint32_t* pMbType=pCtx->m_layer.pDec->pMbType;
	int32_t iLeftAvail=uiNeighAvail&0x04;
	int32_t iTopAvail=uiNeighAvail&0x01;

	int32_t iMbXy=pCtx->m_layer.iMbXyIndex;
	int32_t iMbXyTop=iMbXy-pCtx->m_layer.iMbWidth;
	int32_t iMbXyLeft=iMbXy-1;

	iBinVal=0;

	iIdxB=iTopAvail && (pChromaPredMode[iMbXyTop]>0 && pChromaPredMode[iMbXyTop]<=3) && pMbType[iMbXyTop]!=MB_TYPE_INTRA_PCM;
	iIdxA=iLeftAvail && (pChromaPredMode[iMbXyLeft]>0 && pChromaPredMode[iMbXyLeft]<=3) && pMbType[iMbXyLeft]!=MB_TYPE_INTRA_PCM;
	iCtxInc=iIdxA+iIdxB;
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CIPR+iCtxInc,uiCode));
	iBinVal=uiCode;
	if(iBinVal!=0){
		uint32_t iSym;
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CIPR+3,iSym));
		if(iSym==0){
			iBinVal=(iSym+1);
			return ERR_NONE;
		}
		iSym=0;
		do{
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CIPR+3,uiCode));
			++iSym;
		} while((uiCode!=0) && (iSym<1));

		if((uiCode!=0) && (iSym==1))
			++iSym;
		iBinVal=(iSym+1);
		return ERR_NONE;
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::CheckIntraChromaPredMode(uint8_t uiSampleAvail,int8_t* pMode){
	int32_t iLeftAvail=uiSampleAvail&0x04;
	int32_t bLeftTopAvail=uiSampleAvail&0x02;
	int32_t iTopAvail=uiSampleAvail&0x01;

	if(C_PRED_DC==*pMode){
		if(iLeftAvail && iTopAvail){
			return ERR_NONE;
		}else
		if(iLeftAvail){
			*pMode=C_PRED_DC_L;
		}else
		if(iTopAvail){
			*pMode=C_PRED_DC_T;
		}else{
			*pMode=C_PRED_DC_128;
		}
	}else{
		bool bModeAvail=CHECK_CHROMA_MODE(*pMode,iLeftAvail,iTopAvail,bLeftTopAvail);
		if(0==bModeAvail){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseIntra8x8Mode(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,int8_t* pIntraPredMode,SBitStringAux* pBs,SDqLayer* layer){
	// Similar with Intra_4x4,can put them together when needed
	int32_t iSampleAvail[5*6]={0};		// initialize as 0
	int32_t iMbXy=layer->iMbXyIndex;
	int32_t iFinalMode,i;
	uint8_t uiNeighAvail=0;
	int32_t iCode;
	MapNxNNeighToSampleNormal(pNeighAvail,iSampleAvail);
	// Top-Right : Left : Top-Left : Top
	uiNeighAvail=(iSampleAvail[5]<<3)|(iSampleAvail[6]<<2)|(iSampleAvail[0]<<1)|(iSampleAvail[1]);
	layer->pIntraNxNAvailFlag[iMbXy]=uiNeighAvail;
	for(i=0; i<4; i++){
		int32_t iPrevIntra4x4PredMode=0;
		READ_VERIFY(ParseIntraPredModeLumaCabac(cabacDecEngine,pCtx,iCode));
		iPrevIntra4x4PredMode=iCode;
		const int32_t kiPredMode=PredIntra4x4Mode(pIntraPredMode,i<<2);
		int8_t iBestMode=iPrevIntra4x4PredMode==-1 ? kiPredMode : iPrevIntra4x4PredMode+(iPrevIntra4x4PredMode>=kiPredMode);
		iFinalMode=CheckIntraNxNPredMode(&iSampleAvail[0],&iBestMode,i<<2,true);
		if(iFinalMode==GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE)){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I4x4_PRED_MODE);
		}
		for(int j=0; j<4; j++){
			layer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[(i<<2)+j]]=iFinalMode;
			pIntraPredMode[g_kuiScan8[(i<<2)+j]]=iBestMode;
			iSampleAvail[g_kuiCache30ScanIdx[(i<<2)+j]]=1;
		}
	}
	ST32(&layer->pIntraPredMode[iMbXy][0],LD32(&pIntraPredMode[1+8*4]));
	layer->pIntraPredMode[iMbXy][4]=pIntraPredMode[4+8*1];
	layer->pIntraPredMode[iMbXy][5]=pIntraPredMode[4+8*2];
	layer->pIntraPredMode[iMbXy][6]=pIntraPredMode[4+8*3];
	if(pCtx->m_sps.uiChromaFormatIdc==0)
		return ERR_NONE;
	READ_VERIFY(ParseIntraPredModeChromaCabac(cabacDecEngine,pCtx,uiNeighAvail,iCode));
	if(iCode>MAX_PRED_MODE_ID_CHROMA){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}
	layer->pChromaPredMode[iMbXy]=iCode;
	if(-1==layer->pChromaPredMode[iMbXy] || CheckIntraChromaPredMode(uiNeighAvail,&layer->pChromaPredMode[iMbXy])){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseIntra4x4Mode(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,int8_t* pIntraPredMode,SBitStringAux* pBs,SDqLayer* layer){
	int32_t iSampleAvail[5*6]={0};		// initialize as 0
	int32_t iMbXy=layer->iMbXyIndex;
	int32_t iFinalMode,i;
	uint8_t uiNeighAvail=0;
	int32_t iCode;
	MapNxNNeighToSampleNormal(pNeighAvail,iSampleAvail);
	uiNeighAvail=(iSampleAvail[6]<<2)|(iSampleAvail[0]<<1)|(iSampleAvail[1]);
	for(i=0; i<16; i++){
		int32_t iPrevIntra4x4PredMode=0;
		READ_VERIFY(ParseIntraPredModeLumaCabac(cabacDecEngine,pCtx,iCode));
		iPrevIntra4x4PredMode=iCode;
		const int32_t kiPredMode=PredIntra4x4Mode(pIntraPredMode,i);
		int8_t iBestMode;
		if(iPrevIntra4x4PredMode==-1)
			iBestMode=kiPredMode;
		else
			iBestMode=iPrevIntra4x4PredMode+(iPrevIntra4x4PredMode>=kiPredMode);
		iFinalMode=CheckIntraNxNPredMode(&iSampleAvail[0],&iBestMode,i,false);
		if(iFinalMode==GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE)){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I4x4_PRED_MODE);
		}
		layer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[i]]=iFinalMode;
		pIntraPredMode[g_kuiScan8[i]]=iBestMode;
		iSampleAvail[g_kuiCache30ScanIdx[i]]=1;
	}
	ST32(&layer->pIntraPredMode[iMbXy][0],LD32(&pIntraPredMode[1+8*4]));
	layer->pIntraPredMode[iMbXy][4]=pIntraPredMode[4+8*1];
	layer->pIntraPredMode[iMbXy][5]=pIntraPredMode[4+8*2];
	layer->pIntraPredMode[iMbXy][6]=pIntraPredMode[4+8*3];
	if(pCtx->m_sps.uiChromaFormatIdc==0)// no need parse chroma
		return ERR_NONE;
	READ_VERIFY(ParseIntraPredModeChromaCabac(cabacDecEngine,pCtx,uiNeighAvail,iCode));
	if(iCode>MAX_PRED_MODE_ID_CHROMA){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}
	layer->pChromaPredMode[iMbXy]=iCode;
	if(-1==layer->pChromaPredMode[iMbXy] || CheckIntraChromaPredMode(uiNeighAvail,&layer->pChromaPredMode[iMbXy])){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::CheckIntra16x16PredMode(uint8_t uiSampleAvail,int8_t* pMode){
	int32_t iLeftAvail=uiSampleAvail&0x04;
	int32_t bLeftTopAvail=uiSampleAvail&0x02;
	int32_t iTopAvail=uiSampleAvail&0x01;
	if((*pMode<0) || (*pMode>MAX_PRED_MODE_ID_I16x16)){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I16x16_PRED_MODE);
	}
	if(I16_PRED_DC==*pMode){
		if(iLeftAvail && iTopAvail){
			return ERR_NONE;
		}else
		if(iLeftAvail){
			*pMode=I16_PRED_DC_L;
		}else
		if(iTopAvail){
			*pMode=I16_PRED_DC_T;
		}else{
			*pMode=I16_PRED_DC_128;
		}
	}else{
		bool bModeAvail=CHECK_I16_MODE(*pMode,iLeftAvail,iTopAvail,bLeftTopAvail);
		if(0==bModeAvail){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I16x16_PRED_MODE);
		}
	}
	return ERR_NONE;
}

void Map16x16NeighToSampleNormal(const SNeighAvail* pNeighAvail,uint8_t* pSampleAvail){
	if(pNeighAvail->iLeftAvail){
		*pSampleAvail=(1<<2);
	}
	if(pNeighAvail->iLeftTopAvail){
		*pSampleAvail|=(1<<1);
	}
	if(pNeighAvail->iTopAvail){
		*pSampleAvail|=1;
	}
}

int32_t PlainH264Decoder::ParseIntra16x16Mode(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,SBitStringAux* pBs,SDqLayer* layer){
	int32_t iMbXy=layer->iMbXyIndex;
	uint8_t uiNeighAvail=0;		// 0x07=0 1 1 1,means left,top-left,top avail or not. (1: avail,0: unavail)
	int32_t iCode;
	Map16x16NeighToSampleNormal(pNeighAvail,&uiNeighAvail);

	if(CheckIntra16x16PredMode(uiNeighAvail,&layer->pIntraPredMode[iMbXy][7])){		// invalid iPredMode,must stop decoding
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I16x16_PRED_MODE);
	}
	if(pCtx->m_sps.uiChromaFormatIdc==0)
		return ERR_NONE;

	READ_VERIFY(ParseIntraPredModeChromaCabac(cabacDecEngine,pCtx,uiNeighAvail,iCode));
	if(iCode>MAX_PRED_MODE_ID_CHROMA){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}
	layer->pChromaPredMode[iMbXy]=iCode;
	if(-1==layer->pChromaPredMode[iMbXy] || CheckIntraChromaPredMode(uiNeighAvail,&layer->pChromaPredMode[iMbXy])){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}

	return ERR_NONE;
}

static const uint8_t g_kuiI16CbpTable[6]={0,16,32,15,31,47};

int32_t PlainH264Decoder::ParseCbpInfoCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,const SNeighAvail* pNeighAvail,uint32_t& uiCbp){
	int32_t iIdxA=0,iIdxB=0,pALeftMb[2],pBTopMb[2];
	uiCbp=0;
	uint32_t pCbpBit[6];
	int32_t iCtxInc;

	// Luma: bit by bit for 4 8x8 blocks in z-order
	pBTopMb[0]=pNeighAvail->iTopAvail && pNeighAvail->iTopType!=MB_TYPE_INTRA_PCM && ((pNeighAvail->iTopCbp&(1<<2))==0);
	pBTopMb[1]=pNeighAvail->iTopAvail && pNeighAvail->iTopType!=MB_TYPE_INTRA_PCM && ((pNeighAvail->iTopCbp&(1<<3))==0);
	pALeftMb[0]=pNeighAvail->iLeftAvail && pNeighAvail->iLeftType!=MB_TYPE_INTRA_PCM && ((pNeighAvail->iLeftCbp&(1<<1))==0);
	pALeftMb[1]=pNeighAvail->iLeftAvail && pNeighAvail->iLeftType!=MB_TYPE_INTRA_PCM && ((pNeighAvail->iLeftCbp&(1<<3))==0);

	// left_top 8x8 block
	iCtxInc=pALeftMb[0]+(pBTopMb[0]<<1);
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[0]));
	if(pCbpBit[0])
		uiCbp+=0x01;

	// right_top 8x8 block
	iIdxA=!pCbpBit[0];
	iCtxInc=iIdxA+(pBTopMb[1]<<1);
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[1]));
	if(pCbpBit[1])
		uiCbp+=0x02;

	// left_bottom 8x8 block
	iIdxB=!pCbpBit[0];
	iCtxInc=pALeftMb[1]+(iIdxB<<1);
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[2]));
	if(pCbpBit[2])
		uiCbp+=0x04;

	// right_bottom 8x8 block
	iIdxB=!pCbpBit[1];
	iIdxA=!pCbpBit[2];
	iCtxInc=iIdxA+(iIdxB<<1);
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[3]));
	if(pCbpBit[3])
		uiCbp+=0x08;
	if(pCtx->m_sps.uiChromaFormatIdc==0)// monochroma
		return ERR_NONE;
	// Chroma: bit by bit
	iIdxB=pNeighAvail->iTopAvail && (pNeighAvail->iTopType==MB_TYPE_INTRA_PCM || (pNeighAvail->iTopCbp>>4));
	iIdxA=pNeighAvail->iLeftAvail && (pNeighAvail->iLeftType==MB_TYPE_INTRA_PCM || (pNeighAvail->iLeftCbp>>4));
	// BitIdx=0
	iCtxInc=iIdxA+(iIdxB<<1);
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBP+CTX_NUM_CBP+iCtxInc,pCbpBit[4]));
	// BitIdx=1
	if(pCbpBit[4]){
		iIdxB=pNeighAvail->iTopAvail && (pNeighAvail->iTopType==MB_TYPE_INTRA_PCM || (pNeighAvail->iTopCbp>>4)==2);
		iIdxA=pNeighAvail->iLeftAvail && (pNeighAvail->iLeftType==MB_TYPE_INTRA_PCM || (pNeighAvail->iLeftCbp>>4)==2);
		iCtxInc=iIdxA+(iIdxB<<1);
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBP+2*CTX_NUM_CBP+iCtxInc,pCbpBit[5]));
		uiCbp+=1<<(4+pCbpBit[5]);
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseDeltaQpCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,int32_t& iQpDelta){
	uint32_t uiCode;
	SSlice* pCurrSlice=&(pCtx->m_slice);
	iQpDelta=0;
	int32_t iCtxInc=(pCurrSlice->iLastDeltaQp!=0);
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_DELTA_QP+iCtxInc,uiCode));
	if(uiCode!=0){
		READ_VERIFY(cabacDecEngine->DecodeUnaryBinCabac(NEW_CTX_OFFSET_DELTA_QP+2,1,uiCode));
		uiCode++;
		iQpDelta=(uiCode+1)>>1;
		if((uiCode&1)==0)
			iQpDelta=-iQpDelta;
	}
	pCurrSlice->iLastDeltaQp=iQpDelta;
	return ERR_NONE;
}

const uint8_t g_kuiLumaDcZigzagScan[16]={
	0,16,32,128,		// 0*16+0*64,1*16+0*64,2*16+0*64,0*16+2*64,
	48,64,80,96,		// 3*16+0*64,0*16+1*64,1*16+1*64,2*16+1*64,
	144,160,176,192,	// 1*16+2*64,2*16+2*64,3*16+2*64,0*16+3*64,
	112,208,224,240		// 3*16+1*64,1*16+3*64,2*16+3*64,3*16+3*64,
};

static inline void GetMbResProperty(int32_t* pMBproperty,int32_t* pResidualProperty,bool bCavlc){
	switch(*pResidualProperty){
		case CHROMA_AC_U:
			*pMBproperty=1;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_U;
			break;
		case CHROMA_AC_V:
			*pMBproperty=2;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_V;
			break;
		case LUMA_DC_AC_INTRA:
			*pMBproperty=0;
			*pResidualProperty=LUMA_DC_AC;
			break;
		case CHROMA_DC_U:
			*pMBproperty=1;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_U;
			break;
		case CHROMA_DC_V:
			*pMBproperty=2;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_V;
			break;
		case I16_LUMA_AC:
			*pMBproperty=0;
			break;
		case I16_LUMA_DC:
			*pMBproperty=0;
			break;
		case LUMA_DC_AC_INTER:
			*pMBproperty=3;
			*pResidualProperty=LUMA_DC_AC;
			break;
		case CHROMA_DC_U_INTER:
			*pMBproperty=4;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_U;
			break;
		case CHROMA_DC_V_INTER:
			*pMBproperty=5;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_V;
			break;
		case CHROMA_AC_U_INTER:
			*pMBproperty=4;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_U;
			break;
		case CHROMA_AC_V_INTER:
			*pMBproperty=5;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_V;
			break;
			// Reference to Table 7-2
		case LUMA_DC_AC_INTRA_8:
			*pMBproperty=6;
			*pResidualProperty=LUMA_DC_AC_8;
			break;
		case LUMA_DC_AC_INTER_8:
			*pMBproperty=7;
			*pResidualProperty=LUMA_DC_AC_8;
			break;
	}
}

ALIGNED_DECLARE(const uint16_t,g_kuiDequantCoeff[52][8],16)={
	/* 0*/{10,13,10,13,13,16,13,16},/* 1*/{11,14,11,14,14,18,14,18},
	/* 2*/{13,16,13,16,16,20,16,20},/* 3*/{14,18,14,18,18,23,18,23},
	/* 4*/{16,20,16,20,20,25,20,25},/* 5*/{18,23,18,23,23,29,23,29},
	/* 6*/{20,26,20,26,26,32,26,32},/* 7*/{22,28,22,28,28,36,28,36},
	/* 8*/{26,32,26,32,32,40,32,40},/* 9*/{28,36,28,36,36,46,36,46},
	/*10*/{32,40,32,40,40,50,40,50},/*11*/{36,46,36,46,46,58,46,58},
	/*12*/{40,52,40,52,52,64,52,64},/*13*/{44,56,44,56,56,72,56,72},
	/*14*/{52,64,52,64,64,80,64,80},/*15*/{56,72,56,72,72,92,72,92},
	/*16*/{64,80,64,80,80,100,80,100},/*17*/{72,92,72,92,92,116,92,116},
	/*18*/{80,104,80,104,104,128,104,128},/*19*/{88,112,88,112,112,144,112,144},
	/*20*/{104,128,104,128,128,160,128,160},/*21*/{112,144,112,144,144,184,144,184},
	/*22*/{128,160,128,160,160,200,160,200},/*23*/{144,184,144,184,184,232,184,232},
	/*24*/{160,208,160,208,208,256,208,256},/*25*/{176,224,176,224,224,288,224,288},
	/*26*/{208,256,208,256,256,320,256,320},/*27*/{224,288,224,288,288,368,288,368},
	/*28*/{256,320,256,320,320,400,320,400},/*29*/{288,368,288,368,368,464,368,464},
	/*30*/{320,416,320,416,416,512,416,512},/*31*/{352,448,352,448,448,576,448,576},
	/*32*/{416,512,416,512,512,640,512,640},/*33*/{448,576,448,576,576,736,576,736},
	/*34*/{512,640,512,640,640,800,640,800},/*35*/{576,736,576,736,736,928,736,928},
	/*36*/{640,832,640,832,832,1024,832,1024},/*37*/{704,896,704,896,896,1152,896,1152},
	/*38*/{832,1024,832,1024,1024,1280,1024,1280},/*39*/{896,1152,896,1152,1152,1472,1152,1472},
	/*40*/{1024,1280,1024,1280,1280,1600,1280,1600},/*41*/{1152,1472,1152,1472,1472,1856,1472,1856},
	/*42*/{1280,1664,1280,1664,1664,2048,1664,2048},/*43*/{1408,1792,1408,1792,1792,2304,1792,2304},
	/*44*/{1664,2048,1664,2048,2048,2560,2048,2560},/*45*/{1792,2304,1792,2304,2304,2944,2304,2944},
	/*46*/{2048,2560,2048,2560,2560,3200,2560,3200},/*47*/{2304,2944,2304,2944,2944,3712,2944,3712},
	/*48*/{2560,3328,2560,3328,3328,4096,3328,4096},/*49*/{2816,3584,2816,3584,3584,4608,3584,4608},
	/*50*/{3328,4096,3328,4096,4096,5120,4096,5120},/*51*/{3584,4608,3584,4608,4608,5888,4608,5888},
};

ALIGNED_DECLARE(const uint16_t,g_kuiDequantCoeff8x8[52][64],16)={
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==0
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==1
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==2
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==3
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==4
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==5
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==6
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==7
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==8
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==9
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==10
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==11
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==12
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==13
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==14
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==15
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==16
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==17
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==18
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==19
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==20
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==21
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==22
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==23
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==24
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==25
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==26
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==27
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==28
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==29
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==30
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==31
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==32
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==33
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==34
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==35
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==36
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==37
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==38
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==39
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==40
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==41
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==42
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==43
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==44
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==45
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==46
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==47
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==48
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==49
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==50
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==51
};

#define IDX_UNUSED -1

static const int16_t g_kMaxPos[]={IDX_UNUSED,15,14,15,3,14,63,3,3,14,14};
static const int16_t g_kMaxC2[]={IDX_UNUSED,4,4,4,3,4,4,3,3,4,4};
static const int16_t g_kBlockCat2CtxOffsetCBF[]={IDX_UNUSED,0,4,8,12,16,0,12,12,16,16};
static const int16_t g_kBlockCat2CtxOffsetMap[]={IDX_UNUSED,0,15,29,44,47,0,44,44,47,47};
static const int16_t g_kBlockCat2CtxOffsetLast[]={IDX_UNUSED,0,15,29,44,47,0,44,44,47,47};
static const int16_t g_kBlockCat2CtxOffsetOne[]={IDX_UNUSED,0,10,20,30,39,0,30,30,39,39};
static const int16_t g_kBlockCat2CtxOffsetAbs[]={IDX_UNUSED,0,10,20,30,39,0,30,30,39,39};

const uint8_t g_kTopBlkInsideMb[24]={		// for index with z-order 0~23
	// 0 1 | 4 5 luma 8*8 block pNonZeroCount[16+8]
	0,0,1,1,		// 2 3 | 6 7 0 | 1 0 1 2 3
	0,0,1,1,		// --------------- --------- 4 5 6 7
	1,1,1,1,		// 8 9 | 12 13 2 | 3 8 9 10 11
	1,1,1,1,		// 10 11 | 14 15-----------------------------> 12 13 14 15
	0,0,1,1,		// ---------------- chroma 8*8 block 16 17 18 19
	0,0,1,1		// 16 17 | 20 21 0 1 20 21 22 23
	// 18 19 | 22 23
};

const uint8_t g_kLeftBlkInsideMb[24]={		// for index with z-order 0~23
	// 0 1 | 4 5 luma 8*8 block pNonZeroCount[16+8]
	0,1,0,1,		// 2 3 | 6 7 0 | 1 0 1 2 3
	1,1,1,1,		// --------------- --------- 4 5 6 7
	0,1,0,1,		// 8 9 | 12 13 2 | 3 8 9 10 11
	1,1,1,1,		// 10 11 | 14 15-----------------------------> 12 13 14 15
	0,1,0,1,		// ---------------- chroma 8*8 block 16 17 18 19
	0,1,0,1		// 16 17 | 20 21 0 1 20 21 22 23
	// 18 19 | 22 23
};

const uint8_t g_kCacheNzcScanIdx[4*4+4+4+3]={
	// Luma
	9,10,17,18,			// 1+1*8,2+1*8,1+2*8,2+2*8,
	11,12,19,20,		// 3+1*8,4+1*8,3+2*8,4+2*8,
	25,26,33,34,		// 1+3*8,2+3*8,1+4*8,2+4*8,
	27,28,35,36,		// 3+3*8,4+3*8,3+4*8,4+4*8,
	// Cb
	14,15,				// 6+1*8,7+1*8,
	22,23,				// 6+2*8,7+2*8,

	// Cr
	38,39,				// 6+4*8,7+4*8,
	46,47,				// 6+5*8,7+5*8,
// Luma DC
	41,					// 1+5*8
// Chroma DC 
	42,43				// 2+5*8,3+5*8,
};


static const uint8_t g_kuiZigzagScan[16]={		// 4*4block residual zig-zag scan order
	0,1,4,8,
	5,2,3,6,
	9,12,13,10,
	7,11,14,15,
};

static const uint8_t g_kuiZigzagScan8x8[64]={		// 8x8 block residual zig-zag scan order
	0,1,8,16,9,2,3,10,
	17,24,32,25,18,11,4,5,
	12,19,26,33,40,48,41,34,
	27,20,13,6,7,14,21,28,
	35,42,49,56,57,50,43,36,
	29,22,15,23,30,37,44,51,
	58,59,52,45,38,31,39,46,
	53,60,61,54,47,55,62,63,
};

static const uint8_t g_kuiIdx2CtxSignificantCoeffFlag8x8[64]={		// Table 9-43,Page 289
	0,1,2,3,4,5,5,4,
	4,3,3,4,4,4,5,5,
	4,4,4,4,3,3,6,7,
	7,7,8,9,10,9,8,7,
	7,6,11,12,13,11,6,7,
	8,9,14,10,9,8,6,11,
	12,13,11,6,9,14,10,9,
	11,12,13,11,14,10,12,14,
};

static const uint8_t g_kuiIdx2CtxLastSignificantCoeffFlag8x8[64]={		// Table 9-43,Page 289
	0,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	3,3,3,3,3,3,3,3,
	4,4,4,4,4,4,4,4,
	5,5,5,5,6,6,6,6,
	7,7,7,7,8,8,8,8,
};


int32_t PlainH264Decoder::ParseCbfInfoCabac(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail,uint8_t* pNzcCache,int32_t iZIndex,int32_t iResProperty,SDecoderContext* pCtx,uint32_t& uiCbfBit){
	int8_t nA,nB;
	int32_t iCurrBlkXy=pCtx->m_layer.iMbXyIndex;
	int32_t iTopBlkXy=iCurrBlkXy-pCtx->m_layer.iMbWidth;	// default value: MB neighboring
	int32_t iLeftBlkXy=iCurrBlkXy-1;							// default value: MB neighboring
	uint16_t* pCbfDc=pCtx->m_layer.pCbfDc;
	uint32_t* pMbType=pCtx->m_layer.pDec->pMbType;
	int32_t iCtxInc;
	uiCbfBit=0;
	nA=nB=(int8_t)!!IS_INTRA(pMbType[iCurrBlkXy]);

	if(iResProperty==I16_LUMA_DC || iResProperty==CHROMA_DC_U || iResProperty==CHROMA_DC_V){		// DC
		if(pNeighAvail->iTopAvail)
			nB=(pMbType[iTopBlkXy]==MB_TYPE_INTRA_PCM) || ((pCbfDc[iTopBlkXy]>>iResProperty)&1);
		if(pNeighAvail->iLeftAvail)
			nA=(pMbType[iLeftBlkXy]==MB_TYPE_INTRA_PCM) || ((pCbfDc[iLeftBlkXy]>>iResProperty)&1);
		iCtxInc=nA+(nB<<1);
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBF+g_kBlockCat2CtxOffsetCBF[iResProperty]+iCtxInc,uiCbfBit));
		if(uiCbfBit)
			pCbfDc[iCurrBlkXy]|=(1<<iResProperty);
	}else{		// AC
// for 4x4 blk,make sure blk-idx is correct
		if(pNzcCache[g_kCacheNzcScanIdx[iZIndex]-8]!=0xff){		// top blk available
			if(g_kTopBlkInsideMb[iZIndex])
				iTopBlkXy=iCurrBlkXy;
			nB=pNzcCache[g_kCacheNzcScanIdx[iZIndex]-8] || pMbType[iTopBlkXy]==MB_TYPE_INTRA_PCM;
		}
		if(pNzcCache[g_kCacheNzcScanIdx[iZIndex]-1]!=0xff){		// left blk available
			if(g_kLeftBlkInsideMb[iZIndex])
				iLeftBlkXy=iCurrBlkXy;
			nA=pNzcCache[g_kCacheNzcScanIdx[iZIndex]-1] || pMbType[iLeftBlkXy]==MB_TYPE_INTRA_PCM;
		}

		iCtxInc=nA+(nB<<1);
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(NEW_CTX_OFFSET_CBF+g_kBlockCat2CtxOffsetCBF[iResProperty]+iCtxInc,uiCbfBit));
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseSignificantMapCabac(SCabacDecEngine* cabacDecEngine,int32_t* pSignificantMap,int32_t iResProperty,SDecoderContext* pCtx,uint32_t& uiCoeffNum){
	uint32_t uiCode;
	uint32_t pMapCtx=(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_MAP_8x8 : NEW_CTX_OFFSET_MAP)+g_kBlockCat2CtxOffsetMap[iResProperty];
	uint32_t pLastCtx=(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_LAST_8x8 : NEW_CTX_OFFSET_LAST)+g_kBlockCat2CtxOffsetLast[iResProperty];
	int32_t i;
	uiCoeffNum=0;
	int32_t i0=0;
	int32_t i1=g_kMaxPos[iResProperty];
	int32_t iCtx;
	for(i=i0; i<i1;++i){
		iCtx=(iResProperty==LUMA_DC_AC_8 ? g_kuiIdx2CtxSignificantCoeffFlag8x8[i] : i);
		// read significant
		READ_VERIFY(cabacDecEngine->DecodeBinCabac(pMapCtx+iCtx,uiCode));
		if(uiCode){
			*(pSignificantMap++)=1;
			++uiCoeffNum;
			// read last significant
			iCtx=(iResProperty==LUMA_DC_AC_8 ? g_kuiIdx2CtxLastSignificantCoeffFlag8x8[i] : i);
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(pLastCtx+iCtx,uiCode));
			if(uiCode){
				memset(pSignificantMap,0,(i1-i)*sizeof(int32_t));
				return ERR_NONE;
			}
		}else
			*(pSignificantMap++)=0;
	}
	// deal with last pSignificantMap if no data
	// if(i < i1+1)
	{
		*pSignificantMap=1;
		++uiCoeffNum;
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseSignificantCoeffCabac(SCabacDecEngine* cabacDecEngine,int32_t* pSignificant,int32_t iResProperty,SDecoderContext* pCtx){
	uint32_t uiCode;
	uint32_t pOneCtx=(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_ONE_8x8 : NEW_CTX_OFFSET_ONE)+g_kBlockCat2CtxOffsetOne[iResProperty];
	uint32_t pAbsCtx=(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_ABS_8x8 : NEW_CTX_OFFSET_ABS)+g_kBlockCat2CtxOffsetAbs[iResProperty];
	const int16_t iMaxType=g_kMaxC2[iResProperty];
	int32_t i=g_kMaxPos[iResProperty];
	int32_t* pCoff=pSignificant+i;
	int32_t c1=1;
	int32_t c2=0;
	for(; i>=0;--i){
		if(*pCoff!=0){
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(pOneCtx+c1,uiCode));
			*pCoff+=uiCode;
			if(*pCoff==2){
				READ_VERIFY(cabacDecEngine->DecodeUEGLevelCabac(pAbsCtx+c2,uiCode));
				*pCoff+=uiCode;
				++c2;
				c2=WELS_MIN(c2,iMaxType);
				c1=0;
			}else
			if(c1){
				++c1;
				c1=WELS_MIN(c1,4);
			}
			READ_VERIFY(cabacDecEngine->DecodeBypassCabac(uiCode));
			if(uiCode)
				*pCoff=-*pCoff;
		}
		pCoff--;
	}
	return ERR_NONE;
}

void PlainH264Decoder::LumaDcDequantIdct(int16_t* pBlock,int32_t iQp,SDecoderContext* pCtx){
	const int32_t kiQMul=g_kuiDequantCoeff[iQp][0]<<4;
#define STRIDE 16
	int32_t i;
	int32_t iTemp[16];
	int16_t* pBlk=pBlock;
	static const int32_t kiXOffset[4]={0,STRIDE,STRIDE<<2,5*STRIDE};
	static const int32_t kiYOffset[4]={0,STRIDE<<1,STRIDE<<3,10*STRIDE};
	for(i=0; i<4; i++){
		const int32_t kiOffset=kiYOffset[i];
		const int32_t kiX1=kiOffset+kiXOffset[2];
		const int32_t kiX2=STRIDE+kiOffset;
		const int32_t kiX3=kiOffset+kiXOffset[3];
		const int32_t kiI4=i<<2;
		const int32_t kiZ0=pBlk[kiOffset]+pBlk[kiX1];
		const int32_t kiZ1=pBlk[kiOffset]-pBlk[kiX1];
		const int32_t kiZ2=pBlk[kiX2]-pBlk[kiX3];
		const int32_t kiZ3=pBlk[kiX2]+pBlk[kiX3];
		iTemp[kiI4]=kiZ0+kiZ3;
		iTemp[1+kiI4]=kiZ1+kiZ2;
		iTemp[2+kiI4]=kiZ1-kiZ2;
		iTemp[3+kiI4]=kiZ0-kiZ3;
	}
	for(i=0; i<4; i++){
		const int32_t kiOffset=kiXOffset[i];
		const int32_t kiI4=4+i;
		const int32_t kiZ0=iTemp[i]+iTemp[4+kiI4];
		const int32_t kiZ1=iTemp[i]-iTemp[4+kiI4];
		const int32_t kiZ2=iTemp[kiI4]-iTemp[8+kiI4];
		const int32_t kiZ3=iTemp[kiI4]+iTemp[8+kiI4];
		pBlk[kiOffset]=((kiZ0+kiZ3)*kiQMul+(1<<5))>>6;		// FIXME think about merging this into decode_resdual
		pBlk[kiYOffset[1]+kiOffset]=((kiZ1+kiZ2)*kiQMul+(1<<5))>>6;
		pBlk[kiYOffset[2]+kiOffset]=((kiZ1-kiZ2)*kiQMul+(1<<5))>>6;
		pBlk[kiYOffset[3]+kiOffset]=((kiZ0-kiZ3)*kiQMul+(1<<5))>>6;
	}
#undef STRIDE
}

void ChromaDcIdct(int16_t* pBlock){
	int32_t iStride=32;
	int32_t iXStride=16;
	int32_t iStride1=iXStride+iStride;
	int16_t* pBlk=pBlock;
	int32_t iA,iB,iC,iD,iE;
	iA=pBlk[0];
	iB=pBlk[iXStride];
	iC=pBlk[iStride];
	iD=pBlk[iStride1];
	iE=iA-iB;
	iA+=iB;
	iB=iC-iD;
	iC+=iD;
	pBlk[0]=(iA+iC);
	pBlk[iXStride]=(iE+iB);
	pBlk[iStride]=(iA-iC);
	pBlk[iStride1]=(iE-iB);
}

int32_t PlainH264Decoder::ParseResidualBlockCabac(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCountCache,SBitStringAux* pBsAux,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* pScanTable,int32_t iResProperty,short* sTCoeff,uint8_t uiQp,SDecoderContext* pCtx){
	int32_t iCurNzCacheIdx;
	uint32_t uiTotalCoeffNum=0;
	uint32_t uiCbpBit;
	int32_t pSignificantMap[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int32_t iMbResProperty=0;
	GetMbResProperty(&iMbResProperty,&iResProperty,false);
	const uint16_t* pDeQuantMul=g_kuiDequantCoeff[uiQp];
	READ_VERIFY(ParseCbfInfoCabac(cabacDecEngine,pNeighAvail,pNonZeroCountCache,iIndex,iResProperty,pCtx,uiCbpBit));
	if(uiCbpBit){		// has coeff
		READ_VERIFY(ParseSignificantMapCabac(cabacDecEngine,pSignificantMap,iResProperty,pCtx,uiTotalCoeffNum));
		READ_VERIFY(ParseSignificantCoeffCabac(cabacDecEngine,pSignificantMap,iResProperty,pCtx));
	}
	iCurNzCacheIdx=g_kCacheNzcScanIdx[iIndex];
	pNonZeroCountCache[iCurNzCacheIdx]=(uint8_t)uiTotalCoeffNum;
	if(uiTotalCoeffNum==0){
		return ERR_NONE;
	}
	int32_t j=0;
	if(iResProperty==I16_LUMA_DC){
		do{
			sTCoeff[pScanTable[j]]=pSignificantMap[j];
			++j;
		} while(j<16);
		LumaDcDequantIdct(sTCoeff,uiQp,pCtx);
	}else
	if(iResProperty==CHROMA_DC_U || iResProperty==CHROMA_DC_V){
		do{
			sTCoeff[pScanTable[j]]=pSignificantMap[j];
			++j;
		} while(j<4);
// iHadamard2x2
		ChromaDcIdct(sTCoeff);
// scaling
		for(j=0; j<4;++j){
			sTCoeff[pScanTable[j]]=(int16_t)((int64_t)sTCoeff[pScanTable[j]]*(int64_t)pDeQuantMul[0]>>1);
		}
	}else{											// luma ac,chroma ac
		do{
			if(pSignificantMap[j]!=0){
				sTCoeff[pScanTable[j]]=pSignificantMap[j]*pDeQuantMul[pScanTable[j]&0x07];
			}
			++j;
		} while(j<16);
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParseResidualBlockCabac8x8(SCabacDecEngine* cabacDecEngine,const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCountCache,SBitStringAux* pBsAux,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* pScanTable,int32_t iResProperty,short* sTCoeff,uint8_t uiQp,SDecoderContext* pCtx){
	uint32_t uiTotalCoeffNum=0;
	uint32_t uiCbpBit;
	int32_t pSignificantMap[64]={0};
	int32_t iMbResProperty=0;
	GetMbResProperty(&iMbResProperty,&iResProperty,false);
	const uint16_t* pDeQuantMul=g_kuiDequantCoeff8x8[uiQp];
	uiCbpBit=1;										// for 8x8,MaxNumCoeff==64 && uiCbpBit==1
	if(uiCbpBit){									// has coeff
		READ_VERIFY(ParseSignificantMapCabac(cabacDecEngine,pSignificantMap,iResProperty,pCtx,uiTotalCoeffNum));
		READ_VERIFY(ParseSignificantCoeffCabac(cabacDecEngine,pSignificantMap,iResProperty,pCtx));
	}
	pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex]]=pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex+1]]=pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex+2]]=pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex+3]]=(uint8_t)uiTotalCoeffNum;
	if(uiTotalCoeffNum==0){
		return ERR_NONE;
	}
	int32_t j=0,i;
	if(iResProperty==LUMA_DC_AC_8){
		do{
			if(pSignificantMap[j]!=0){
				i=pScanTable[j];
				sTCoeff[i]=uiQp>=36 ? ((pSignificantMap[j]*pDeQuantMul[i])*(1<<(uiQp/6-6))) : ((pSignificantMap[j]*pDeQuantMul[i]+(1<<(5-uiQp/6)))>>(6-uiQp/6));
			}
			++j;
		} while(j<64);
	}
	return ERR_NONE;
}

const uint8_t g_kuiChromaDcScan[4]={
	0,16,32,48
};

int32_t PlainH264Decoder::DecodeMbCabacPSliceBaseMode0(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SBitStringAux* pBsAux,const SNeighAvail* pNeighAvail){
	SDqLayer* layer=&pCtx->m_layer;
	SSlice* slice=&pCtx->m_slice;

	int32_t iScanIdxStart=0;
	int32_t iScanIdxEnd=15;
	int32_t iMbXy=layer->iMbXyIndex;
	int32_t iMbResProperty;
	int32_t i;
	uint32_t uiCbp=0,uiCbpLuma=0,uiCbpChroma=0;
	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);
	uint32_t uiMbType=ParseMBTypePSliceCabac(cabacDecEngine,pNeighAvail);
// uiMbType=4 is not allowded.
	if(uiMbType<4){										// Inter mode
		int16_t pMotionVector[LIST_A][30][MV_A];
		int16_t pMvdCache[LIST_A][30][MV_A];
		int8_t pRefIndex[LIST_A][30];
		layer->pDec->pMbType[iMbXy]=g_ksInterPMbTypeInfo[uiMbType].iType;
		FillCacheInterCabac(pCtx,pNeighAvail,pNonZeroCount,pMotionVector,pMvdCache,pRefIndex,layer);
		READ_VERIFY(ParseInterPMotionInfoCabac(cabacDecEngine,pCtx,pNeighAvail,pNonZeroCount,pMotionVector,pMvdCache,pRefIndex));
	}else{												// Intra mode
		uiMbType-=5;
		if(uiMbType>25)
			FATAL("uiMbType %d out of range",uiMbType);
		if(!pCtx->m_sps.uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17 && uiMbType<=24)))
			FATAL("uiMbType %d invalid",uiMbType);
		if(25==uiMbType){								// I_PCM
			FATAL("IPCM SUPPORT REMOVED");
		}else{											// normal Intra mode
			if(0==uiMbType){							// Intra4x4
				ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
				layer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
				if(pCtx->m_pps.bTransform8x8ModeFlag){
					READ_VERIFY(ParseTransformSize8x8FlagCabac(cabacDecEngine,pCtx,pNeighAvail,pCtx->m_layer.pTransformSize8x8Flag[iMbXy]));
				}
				if(pCtx->m_layer.pTransformSize8x8Flag[iMbXy]){
					uiMbType=layer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
					FillCacheConstrain0IntraNxN(pNeighAvail,pNonZeroCount,pIntraPredMode,layer);
					READ_VERIFY(ParseIntra8x8Mode(cabacDecEngine,pCtx,pNeighAvail,pIntraPredMode,pBsAux,layer));
				}else{
					FillCacheConstrain0IntraNxN(pNeighAvail,pNonZeroCount,pIntraPredMode,layer);
					READ_VERIFY(ParseIntra4x4Mode(cabacDecEngine,pCtx,pNeighAvail,pIntraPredMode,pBsAux,layer));
				}
			}else{		// Intra16x16
				layer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
				layer->pTransformSize8x8Flag[iMbXy]=false;
				layer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
				layer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
				layer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
				uiCbpChroma=pCtx->m_sps.uiChromaFormatIdc ? layer->pCbp[iMbXy]>>4 : 0;
				uiCbpLuma=layer->pCbp[iMbXy]&15;
				FillCacheNonZeroCount(pNeighAvail,pNonZeroCount,layer);
				READ_VERIFY(ParseIntra16x16Mode(cabacDecEngine,pCtx,pNeighAvail,pBsAux,layer));
			}
		}
	}
	ST32(&layer->pNzc[iMbXy][0],0);
	ST32(&layer->pNzc[iMbXy][4],0);
	ST32(&layer->pNzc[iMbXy][8],0);
	ST32(&layer->pNzc[iMbXy][12],0);
	ST32(&layer->pNzc[iMbXy][16],0);
	ST32(&layer->pNzc[iMbXy][20],0);
	if(MB_TYPE_INTRA16x16!=layer->pDec->pMbType[iMbXy]){
		READ_VERIFY(ParseCbpInfoCabac(cabacDecEngine,pCtx,pNeighAvail,uiCbp));
		layer->pCbp[iMbXy]=uiCbp;
		slice->iLastDeltaQp=uiCbp==0 ? 0 : slice->iLastDeltaQp;
		uiCbpChroma=pCtx->m_sps.uiChromaFormatIdc ? layer->pCbp[iMbXy]>>4 : 0;
		uiCbpLuma=layer->pCbp[iMbXy]&15;
	}
	if(layer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==layer->pDec->pMbType[iMbXy]){
		if(MB_TYPE_INTRA16x16!=layer->pDec->pMbType[iMbXy]){
// Need modification when B picutre add in
			bool bNeedParseTransformSize8x8Flag=(((layer->pDec->pMbType[iMbXy]>=MB_TYPE_16x16 && layer->pDec->pMbType[iMbXy]<=MB_TYPE_8x16) || layer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]) && (layer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA8x8) && (layer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA4x4) && ((layer->pCbp[iMbXy]&0x0F)>0) && (pCtx->m_pps.bTransform8x8ModeFlag));
			if(bNeedParseTransformSize8x8Flag){
				READ_VERIFY(ParseTransformSize8x8FlagCabac(cabacDecEngine,pCtx,pNeighAvail,pCtx->m_layer.pTransformSize8x8Flag[iMbXy]));		// transform_size_8x8_flag
			}
		}
		memset(layer->pScaledTCoeff[iMbXy],0,384*sizeof(layer->pScaledTCoeff[iMbXy][0]));
		int32_t iQpDelta,iId8x8,iId4x4;
		READ_VERIFY(ParseDeltaQpCabac(cabacDecEngine,pCtx,iQpDelta));
		if(iQpDelta>25 || iQpDelta<-26){								// out of iQpDelta range
			FATAL("iQpDelta %d out of range",iQpDelta);
		}
		layer->pLumaQp[iMbXy]=(slice->iLastMbQp+iQpDelta+52)%52;	// update last_mb_qp
		slice->iLastMbQp=layer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			layer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(slice->iLastMbQp+pCtx->m_pps.iChromaQpIndexOffset[i],0,51)];
		}
		if(MB_TYPE_INTRA16x16==layer->pDec->pMbType[iMbXy]){
// step1: Luma DC
			READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,pNeighAvail,pNonZeroCount,pBsAux,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,layer->pScaledTCoeff[iMbXy],layer->pLumaQp[iMbXy],pCtx));
// step2: Luma AC
			if(uiCbpLuma){
				for(i=0; i<16; i++){
					READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,pNeighAvail,pNonZeroCount,pBsAux,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,layer->pScaledTCoeff[iMbXy]+(i<<4),layer->pLumaQp[iMbXy],pCtx));
				}
				ST32(&layer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&layer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&layer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&layer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				ST32(&layer->pNzc[iMbXy][0],0);
				ST32(&layer->pNzc[iMbXy][4],0);
				ST32(&layer->pNzc[iMbXy][8],0);
				ST32(&layer->pNzc[iMbXy][12],0);
			}
		}else{															// non-MB_TYPE_INTRA16x16
			if(pCtx->m_layer.pTransformSize8x8Flag[iMbXy]){
// Transform 8x8 support for CABAC
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						READ_VERIFY(ParseResidualBlockCabac8x8(cabacDecEngine,pNeighAvail,pNonZeroCount,pBsAux,(iId8x8<<2),iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,IS_INTRA(layer->pDec->pMbType[iMbXy]) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8,layer->pScaledTCoeff[iMbXy]+(iId8x8<<6),layer->pLumaQp[iMbXy],pCtx));
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&layer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&layer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&layer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&layer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				iMbResProperty=(IS_INTRA(layer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						int32_t iIdx=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
// Luma (DC and AC decoding together)
							READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,pNeighAvail,pNonZeroCount,pBsAux,iIdx,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,iMbResProperty,layer->pScaledTCoeff[iMbXy]+(iIdx<<4),layer->pLumaQp[iMbXy],pCtx));
							iIdx++;
						}
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&layer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&layer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&layer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&layer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}
		}
// chroma
// step1: DC
		if(1==uiCbpChroma || 2==uiCbpChroma){
			for(i=0; i<2; i++){
				if(IS_INTRA(layer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				else
					iMbResProperty=i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;
				READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,pNeighAvail,pNonZeroCount,pBsAux,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,layer->pScaledTCoeff[iMbXy]+256+(i<<6),layer->pChromaQp[iMbXy][i],pCtx));
			}
		}
// step2: AC
		if(2==uiCbpChroma){
			for(i=0; i<2; i++){
				if(IS_INTRA(layer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				else
					iMbResProperty=i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;
				int32_t index=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,pNeighAvail,pNonZeroCount,pBsAux,index,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,layer->pScaledTCoeff[iMbXy]+(index<<4),layer->pChromaQp[iMbXy][i],pCtx));
					index++;
				}
			}
			ST16(&layer->pNzc[iMbXy][16],LD16(&pNonZeroCount[6+8*1]));
			ST16(&layer->pNzc[iMbXy][20],LD16(&pNonZeroCount[6+8*2]));
			ST16(&layer->pNzc[iMbXy][18],LD16(&pNonZeroCount[6+8*4]));
			ST16(&layer->pNzc[iMbXy][22],LD16(&pNonZeroCount[6+8*5]));
		}else{
			ST32(&layer->pNzc[iMbXy][16],0);
			ST32(&layer->pNzc[iMbXy][20],0);
		}
	}else{
		layer->pLumaQp[iMbXy]=slice->iLastMbQp;
		for(i=0; i<2; i++){
			layer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(layer->pLumaQp[iMbXy]+pCtx->m_pps.iChromaQpIndexOffset[i],0,51)];
		}
	}
	return ERR_NONE;
}

void PlainH264Decoder::DecodeMbCabacPSlice(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SBitStringAux* pBsAux){
	SDqLayer* layer=&pCtx->m_layer;
	uint32_t uiCode;
	int32_t iMbXy=layer->iMbXyIndex;
	int32_t i;
	SNeighAvail uiNeighAvail;
	layer->pCbp[iMbXy]=0;
	layer->pCbfDc[iMbXy]=0;
	layer->pChromaPredMode[iMbXy]=C_PRED_DC;
	layer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	layer->pTransformSize8x8Flag[iMbXy]=false;
	GetNeighborAvailMbType(&uiNeighAvail,layer);
	ParseSkipFlagCabac(cabacDecEngine,&uiNeighAvail,uiCode);
	if(uiCode){
		int16_t pMv[2]={0};
		layer->pDec->pMbType[iMbXy]=MB_TYPE_SKIP;
		ST32(&layer->pNzc[iMbXy][0],0);
		ST32(&layer->pNzc[iMbXy][4],0);
		ST32(&layer->pNzc[iMbXy][8],0);
		ST32(&layer->pNzc[iMbXy][12],0);
		ST32(&layer->pNzc[iMbXy][16],0);
		ST32(&layer->pNzc[iMbXy][20],0);
		memset(layer->pDec->pRefIndex[0][iMbXy],0,sizeof(int8_t)*16);
		PredPSkipMvFromNeighbor(layer,pMv);
		for(i=0;i<16;i++){
			ST32(layer->pDec->pMv[0][iMbXy][i],*(uint32_t*)pMv);
			ST32(layer->pMvd[0][iMbXy][i],0);
		}
		layer->pLumaQp[iMbXy]=pCtx->m_slice.iLastMbQp;
		for(i=0;i<2;i++){
			layer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(layer->pLumaQp[iMbXy]+pCtx->m_pps.iChromaQpIndexOffset[i],0,51)];
		}
		pCtx->m_slice.iLastDeltaQp=0;
	}else{
		DecodeMbCabacPSliceBaseMode0(cabacDecEngine,pCtx,pBsAux,&uiNeighAvail);
	}
}

int32_t PlainH264Decoder::ParseMBTypeISliceCabac(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SNeighAvail* pNeighAvail,uint32_t& uiBinVal){
/*
	uint32_t uiCode3,uiCode6,uiCode7,uiCode8,uiCode9,uiCode10;
	cabacDecEngine->DecodeBinCabac(3,uiCode3);
	cabacDecEngine->DecodeTerminateCabac();
	cabacDecEngine->DecodeBinCabac(6,uiCode6);
	cabacDecEngine->DecodeBinCabac(7,uiCode7);
	cabacDecEngine->DecodeBinCabac(8,uiCode8);
	cabacDecEngine->DecodeBinCabac(9,uiCode9);
	cabacDecEngine->DecodeBinCabac(10,uiCode10);
	uprintf("tst %d,%d,%d,%d,%d,%d\n",uiCode3,uiCode6,uiCode7,uiCode8,uiCode9,uiCode10);
*/
	uiBinVal=0;
	int32_t iIdxA=(pNeighAvail->iLeftAvail) && (pNeighAvail->iLeftType!=MB_TYPE_INTRA4x4 && pNeighAvail->iLeftType!=MB_TYPE_INTRA8x8);
	int32_t iIdxB=(pNeighAvail->iTopAvail) && (pNeighAvail->iTopType!=MB_TYPE_INTRA4x4  && pNeighAvail->iTopType!=MB_TYPE_INTRA8x8);
	int32_t iCtxInc=iIdxA+iIdxB;
	uint32_t uiCode;
	//READ_VERIFY(cabacDecEngine->DecodeBinCabac(pBinCtx+iCtxInc,uiCode));
	READ_VERIFY(cabacDecEngine->DecodeBinCabac(iCtxInc+3,uiCode));
	uiBinVal=uiCode;
	if(uiBinVal!=0){			// I16x16
		if(cabacDecEngine->DecodeTerminateCabac())
			uiBinVal=25;		// I_PCM
		else{
			//READ_VERIFY(cabacDecEngine->DecodeBinCabac(pBinCtx+3,uiCode));
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(6,uiCode));
			uiBinVal=1+uiCode*12;
			// decoding of uiCbp:0,1,2
			//READ_VERIFY(cabacDecEngine->DecodeBinCabac(pBinCtx+4,uiCode));
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(7,uiCode));
			if(uiCode!=0){
				//READ_VERIFY(cabacDecEngine->DecodeBinCabac(pBinCtx+5,uiCode));
				READ_VERIFY(cabacDecEngine->DecodeBinCabac(8,uiCode));
				uiBinVal+=4;
				if(uiCode!=0)
					uiBinVal+=4;
			}
			// decoding of I pred-mode: 0,1,2,3
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(9,uiCode));
			//READ_VERIFY(cabacDecEngine->DecodeBinCabac(pBinCtx+6,uiCode));
			uiBinVal+=(uiCode<<1);
			READ_VERIFY(cabacDecEngine->DecodeBinCabac(10,uiCode));
			//READ_VERIFY(cabacDecEngine->DecodeBinCabac(pBinCtx+7,uiCode));
			uiBinVal+=uiCode;
		}
	}
	// I4x4
	return ERR_NONE;
}

int32_t PlainH264Decoder::DecodeMbCabacISlice(SCabacDecEngine* cabacDecEngine,SDecoderContext* pCtx,SBitStringAux* pBsAux){
	SDqLayer* layer=&pCtx->m_layer;
	SSlice* slice=&pCtx->m_slice;
	SNeighAvail sNeighAvail;
	int32_t iScanIdxStart=0;
	int32_t iScanIdxEnd=15;
	int32_t iMbXy=layer->iMbXyIndex;
	int32_t i;
	uint32_t uiMbType=0,uiCbp=0,uiCbpLuma=0,uiCbpChroma=0;
	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);
	layer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	layer->pTransformSize8x8Flag[iMbXy]=false;
	GetNeighborAvailMbType(&sNeighAvail,layer);
	READ_VERIFY(ParseMBTypeISliceCabac(cabacDecEngine,pCtx,&sNeighAvail,uiMbType));
	if(uiMbType>25){
		FATAL("uiMbType %d out of range",uiMbType);
	}else
	if(!pCtx->m_sps.uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17 && uiMbType<=24))){
		FATAL("uiMbType %d invalid",uiMbType);
	}else
	if(25==uiMbType){		// I_PCM
		FATAL("IPCM SUPPORT REMOVED");
	}else
	if(0==uiMbType){		// I4x4
		ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
		layer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
		if(pCtx->m_pps.bTransform8x8ModeFlag){
			// Transform 8x8 cabac will be added soon
			READ_VERIFY(ParseTransformSize8x8FlagCabac(cabacDecEngine,pCtx,&sNeighAvail,pCtx->m_layer.pTransformSize8x8Flag[iMbXy]));
		}
		if(pCtx->m_layer.pTransformSize8x8Flag[iMbXy]){
			uiMbType=layer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
			FillCacheConstrain0IntraNxN(&sNeighAvail,pNonZeroCount,pIntraPredMode,layer);
			READ_VERIFY(ParseIntra8x8Mode(cabacDecEngine,pCtx,&sNeighAvail,pIntraPredMode,pBsAux,layer));
		}else{
			FillCacheConstrain0IntraNxN(&sNeighAvail,pNonZeroCount,pIntraPredMode,layer);
			READ_VERIFY(ParseIntra4x4Mode(cabacDecEngine,pCtx,&sNeighAvail,pIntraPredMode,pBsAux,layer));
		}
		// get uiCbp for I4x4
		READ_VERIFY(ParseCbpInfoCabac(cabacDecEngine,pCtx,&sNeighAvail,uiCbp));
		layer->pCbp[iMbXy]=uiCbp;
		slice->iLastDeltaQp=uiCbp==0 ? 0 : slice->iLastDeltaQp;
		uiCbpChroma=pCtx->m_sps.uiChromaFormatIdc ? uiCbp>>4 : 0;
		uiCbpLuma=uiCbp&15;
	}else{		// I16x16;
		layer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
		layer->pTransformSize8x8Flag[iMbXy]=false;
		layer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
		layer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
		layer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
		uiCbpChroma=pCtx->m_sps.uiChromaFormatIdc ? layer->pCbp[iMbXy]>>4 : 0;
		uiCbpLuma=layer->pCbp[iMbXy]&15;
		FillCacheNonZeroCount(&sNeighAvail,pNonZeroCount,layer);
		READ_VERIFY(ParseIntra16x16Mode(cabacDecEngine,pCtx,&sNeighAvail,pBsAux,layer));
	}
	ST32(&layer->pNzc[iMbXy][0],0);
	ST32(&layer->pNzc[iMbXy][4],0);
	ST32(&layer->pNzc[iMbXy][8],0);
	ST32(&layer->pNzc[iMbXy][12],0);
	ST32(&layer->pNzc[iMbXy][16],0);
	ST32(&layer->pNzc[iMbXy][20],0);
	layer->pCbfDc[iMbXy]=0;
	if(layer->pCbp[iMbXy]==0 && IS_INTRANxN(layer->pDec->pMbType[iMbXy])){
		layer->pLumaQp[iMbXy]=slice->iLastMbQp;
		for(i=0; i<2; i++){
			layer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3((layer->pLumaQp[iMbXy]+pCtx->m_pps.iChromaQpIndexOffset[i]),0,51)];
		}
	}
	if(layer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==layer->pDec->pMbType[iMbXy]){
		memset(layer->pScaledTCoeff[iMbXy],0,384*sizeof(layer->pScaledTCoeff[iMbXy][0]));
		int32_t iQpDelta,iId8x8,iId4x4;
		READ_VERIFY(ParseDeltaQpCabac(cabacDecEngine,pCtx,iQpDelta));
		if(iQpDelta>25 || iQpDelta<-26){// out of iQpDelta range
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_QP);
		}
		layer->pLumaQp[iMbXy]=(slice->iLastMbQp+iQpDelta+52)%52;		// update last_mb_qp
		slice->iLastMbQp=layer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			layer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3((slice->iLastMbQp+pCtx->m_pps.iChromaQpIndexOffset[i]),0,51)];
		}
		if(MB_TYPE_INTRA16x16==layer->pDec->pMbType[iMbXy]){
			// step1: Luma DC
			READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,&sNeighAvail,pNonZeroCount,pBsAux,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,layer->pScaledTCoeff[iMbXy],layer->pLumaQp[iMbXy],pCtx));
			// step2: Luma AC
			if(uiCbpLuma){
				for(i=0; i<16; i++){
					READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,&sNeighAvail,pNonZeroCount,pBsAux,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,layer->pScaledTCoeff[iMbXy]+(i<<4),layer->pLumaQp[iMbXy],pCtx));
				}
				ST32(&layer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&layer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&layer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&layer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{		// pNonZeroCount=0
				ST32(&layer->pNzc[iMbXy][0],0);
				ST32(&layer->pNzc[iMbXy][4],0);
				ST32(&layer->pNzc[iMbXy][8],0);
				ST32(&layer->pNzc[iMbXy][12],0);
			}
		}else{		// non-MB_TYPE_INTRA16x16
			if(layer->pTransformSize8x8Flag[iMbXy]){
				// Transform 8x8 support for CABAC
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						READ_VERIFY(ParseResidualBlockCabac8x8(cabacDecEngine,&sNeighAvail,pNonZeroCount,pBsAux,(iId8x8<<2),iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,LUMA_DC_AC_INTRA_8,layer->pScaledTCoeff[iMbXy]+(iId8x8<<6),layer->pLumaQp[iMbXy],pCtx));
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&layer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&layer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&layer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&layer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						int32_t iIdx=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							// Luma (DC and AC decoding together)
							READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,&sNeighAvail,pNonZeroCount,pBsAux,iIdx,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,LUMA_DC_AC_INTRA,layer->pScaledTCoeff[iMbXy]+(iIdx<<4),layer->pLumaQp[iMbXy],pCtx));
							iIdx++;
						}
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&layer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&layer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&layer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&layer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}
		}
		int32_t iMbResProperty;
		// chroma
		// step1: DC
		if(1==uiCbpChroma || 2==uiCbpChroma){
			// Cb Cr
			for(i=0; i<2; i++){
				iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,&sNeighAvail,pNonZeroCount,pBsAux,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,layer->pScaledTCoeff[iMbXy]+256+(i<<6),layer->pChromaQp[iMbXy][i],pCtx));
			}
		}

		// step2: AC
		if(2==uiCbpChroma){
			for(i=0; i<2; i++){		// Cb Cr
				iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				int32_t iIdx=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					READ_VERIFY(ParseResidualBlockCabac(cabacDecEngine,&sNeighAvail,pNonZeroCount,pBsAux,iIdx,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,layer->pScaledTCoeff[iMbXy]+(iIdx<<4),layer->pChromaQp[iMbXy][i],pCtx));
					iIdx++;
				}
			}
			ST16(&layer->pNzc[iMbXy][16],LD16(&pNonZeroCount[6+8*1]));
			ST16(&layer->pNzc[iMbXy][20],LD16(&pNonZeroCount[6+8*2]));
			ST16(&layer->pNzc[iMbXy][18],LD16(&pNonZeroCount[6+8*4]));
			ST16(&layer->pNzc[iMbXy][22],LD16(&pNonZeroCount[6+8*5]));
		}else{
			ST16(&layer->pNzc[iMbXy][16],0);
			ST16(&layer->pNzc[iMbXy][20],0);
			ST16(&layer->pNzc[iMbXy][18],0);
			ST16(&layer->pNzc[iMbXy][22],0);
		}
	}else{
		ST32(&layer->pNzc[iMbXy][0],0);
		ST32(&layer->pNzc[iMbXy][4],0);
		ST32(&layer->pNzc[iMbXy][8],0);
		ST32(&layer->pNzc[iMbXy][12],0);
		ST32(&layer->pNzc[iMbXy][16],0);
		ST32(&layer->pNzc[iMbXy][20],0);
	}
	return ERR_NONE;
}
void PlainH264Decoder::FillCacheConstrain0IntraNxN(const SNeighAvail* pNeighAvail,uint8_t* pNonZeroCount,int8_t* pIntraPredMode,SDqLayer* layer){		// no matter slice type
	int32_t iCurXy=layer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;
	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	FillCacheNonZeroCount(pNeighAvail,pNonZeroCount,layer);
	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-layer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}
	// intra4x4_pred_mode
	if(pNeighAvail->iTopAvail && IS_INTRANxN(pNeighAvail->iTopType)){		// top
		ST32(pIntraPredMode+1,LD32(&layer->pIntraPredMode[iTopXy][0]));
	}else{
		int32_t iPred;
		if(pNeighAvail->iTopAvail)
			iPred=0x02020202;
		else
			iPred=0xffffffff;
		ST32(pIntraPredMode+1,iPred);
	}
	if(pNeighAvail->iLeftAvail && IS_INTRANxN(pNeighAvail->iLeftType)){		// left
		pIntraPredMode[0+8*1]=layer->pIntraPredMode[iLeftXy][4];
		pIntraPredMode[0+8*2]=layer->pIntraPredMode[iLeftXy][5];
		pIntraPredMode[0+8*3]=layer->pIntraPredMode[iLeftXy][6];
		pIntraPredMode[0+8*4]=layer->pIntraPredMode[iLeftXy][3];
	}else{
		int8_t iPred;
		if(pNeighAvail->iLeftAvail)
			iPred=2;
		else
			iPred=-1;
		pIntraPredMode[0+8*1]=pIntraPredMode[0+8*2]=pIntraPredMode[0+8*3]=pIntraPredMode[0+8*4]=iPred;
	}
}
void PlainH264Decoder::MapNxNNeighToSampleNormal(const SNeighAvail* pNeighAvail,int32_t* pSampleAvail){
	if(pNeighAvail->iLeftAvail){		// left
		pSampleAvail[6]=
			pSampleAvail[12]=
			pSampleAvail[18]=
			pSampleAvail[24]=1;
	}
	if(pNeighAvail->iLeftTopAvail){		// top_left
		pSampleAvail[0]=1;
	}
	if(pNeighAvail->iTopAvail){		// top
		pSampleAvail[1]=pSampleAvail[2]=pSampleAvail[3]=pSampleAvail[4]=1;
	}
	if(pNeighAvail->iRightTopAvail){		// top_right
		pSampleAvail[5]=1;
	}
}

const int8_t g_kiCabacGlobalContextIdx[CABAC_CONTEXT_COUNT][2][2]={
// 0-10 Table 9-12
 {{20,-15},{20,-15}},
 {{2,54},{2,54}},
 {{3,74},{3,74}},
 {{20,-15},{20,-15}},
 {{2,54},{2,54}},
 {{3,74},{3,74}},
 {{-28,127},{-28,127}},
 {{-23,104},{-23,104}},
 {{-6,53},{-6,53}},
 {{-1,54},{-1,54}},
 {{7,51},{7,51}},
// 11-23 Table 9-13
 {{CTX_NA,CTX_NA},{23,33}},
 {{CTX_NA,CTX_NA},{23,2}},
 {{CTX_NA,CTX_NA},{21,0}},
 {{CTX_NA,CTX_NA},{1,9}},
 {{CTX_NA,CTX_NA},{0,49}},
 {{CTX_NA,CTX_NA},{-37,118}},
 {{CTX_NA,CTX_NA},{5,57}},
 {{CTX_NA,CTX_NA},{-13,78}},
 {{CTX_NA,CTX_NA},{-11,65}},
 {{CTX_NA,CTX_NA},{1,62}},
 {{CTX_NA,CTX_NA},{12,49}},
 {{CTX_NA,CTX_NA},{-4,73}},
 {{CTX_NA,CTX_NA},{17,50}},
// 24-39 Table9-14
 {{CTX_NA,CTX_NA},{18,64}},
 {{CTX_NA,CTX_NA},{9,43}},
 {{CTX_NA,CTX_NA},{29,0}},
 {{CTX_NA,CTX_NA},{26,67}},
 {{CTX_NA,CTX_NA},{16,90}},
 {{CTX_NA,CTX_NA},{9,104}},
 {{CTX_NA,CTX_NA},{-46,127}},
 {{CTX_NA,CTX_NA},{-20,104}},
 {{CTX_NA,CTX_NA},{1,67}},
 {{CTX_NA,CTX_NA},{-13,78}},
 {{CTX_NA,CTX_NA},{-11,65}},
 {{CTX_NA,CTX_NA},{1,62}},
 {{CTX_NA,CTX_NA},{-6,86}},
 {{CTX_NA,CTX_NA},{-17,95}},
 {{CTX_NA,CTX_NA},{-6,61}},
 {{CTX_NA,CTX_NA},{9,45}},
// 40-53 Table 9-15
 {{CTX_NA,CTX_NA},{-3,69}},
 {{CTX_NA,CTX_NA},{-6,81}},
 {{CTX_NA,CTX_NA},{-11,96}},
 {{CTX_NA,CTX_NA},{6,55}},
 {{CTX_NA,CTX_NA},{7,67}},
 {{CTX_NA,CTX_NA},{-5,86}},
 {{CTX_NA,CTX_NA},{2,88}},
 {{CTX_NA,CTX_NA},{0,58}},
 {{CTX_NA,CTX_NA},{-3,76}},
 {{CTX_NA,CTX_NA},{-10,94}},
 {{CTX_NA,CTX_NA},{5,54}},
 {{CTX_NA,CTX_NA},{4,69}},
 {{CTX_NA,CTX_NA},{-3,81}},
 {{CTX_NA,CTX_NA},{0,88}},
// 54-59 Table 9-16
 {{CTX_NA,CTX_NA},{-7,67}},
 {{CTX_NA,CTX_NA},{-5,74}},
 {{CTX_NA,CTX_NA},{-4,74}},
 {{CTX_NA,CTX_NA},{-5,80}},
 {{CTX_NA,CTX_NA},{-7,72}},
 {{CTX_NA,CTX_NA},{1,58}},
// 60-69 Table 9-17
 {{0,41},{0,41}},
 {{0,63},{0,63}},
 {{0,63},{0,63}},
 {{0,63},{0,63}},
 {{-9,83},{-9,83}},
 {{4,86},{4,86}},
 {{0,97},{0,97}},
 {{-7,72},{-7,72}},
 {{13,41},{13,41}},
 {{3,62},{3,62}},
// 70-104 Table 9-18
 {{0,11},{0,45}},
 {{1,55},{-4,78}},
 {{0,69},{-3,96}},
 {{-17,127},{-27,126}},
 {{-13,102},{-28,98}},
 {{0,82},{-25,101}},
 {{-7,74},{-23,67}},
 {{-21,107},{-28,82}},
 {{-27,127},{-20,94}},
 {{-31,127},{-16,83}},
 {{-24,127},{-22,110}},
 {{-18,95},{-21,91}},
 {{-27,127},{-18,102}},
 {{-21,114},{-13,93}},
 {{-30,127},{-29,127}},
 {{-17,123},{-7,92}},
 {{-12,115},{-5,89}},
 {{-16,122},{-7,96}},
 {{-11,115},{-13,108}},
 {{-12,63},{-3,46}},
 {{-2,68},{-1,65}},
 {{-15,84},{-1,57}},
 {{-13,104},{-9,93}},
 {{-3,70},{-3,74}},
 {{-8,93},{-9,92}},
 {{-10,90},{-8,87}},
 {{-30,127},{-23,126}},
 {{-1,74},{5,54}},
 {{-6,97},{6,60}},
 {{-7,91},{6,59}},
 {{-20,127},{6,69}},
 {{-4,56},{-1,48}},
 {{-5,82},{0,68}},
 {{-7,76},{-4,69}},
 {{-22,125},{-8,88}},
// 105-165 Table 9-19
 {{-7,93},{-2,85}},
 {{-11,87},{-6,78}},
 {{-3,77},{-1,75}},
 {{-5,71},{-7,77}},
 {{-4,63},{2,54}},
 {{-4,68},{5,50}},
 {{-12,84},{-3,68}},
 {{-7,62},{1,50}},
 {{-7,65},{6,42}},
 {{8,61},{-4,81}},
 {{5,56},{1,63}},
 {{-2,66},{-4,70}},
 {{1,64},{0,67}},
 {{0,61},{2,57}},
 {{-2,78},{-2,76}},
 {{1,50},{11,35}},
 {{7,52},{4,64}},
 {{10,35},{1,61}},
 {{0,44},{11,35}},
 {{11,38},{18,25}},
 {{1,45},{12,24}},
 {{0,46},{13,29}},
 {{5,44},{13,36}},
 {{31,17},{-10,93}},
 {{1,51},{-7,73}},
 {{7,50},{-2,73}},
 {{28,19},{13,46}},
 {{16,33},{9,49}},
 {{14,62},{-7,100}},
 {{-13,108},{9,53}},
 {{-15,100},{2,53}},
 {{-13,101},{5,53}},
 {{-13,91},{-2,61}},
 {{-12,94},{0,56}},
 {{-10,88},{0,56}},
 {{-16,84},{-13,63}},
 {{-10,86},{-5,60}},
 {{-7,83},{-1,62}},
 {{-13,87},{4,57}},
 {{-19,94},{-6,69}},
 {{1,70},{4,57}},
 {{0,72},{14,39}},
 {{-5,74},{4,51}},
 {{18,59},{13,68}},
 {{-8,102},{3,64}},
 {{-15,100},{1,61}},
 {{0,95},{9,63}},
 {{-4,75},{7,50}},
 {{2,72},{16,39}},
 {{-11,75},{5,44}},
 {{-3,71},{4,52}},
 {{15,46},{11,48}},
 {{-13,69},{-5,60}},
 {{0,62},{-1,59}},
 {{0,65},{0,59}},
 {{21,37},{22,33}},
 {{-15,72},{5,44}},
 {{9,57},{14,43}},
 {{16,54},{-1,78}},
 {{0,62},{0,60}},
 {{12,72},{9,69}},
// 166-226 Table 9-20
 {{24,0},{11,28}},
 {{15,9},{2,40}},
 {{8,25},{3,44}},
 {{13,18},{0,49}},
 {{15,9},{0,46}},
 {{13,19},{2,44}},
 {{10,37},{2,51}},
 {{12,18},{0,47}},
 {{6,29},{4,39}},
 {{20,33},{2,62}},
 {{15,30},{6,46}},
 {{4,45},{0,54}},
 {{1,58},{3,54}},
 {{0,62},{2,58}},
 {{7,61},{4,63}},
 {{12,38},{6,51}},
 {{11,45},{6,57}},
 {{15,39},{7,53}},
 {{11,42},{6,52}},
 {{13,44},{6,55}},
 {{16,45},{11,45}},
 {{12,41},{14,36}},
 {{10,49},{8,53}},
 {{30,34},{-1,82}},
 {{18,42},{7,55}},
 {{10,55},{-3,78}},
 {{17,51},{15,46}},
 {{17,46},{22,31}},
 {{0,89},{-1,84}},
 {{26,-19},{25,7}},
 {{22,-17},{30,-7}},
 {{26,-17},{28,3}},
 {{30,-25},{28,4}},
 {{28,-20},{32,0}},
 {{33,-23},{34,-1}},
 {{37,-27},{30,6}},
 {{33,-23},{30,6}},
 {{40,-28},{32,9}},
 {{38,-17},{31,19}},
 {{33,-11},{26,27}},
 {{40,-15},{26,30}},
 {{41,-6},{37,20}},
 {{38,1},{28,34}},
 {{41,17},{17,70}},
 {{30,-6},{1,67}},
 {{27,3},{5,59}},
 {{26,22},{9,67}},
 {{37,-16},{16,30}},
 {{35,-4},{18,32}},
 {{38,-8},{18,35}},
 {{38,-3},{22,29}},
 {{37,3},{24,31}},
 {{38,5},{23,38}},
 {{42,0},{18,43}},
 {{35,16},{20,41}},
 {{39,22},{11,63}},
 {{14,48},{9,59}},
 {{27,37},{9,64}},
 {{21,60},{-1,94}},
 {{12,68},{-2,89}},
 {{2,97},{-9,108}},
// 227-275 Table 9-21
 {{-3,71},{-6,76}},
 {{-6,42},{-2,44}},
 {{-5,50},{0,45}},
 {{-3,54},{0,52}},
 {{-2,62},{-3,64}},
 {{0,58},{-2,59}},
 {{1,63},{-4,70}},
 {{-2,72},{-4,75}},
 {{-1,74},{-8,82}},
 {{-9,91},{-17,102}},
 {{-5,67},{-9,77}},
 {{-5,27},{3,24}},
 {{-3,39},{0,42}},
 {{-2,44},{0,48}},
 {{0,46},{0,55}},
 {{-16,64},{-6,59}},
 {{-8,68},{-7,71}},
 {{-10,78},{-12,83}},
 {{-6,77},{-11,87}},
 {{-10,86},{-30,119}},
 {{-12,92},{1,58}},
 {{-15,55},{-3,29}},
 {{-10,60},{-1,36}},
 {{-6,62},{1,38}},
 {{-4,65},{2,43}},
 {{-12,73},{-6,55}},
 {{-8,76},{0,58}},
 {{-7,80},{0,64}},
 {{-9,88},{-3,74}},
 {{-17,110},{-10,90}},
 {{-11,97},{0,70}},
 {{-20,84},{-4,29}},
 {{-11,79},{5,31}},
 {{-6,73},{7,42}},
 {{-4,74},{1,59}},
 {{-13,86},{-2,58}},
 {{-13,96},{-3,72}},
 {{-11,97},{-3,81}},
 {{-19,117},{-11,97}},
 {{-8,78},{0,58}},
 {{-5,33},{8,5}},
 {{-4,48},{10,14}},
 {{-2,53},{14,18}},
 {{-3,62},{13,27}},
 {{-13,71},{2,40}},
 {{-10,79},{0,58}},
 {{-12,86},{-3,70}},
 {{-13,90},{-6,79}},
 {{-14,97},{-8,85}},
// 276 no use
 {{CTX_NA,CTX_NA},{CTX_NA,CTX_NA}},
// 277-337 Table 9-22
 {{-6,93},{-13,106}},
 {{-6,84},{-16,106}},
 {{-8,79},{-10,87}},
 {{0,66},{-21,114}},
 {{-1,71},{-18,110}},
 {{0,62},{-14,98}},
 {{-2,60},{-22,110}},
 {{-2,59},{-21,106}},
 {{-5,75},{-18,103}},
 {{-3,62},{-21,107}},
 {{-4,58},{-23,108}},
 {{-9,66},{-26,112}},
 {{-1,79},{-10,96}},
 {{0,71},{-12,95}},
 {{3,68},{-5,91}},
 {{10,44},{-9,93}},
 {{-7,62},{-22,94}},
 {{15,36},{-5,86}},
 {{14,40},{9,67}},
 {{16,27},{-4,80}},
 {{12,29},{-10,85}},
 {{1,44},{-1,70}},
 {{20,36},{7,60}},
 {{18,32},{9,58}},
 {{5,42},{5,61}},
 {{1,48},{12,50}},
 {{10,62},{15,50}},
 {{17,46},{18,49}},
 {{9,64},{17,54}},
 {{-12,104},{10,41}},
 {{-11,97},{7,46}},
 {{-16,96},{-1,51}},
 {{-7,88},{7,49}},
 {{-8,85},{8,52}},
 {{-7,85},{9,41}},
 {{-9,85},{6,47}},
 {{-13,88},{2,55}},
 {{4,66},{13,41}},
 {{-3,77},{10,44}},
 {{-3,76},{6,50}},
 {{-6,76},{5,53}},
 {{10,58},{13,49}},
 {{-1,76},{4,63}},
 {{-1,83},{6,64}},
 {{-7,99},{-2,69}},
 {{-14,95},{-2,59}},
 {{2,95},{6,70}},
 {{0,76},{10,44}},
 {{-5,74},{9,31}},
 {{0,70},{12,43}},
 {{-11,75},{3,53}},
 {{1,68},{14,34}},
 {{0,65},{10,38}},
 {{-14,73},{-3,52}},
 {{3,62},{13,40}},
 {{4,62},{17,32}},
 {{-1,68},{7,44}},
 {{-13,75},{7,38}},
 {{11,55},{13,50}},
 {{5,64},{10,57}},
 {{12,70},{26,43}},
// 338-398 Table9-23
 {{15,6},{14,11}},
 {{6,19},{11,14}},
 {{7,16},{9,11}},
 {{12,14},{18,11}},
 {{18,13},{21,9}},
 {{13,11},{23,-2}},
 {{13,15},{32,-15}},
 {{15,16},{32,-15}},
 {{12,23},{34,-21}},
 {{13,23},{39,-23}},
 {{15,20},{42,-33}},
 {{14,26},{41,-31}},
 {{14,44},{46,-28}},
 {{17,40},{38,-12}},
 {{17,47},{21,29}},
 {{24,17},{45,-24}},
 {{21,21},{53,-45}},
 {{25,22},{48,-26}},
 {{31,27},{65,-43}},
 {{22,29},{43,-19}},
 {{19,35},{39,-10}},
 {{14,50},{30,9}},
 {{10,57},{18,26}},
 {{7,63},{20,27}},
 {{-2,77},{0,57}},
 {{-4,82},{-14,82}},
 {{-3,94},{-5,75}},
 {{9,69},{-19,97}},
 {{-12,109},{-35,125}},
 {{36,-35},{27,0}},
 {{36,-34},{28,0}},
 {{32,-26},{31,-4}},
 {{37,-30},{27,6}},
 {{44,-32},{34,8}},
 {{34,-18},{30,10}},
 {{34,-15},{24,22}},
 {{40,-15},{33,19}},
 {{33,-7},{22,32}},
 {{35,-5},{26,31}},
 {{33,0},{21,41}},
 {{38,2},{26,44}},
 {{33,13},{23,47}},
 {{23,35},{16,65}},
 {{13,58},{14,71}},
 {{29,-3},{8,60}},
 {{26,0},{6,63}},
 {{22,30},{17,65}},
 {{31,-7},{21,24}},
 {{35,-15},{23,20}},
 {{34,-3},{26,23}},
 {{34,3},{27,32}},
 {{36,-1},{28,23}},
 {{34,5},{28,24}},
 {{32,11},{23,40}},
 {{35,5},{24,32}},
 {{34,12},{28,29}},
 {{39,11},{23,42}},
 {{30,29},{19,57}},
 {{34,26},{22,53}},
 {{29,39},{22,61}},
 {{19,66},{11,86}},
 {{31,21},{12,40}},
 {{31,31},{11,51}},
 {{25,50},{14,59}},
// 402-459 Table 9-24
 {{-17,120},{-4,79}},
 {{-20,112},{-7,71}},
 {{-18,114},{-5,69}},
 {{-11,85},{-9,70}},
 {{-15,92},{-8,66}},
 {{-14,89},{-10,68}},
 {{-26,71},{-19,73}},
 {{-15,81},{-12,69}},
 {{-14,80},{-16,70}},
 {{0,68},{-15,67}},
 {{-14,70},{-20,62}},
 {{-24,56},{-19,70}},
 {{-23,68},{-16,66}},
 {{-24,50},{-22,65}},
 {{-11,74},{-20,63}},
 {{23,-13},{9,-2}},
 {{26,-13},{26,-9}},
 {{40,-15},{33,-9}},
 {{49,-14},{39,-7}},
 {{44,3},{41,-2}},
 {{45,6},{45,3}},
 {{44,34},{49,9}},
 {{33,54},{45,27}},
 {{19,82},{36,59}},
 {{-3,75},{-6,66}},
 {{-1,23},{-7,35}},
 {{1,34},{-7,42}},
 {{1,43},{-8,45}},
 {{0,54},{-5,48}},
 {{-2,55},{-12,56}},
 {{0,61},{-6,60}},
 {{1,64},{-5,62}},
 {{0,68},{-8,66}},
 {{-9,92},{-8,76}},
 {{-14,106},{-5,85}},
 {{-13,97},{-6,81}},
 {{-15,90},{-10,77}},
 {{-12,90},{-7,81}},
 {{-18,88},{-17,80}},
 {{-10,73},{-18,73}},
 {{-9,79},{-4,74}},
 {{-14,86},{-10,83}},
 {{-10,73},{-9,71}},
 {{-10,70},{-9,67}},
 {{-10,69},{-1,61}},
 {{-5,66},{-8,66}},
 {{-9,64},{-14,66}},
 {{-5,58},{0,59}},
 {{2,59},{2,59}},
 {{21,-10},{21,-13}},
 {{24,-11},{33,-14}},
 {{28,-8},{39,-7}},
 {{28,-1},{46,-2}},
 {{29,3},{51,2}},
 {{29,9},{60,6}},
 {{35,20},{61,17}},
 {{29,36},{55,34}},
 {{14,67},{42,62}},
};

const uint8_t g_kuiMatrixV[6][8][8]={{		// generated from equation 8-317,8-318
	{20,19,25,19,20,19,25,19},
	{19,18,24,18,19,18,24,18},
	{25,24,32,24,25,24,32,24},
	{19,18,24,18,19,18,24,18},
	{20,19,25,19,20,19,25,19},
	{19,18,24,18,19,18,24,18},
	{25,24,32,24,25,24,32,24},
	{19,18,24,18,19,18,24,18}
},{
	{22,21,28,21,22,21,28,21},
	{21,19,26,19,21,19,26,19},
	{28,26,35,26,28,26,35,26},
	{21,19,26,19,21,19,26,19},
	{22,21,28,21,22,21,28,21},
	{21,19,26,19,21,19,26,19},
	{28,26,35,26,28,26,35,26},
	{21,19,26,19,21,19,26,19}
},{
	{26,24,33,24,26,24,33,24},
	{24,23,31,23,24,23,31,23},
	{33,31,42,31,33,31,42,31},
	{24,23,31,23,24,23,31,23},
	{26,24,33,24,26,24,33,24},
	{24,23,31,23,24,23,31,23},
	{33,31,42,31,33,31,42,31},
	{24,23,31,23,24,23,31,23}
},{
	{28,26,35,26,28,26,35,26},
	{26,25,33,25,26,25,33,25},
	{35,33,45,33,35,33,45,33},
	{26,25,33,25,26,25,33,25},
	{28,26,35,26,28,26,35,26},
	{26,25,33,25,26,25,33,25},
	{35,33,45,33,35,33,45,33},
	{26,25,33,25,26,25,33,25}
},{
	{32,30,40,30,32,30,40,30},
	{30,28,38,28,30,28,38,28},
	{40,38,51,38,40,38,51,38},
	{30,28,38,28,30,28,38,28},
	{32,30,40,30,32,30,40,30},
	{30,28,38,28,30,28,38,28},
	{40,38,51,38,40,38,51,38},
	{30,28,38,28,30,28,38,28}
},{
	{36,34,46,34,36,34,46,34},
	{34,32,43,32,34,32,43,32},
	{46,43,58,43,46,43,58,43},
	{34,32,43,32,34,32,43,32},
	{36,34,46,34,36,34,46,34},
	{34,32,43,32,34,32,43,32},
	{46,43,58,43,46,43,58,43},
	{34,32,43,32,34,32,43,32}
}};

#define LEFT_FLAG_BIT 0
#define TOP_FLAG_BIT 1
#define LEFT_FLAG_MASK 0x01
#define TOP_FLAG_MASK 0x02

#define g_kuiAlphaTable(x) g_kuiAlphaTable[(x)+12]
#define g_kiBetaTable(x) g_kiBetaTable[(x)+12]
#define g_kiTc0Table(x) g_kiTc0Table[(x)+12]

#define MB_BS_MV(pRefPic0,pRefPic1,iMotionVector,iMbXy,iMbBn,iIndex,iNeighIndex) \
(\
	( pRefPic0 !=pRefPic1)  || \
	( WELS_ABS( iMotionVector[iMbXy][iIndex][0]-iMotionVector[iMbBn][iNeighIndex][0] ) >=4 )  || \
	( WELS_ABS( iMotionVector[iMbXy][iIndex][1]-iMotionVector[iMbBn][iNeighIndex][1] ) >=4 )\
)

#define SMB_EDGE_MV(pRefPics,iMotionVector,iIndex,iNeighIndex) \
(\
	( pRefPics[iIndex] !=pRefPics[iNeighIndex] ) || (\
	( WELS_ABS( iMotionVector[iIndex][0]-iMotionVector[iNeighIndex][0] ) &(~3) ) |\
	( WELS_ABS( iMotionVector[iIndex][1]-iMotionVector[iNeighIndex][1] ) &(~3) ))\
)

#define BS_EDGE(bsx1,pRefPics,iMotionVector,iIndex,iNeighIndex) ((bsx1|SMB_EDGE_MV(pRefPics,iMotionVector,iIndex,iNeighIndex))<<((uint8_t)(!!bsx1)))

#define GET_ALPHA_BETA_FROM_QP(iQp,iAlphaOffset,iBetaOffset,iIndex,iAlpha,iBeta) \
{\
	iIndex=(iQp+iAlphaOffset);\
	iAlpha=g_kuiAlphaTable(iIndex);\
	iBeta=g_kiBetaTable((iQp+iBetaOffset));\
}

static const uint8_t g_kuiAlphaTable[52+24]={ //this table refers to Table 8-16 in H.264/AVC standard
	0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,4,4,5,6,
	7,8,9,10,12,13,15,17,20,22,
	25,28,32,36,40,45,50,56,63,71,
	80,90,101,113,127,144,162,182,203,226,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255
};

static const int8_t g_kiBetaTable[52+24]={ //this table refers to Table 8-16 in H.264/AVC standard
	0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,2,2,2,3,
	3,3,3,4,4,4,6,6,7,7,
	8,8,9,9,10,10,11,11,12,12,
	13,13,14,14,15,15,16,16,17,17,
	18,18,18,18,18,18,18,18,18,18,18,18,18,18
};

static const int8_t g_kiTc0Table[52+24][4]={ //this table refers Table 8-17 in H.264/AVC standard
	{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },
	{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },
	{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },
	{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },
	{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,0 },{ -1,0,0,1 },
	{ -1,0,0,1 },{ -1,0,0,1 },{ -1,0,0,1 },{ -1,0,1,1 },{ -1,0,1,1 },{ -1,1,1,1 },
	{ -1,1,1,1 },{ -1,1,1,1 },{ -1,1,1,1 },{ -1,1,1,2 },{ -1,1,1,2 },{ -1,1,1,2 },
	{ -1,1,1,2 },{ -1,1,2,3 },{ -1,1,2,3 },{ -1,2,2,3 },{ -1,2,2,4 },{ -1,2,3,4 },
	{ -1,2,3,4 },{ -1,3,3,5 },{ -1,3,4,6 },{ -1,3,4,6 },{ -1,4,5,7 },{ -1,4,5,8 },
	{ -1,4,6,9 },{ -1,5,7,10 },{ -1,6,8,11 },{ -1,6,8,13 },{ -1,7,10,14 },{ -1,8,11,16 },
	{ -1,9,12,18 },{ -1,10,13,20 },{ -1,11,15,23 },{ -1,13,17,25 },
	{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 },
	{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 },{ -1,13,17,25 }
};

static const uint8_t g_kuiTableBIdx[2][8]={{0,4,8,12,3,7,11,15},{0,1,2,3,12,13,14,15},};

static const uint8_t g_kuiTableB8x8Idx[2][16]={
	{
		0,1,4,5,8,9,12,13,		// 0 1 | 2 3
		2,3,6,7,10,11,14,15		// 4 5 | 6 7
	},{	
		0,1,4,5,2,3,6,7,		// 8 9 | 10 11
		8,9,12,13,10,11,14,15	// 12 13 | 14 15
	},
};

#define TC0_TBL_LOOKUP(tc,iIndexA,pBS,bChroma) \
{\
	tc[0]=g_kiTc0Table(iIndexA)[pBS[0] & 3]+bChroma;\
	tc[1]=g_kiTc0Table(iIndexA)[pBS[1] & 3]+bChroma;\
	tc[2]=g_kiTc0Table(iIndexA)[pBS[2] & 3]+bChroma;\
	tc[3]=g_kiTc0Table(iIndexA)[pBS[3] & 3]+bChroma;\
}


// C code only
void DeblockLumaLt4_c(uint8_t* pPix,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc){
	for(int32_t i=0; i<16; i++){
		int32_t iTc0=pTc[i>>2];
		if(iTc0>=0){
			int32_t p0=pPix[-iStrideX];
			int32_t p1=pPix[-2*iStrideX];
			int32_t p2=pPix[-3*iStrideX];
			int32_t q0=pPix[0];
			int32_t q1=pPix[iStrideX];
			int32_t q2=pPix[2*iStrideX];
			bool bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bool bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bool bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
			int32_t iTc=iTc0;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				bool bDetaP2P0=WELS_ABS(p2-p0)<iBeta;
				bool bDetaQ2Q0=WELS_ABS(q2-q0)<iBeta;
				if(bDetaP2P0){
					pPix[-2*iStrideX]=p1+WELS_CLIP3((p2+((p0+q0+1)>>1)-(p1*(1<<1)))>>1,-iTc0,iTc0);
					iTc++;
				}
				if(bDetaQ2Q0){
					pPix[iStrideX]=q1+WELS_CLIP3((q2+((p0+q0+1)>>1)-(q1*(1<<1)))>>1,-iTc0,iTc0);
					iTc++;
				}
				int32_t iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc,iTc);
				pPix[-iStrideX]=Clip1(p0+iDeta);
				pPix[0]=Clip1(q0-iDeta);
			}
		}
		pPix+=iStrideY;
	}
}
void DeblockLumaEq4_c(uint8_t* pPix,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta){
	int32_t p0,p1,p2,q0,q1,q2;
	int32_t iDetaP0Q0;
	bool bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0; i<16; i++){
		p0=pPix[-iStrideX];
		p1=pPix[-2*iStrideX];
		p2=pPix[-3*iStrideX];
		q0=pPix[0];
		q1=pPix[iStrideX];
		q2=pPix[2*iStrideX];
		iDetaP0Q0=WELS_ABS(p0-q0);
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if((iDetaP0Q0<iAlpha) && bDetaP1P0 && bDetaQ1Q0){
			if(iDetaP0Q0<((iAlpha>>2)+2)){
				bool bDetaP2P0=WELS_ABS(p2-p0)<iBeta;
				bool bDetaQ2Q0=WELS_ABS(q2-q0)<iBeta;
				if(bDetaP2P0){
					const int32_t p3=pPix[-4*iStrideX];
					pPix[-iStrideX]=(p2+(p1*(1<<1))+(p0*(1<<1))+(q0*(1<<1))+q1+4)>>3;	// p0
					pPix[-2*iStrideX]=(p2+p1+p0+q0+2)>>2;								// p1
					pPix[-3*iStrideX]=((p3*(1<<1))+p2+(p2*(1<<1))+p1+p0+q0+4)>>3;		// p2
				}else{
					pPix[-1*iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;							// p0
				}
				if(bDetaQ2Q0){
					const int32_t q3=pPix[3*iStrideX];
					pPix[0]=(p1+(p0*(1<<1))+(q0*(1<<1))+(q1*(1<<1))+q2+4)>>3;			// q0
					pPix[iStrideX]=(p0+q0+q1+q2+2)>>2; 									// q1
					pPix[2*iStrideX]=((q3*(1<<1))+q2+(q2*(1<<1))+q1+q0+p0+4)>>3;		// q2
				}else{
					pPix[0]=((q1*(1<<1))+q0+p1+2)>>2; 									// q0
				}
			}else{
				pPix[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;								// p0
				pPix[0]=((q1*(1<<1))+q0+p1+2)>>2;										// q0
			}
		}
		pPix+=iStrideY;
	}
}

void DeblockLumaLt4V_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockLumaLt4_c(pPix,iStride,1,iAlpha,iBeta,tc);
}
void DeblockLumaLt4H_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockLumaLt4_c(pPix,1,iStride,iAlpha,iBeta,tc);
}
void DeblockLumaEq4V_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockLumaEq4_c(pPix,iStride,1,iAlpha,iBeta);
}
void DeblockLumaEq4H_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockLumaEq4_c(pPix,1,iStride,iAlpha,iBeta);
}
void DeblockChromaLt4_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc){
	int32_t p0,p1,q0,q1,iDeta;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;

	for(int32_t i=0; i<8; i++){
		int32_t iTc0=pTc[i>>1];
		if(iTc0>0){
			p0=pPixCb[-iStrideX];
			p1=pPixCb[-2*iStrideX];
			q0=pPixCb[0];
			q1=pPixCb[iStrideX];

			bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCb[-iStrideX]=Clip1(p0+iDeta);
				pPixCb[0]=Clip1(q0-iDeta);
			}
			p0=pPixCr[-iStrideX];
			p1=pPixCr[-2*iStrideX];
			q0=pPixCr[0];
			q1=pPixCr[iStrideX];
			bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;

			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCr[-iStrideX]=Clip1(p0+iDeta);
				pPixCr[0]=Clip1(q0-iDeta);
			}
		}
		pPixCb+=iStrideY;
		pPixCr+=iStrideY;
	}
}
void DeblockChromaEq4_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta){
	int32_t p0,p1,q0,q1;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0; i<8; i++){
		// cb
		p0=pPixCb[-iStrideX];
		p1=pPixCb[-2*iStrideX];
		q0=pPixCb[0];
		q1=pPixCb[iStrideX];
		bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
			pPixCb[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;
			pPixCb[0]=((q1*(1<<1))+q0+p1+2)>>2;
		}

		// cr
		p0=pPixCr[-iStrideX];
		p1=pPixCr[-2*iStrideX];
		q0=pPixCr[0];
		q1=pPixCr[iStrideX];
		bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
			pPixCr[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;
			pPixCr[0]=((q1*(1<<1))+q0+p1+2)>>2;
		}
		pPixCr+=iStrideY;
		pPixCb+=iStrideY;
	}
}
void DeblockChromaLt4V_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt4_c(pPixCb,pPixCr,iStride,1,iAlpha,iBeta,tc);
}
void DeblockChromaLt4H_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt4_c(pPixCb,pPixCr,1,iStride,iAlpha,iBeta,tc);
}
void DeblockChromaEq4V_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq4_c(pPixCb,pPixCr,iStride,1,iAlpha,iBeta);
}
void DeblockChromaEq4H_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq4_c(pPixCb,pPixCr,1,iStride,iAlpha,iBeta);
}

void DeblockChromaLt42_c(uint8_t* pPixCbCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc){
	int32_t p0,p1,q0,q1,iDeta;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;

	for(int32_t i=0; i<8; i++){
		int32_t iTc0=pTc[i>>1];
		if(iTc0>0){
			p0=pPixCbCr[-iStrideX];
			p1=pPixCbCr[-2*iStrideX];
			q0=pPixCbCr[0];
			q1=pPixCbCr[iStrideX];

			bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCbCr[-iStrideX]=Clip1(p0+iDeta);
				pPixCbCr[0]=Clip1(q0-iDeta);
			}


		}
		pPixCbCr+=iStrideY;
	}
}
void DeblockChromaEq42_c(uint8_t* pPixCbCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta){
	int32_t p0,p1,q0,q1;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0; i<8; i++){
		p0=pPixCbCr[-iStrideX];
		p1=pPixCbCr[-2*iStrideX];
		q0=pPixCbCr[0];
		q1=pPixCbCr[iStrideX];
		bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
			pPixCbCr[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;
			pPixCbCr[0]=((q1*(1<<1))+q0+p1+2)>>2;
		}
		pPixCbCr+=iStrideY;
	}
}

void DeblockChromaLt4V2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt42_c(pPixCbCr,iStride,1,iAlpha,iBeta,tc);
}
void DeblockChromaLt4H2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt42_c(pPixCbCr,1,iStride,iAlpha,iBeta,tc);
}
void DeblockChromaEq4V2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq42_c(pPixCbCr,iStride,1,iAlpha,iBeta);
}
void DeblockChromaEq4H2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq42_c(pPixCbCr,1,iStride,iAlpha,iBeta);
}

void PlainH264Decoder::FilteringEdgeLumaIntraV(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
	if(iAlpha|iBeta){
		DeblockLumaEq4H_c(pPix,iStride,iAlpha,iBeta);
	}
}
void PlainH264Decoder::FilteringEdgeLumaIntraH(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
	if(iAlpha|iBeta){
		DeblockLumaEq4V_c(pPix,iStride,iAlpha,iBeta);
	}
}

void PlainH264Decoder::FilteringEdgeLumaHV(SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	int32_t iMbXyIndex=layer->iMbXyIndex;
	int32_t iMbX=layer->iMbX;
	int32_t iMbY=layer->iMbY;
	int32_t iMbWidth=layer->iMbWidth;
	int32_t iLineSize=pFilter->iCsStride[0];
	uint8_t* pDestY;
	int32_t iCurQp;
	int32_t iIndexA,iAlpha,iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBSx4,4,4);
	pDestY=pFilter->pCsData[0]+((iMbY*iLineSize+iMbX)<<4);
	iCurQp=layer->pLumaQp[iMbXyIndex];
	*(uint32_t*)uiBSx4=0x03030303;
	// luma v
	if(iBoundryFlag&LEFT_FLAG_MASK){
		pFilter->iLumaQP=(iCurQp+layer->pLumaQp[iMbXyIndex-1]+1)>>1;
		FilteringEdgeLumaIntraV(pFilter,pDestY,iLineSize,NULL);
	}
	pFilter->iLumaQP=iCurQp;
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
	if(iAlpha|iBeta){
		TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,0);
		if(!layer->pTransformSize8x8Flag[iMbXyIndex]){
			DeblockLumaLt4H_c(&pDestY[1<<2],iLineSize,iAlpha,iBeta,iTc);
		}
		DeblockLumaLt4H_c(&pDestY[2<<2],iLineSize,iAlpha,iBeta,iTc);
		if(!layer->pTransformSize8x8Flag[iMbXyIndex]){
			DeblockLumaLt4H_c(&pDestY[3<<2],iLineSize,iAlpha,iBeta,iTc);
		}
	}
	// luma h
	if(iBoundryFlag&TOP_FLAG_MASK){
		pFilter->iLumaQP=(iCurQp+layer->pLumaQp[iMbXyIndex-iMbWidth]+1)>>1;
		FilteringEdgeLumaIntraH(pFilter,pDestY,iLineSize,NULL);
	}
	pFilter->iLumaQP=iCurQp;
	if(iAlpha|iBeta){
		if(!layer->pTransformSize8x8Flag[iMbXyIndex]){
			DeblockLumaLt4V_c(&pDestY[(1<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		}
		DeblockLumaLt4V_c(&pDestY[(2<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		if(!layer->pTransformSize8x8Flag[iMbXyIndex]){
			DeblockLumaLt4V_c(&pDestY[(3<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		}
	}
}
void PlainH264Decoder::FilteringEdgeChromaIntraV(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){		// QP of cb and cr are the same
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			DeblockChromaEq4H_c(pPixCb,pPixCr,iStride,iAlpha,iBeta);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				DeblockChromaEq4H2_c(pPixCbCr,iStride,iAlpha,iBeta);
			}
		}
	}
}

void PlainH264Decoder::FilteringEdgeChromaIntraH(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			DeblockChromaEq4V_c(pPixCb,pPixCr,iStride,iAlpha,iBeta);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				DeblockChromaEq4V2_c(pPixCbCr,iStride,iAlpha,iBeta);
			}
		}
	}
}

void PlainH264Decoder::FilteringEdgeChromaHV(SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	int32_t iMbXyIndex=layer->iMbXyIndex;
	int32_t iMbX=layer->iMbX;
	int32_t iMbY=layer->iMbY;
	int32_t iMbWidth=layer->iMbWidth;
	int32_t iLineSize=pFilter->iCsStride[1];
	uint8_t* pDestCb;
	uint8_t* pDestCr;
	int8_t* pCurQp;
	int32_t iIndexA,iAlpha,iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBSx4,4,4);
	pDestCb=pFilter->pCsData[1]+((iMbY*iLineSize+iMbX)<<3);
	pDestCr=pFilter->pCsData[2]+((iMbY*iLineSize+iMbX)<<3);
	pCurQp=layer->pChromaQp[iMbXyIndex];
	*(uint32_t*)uiBSx4=0x03030303;
	// chroma v
	if(iBoundryFlag&LEFT_FLAG_MASK){
		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurQp[i]+layer->pChromaQp[iMbXyIndex-1][i]+1)>>1;
		}
		FilteringEdgeChromaIntraV(pFilter,pDestCb,pDestCr,iLineSize,NULL);
	}
	pFilter->iChromaQP[0]=pCurQp[0];
	pFilter->iChromaQP[1]=pCurQp[1];
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
			DeblockChromaLt4H_c(&pDestCb[2<<1],&pDestCr[2<<1],iLineSize,iAlpha,iBeta,iTc);
		}
	}else{

		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pDestCbCr=(i==0) ? &pDestCb[2<<1] : &pDestCr[2<<1];
				TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
				DeblockChromaLt4H2_c(pDestCbCr,iLineSize,iAlpha,iBeta,iTc);
			}

		}
	}
	// chroma h
	if(iBoundryFlag&TOP_FLAG_MASK){
		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurQp[i]+layer->pChromaQp[iMbXyIndex-iMbWidth][i]+1)>>1;
		}
		FilteringEdgeChromaIntraH(pFilter,pDestCb,pDestCr,iLineSize,NULL);
	}
	pFilter->iChromaQP[0]=pCurQp[0];
	pFilter->iChromaQP[1]=pCurQp[1];
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
			DeblockChromaLt4V_c(&pDestCb[(2<<1)*iLineSize],&pDestCr[(2<<1)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
				uint8_t* pDestCbCr=(i==0) ? &pDestCb[(2<<1)*iLineSize] : &pDestCr[(2<<1)*iLineSize];
				DeblockChromaLt4V2_c(pDestCbCr,iLineSize,iAlpha,iBeta,iTc);
			}
		}
	}
}

// merge h&v lookup table operation to save performance
void PlainH264Decoder::DeblockingIntraMb(SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	FilteringEdgeLumaHV(layer,pFilter,iBoundryFlag);
	FilteringEdgeChromaHV(layer,pFilter,iBoundryFlag);
}

inline int8_t* GetPNzc(SDqLayer* layer,int32_t iMbXy){
	return layer->pNzc[iMbXy];
}
uint32_t PlainH264Decoder::DeblockingBsMarginalMBAvcbase(SDeblockingFilter* pFilter,SDqLayer* layer,int32_t iEdge,int32_t iNeighMb,int32_t iMbXy){
	int32_t i,j;
	uint32_t uiBSx4;
	uint8_t* pBS=(uint8_t*)(&uiBSx4);
	const uint8_t* pBIdx=&g_kuiTableBIdx[iEdge][0];
	const uint8_t* pBnIdx=&g_kuiTableBIdx[iEdge][4];
	const uint8_t* pB8x8Idx=&g_kuiTableB8x8Idx[iEdge][0];
	const uint8_t* pBn8x8Idx=&g_kuiTableB8x8Idx[iEdge][8];
	int8_t(*iRefIdx)[MB_BLOCK4x4_NUM]=layer->pDec!=NULL ? layer->pDec->pRefIndex[LIST_0] :layer->pRefIndex[LIST_0];
	if(layer->pTransformSize8x8Flag[iMbXy] && layer->pTransformSize8x8Flag[iNeighMb]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=(GetPNzc(layer,iMbXy)[*(pB8x8Idx+j)]|GetPNzc(layer,iNeighMb)[*(pBn8x8Idx+j)]);
			}
			if(uiNzc){
				pBS[i<<1]=pBS[1+(i<<1)]=2;
			}else{
				SPicture* ref0=(iRefIdx[iMbXy][*pB8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pB8x8Idx]] : NULL;
				SPicture* ref1=(iRefIdx[iNeighMb][*pBn8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBn8x8Idx]] :NULL;
				pBS[i<<1]=pBS[1+(i<<1)]=MB_BS_MV(ref0,ref1,layer->pDec->pMv[LIST_0],iMbXy,iNeighMb,*pB8x8Idx,*pBn8x8Idx);
			}
			pB8x8Idx+=4;
			pBn8x8Idx+=4;
		}
	}else
	if(layer->pTransformSize8x8Flag[iMbXy]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=GetPNzc(layer,iMbXy)[*(pB8x8Idx+j)];
			}
			for(j=0; j<2; j++){
				if(uiNzc|GetPNzc(layer,iNeighMb)[*pBnIdx]){
					pBS[j+(i<<1)]=2;
				}else{
					SPicture* ref0=(iRefIdx[iMbXy][*pB8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pB8x8Idx]] : NULL;
					SPicture* ref1=(iRefIdx[iNeighMb][*pBnIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBnIdx]] : NULL;
					pBS[j+(i<<1)]=MB_BS_MV(ref0,ref1,(layer->pDec!=NULL ? layer->pDec->pMv[LIST_0] : layer->pMv[LIST_0]),iMbXy,iNeighMb,*pB8x8Idx,*pBnIdx);
				}
				pBnIdx++;
			}
			pB8x8Idx+=4;
		}
	}else
	if(layer->pTransformSize8x8Flag[iNeighMb]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=GetPNzc(layer,iNeighMb)[*(pBn8x8Idx+j)];
			}
			for(j=0; j<2; j++){
				if(uiNzc|GetPNzc(layer,iMbXy)[*pBIdx]){
					pBS[j+(i<<1)]=2;
				}else{
					SPicture* ref0=(iRefIdx[iMbXy][*pBIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pBIdx]] : NULL;
					SPicture* ref1=(iRefIdx[iNeighMb][*pBn8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBn8x8Idx]] :NULL;
					pBS[j+(i<<1)]=MB_BS_MV(ref0,ref1,(layer->pDec!=NULL ? layer->pDec->pMv[LIST_0] : layer->pMv[LIST_0]),iMbXy,iNeighMb,*pBIdx,*pBn8x8Idx);
				}
				pBIdx++;
			}
			pBn8x8Idx+=4;
		}
	}else{
		// only 4x4 transform
		for(i=0; i<4; i++){
			if(GetPNzc(layer,iMbXy)[*pBIdx]|GetPNzc(layer,iNeighMb)[*pBnIdx]){
				pBS[i]=2;
			}else{
				SPicture* ref0=(iRefIdx[iMbXy][*pBIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pBIdx]] : NULL;
				SPicture* ref1=(iRefIdx[iNeighMb][*pBnIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBnIdx]] : NULL;
				pBS[i]=MB_BS_MV(ref0,ref1,(layer->pDec!=NULL ? layer->pDec->pMv[LIST_0] : layer->pMv[LIST_0]),iMbXy,iNeighMb,*pBIdx,*pBnIdx);
			}
			pBIdx++;
			pBnIdx++;
		}
	}

	return uiBSx4;
}
void PlainH264Decoder::DeblockingBSInsideMBAvsbase(int8_t* pNnzTab,uint8_t nBS[2][4][4],int32_t iLShiftFactor){
	uint32_t uiNnz32b0,uiNnz32b1,uiNnz32b2,uiNnz32b3;

	uiNnz32b0=*(uint32_t*)(pNnzTab+0);
	uiNnz32b1=*(uint32_t*)(pNnzTab+4);
	uiNnz32b2=*(uint32_t*)(pNnzTab+8);
	uiNnz32b3=*(uint32_t*)(pNnzTab+12);

	nBS[0][1][0]=(pNnzTab[0]|pNnzTab[1])<<iLShiftFactor;
	nBS[0][2][0]=(pNnzTab[1]|pNnzTab[2])<<iLShiftFactor;
	nBS[0][3][0]=(pNnzTab[2]|pNnzTab[3])<<iLShiftFactor;

	nBS[0][1][1]=(pNnzTab[4]|pNnzTab[5])<<iLShiftFactor;
	nBS[0][2][1]=(pNnzTab[5]|pNnzTab[6])<<iLShiftFactor;
	nBS[0][3][1]=(pNnzTab[6]|pNnzTab[7])<<iLShiftFactor;
	*(uint32_t*)nBS[1][1]=(uiNnz32b0|uiNnz32b1)<<iLShiftFactor;

	nBS[0][1][2]=(pNnzTab[8]|pNnzTab[9])<<iLShiftFactor;
	nBS[0][2][2]=(pNnzTab[9]|pNnzTab[10])<<iLShiftFactor;
	nBS[0][3][2]=(pNnzTab[10]|pNnzTab[11])<<iLShiftFactor;
	*(uint32_t*)nBS[1][2]=(uiNnz32b1|uiNnz32b2)<<iLShiftFactor;

	nBS[0][1][3]=(pNnzTab[12]|pNnzTab[13])<<iLShiftFactor;
	nBS[0][2][3]=(pNnzTab[13]|pNnzTab[14])<<iLShiftFactor;
	nBS[0][3][3]=(pNnzTab[14]|pNnzTab[15])<<iLShiftFactor;
	*(uint32_t*)nBS[1][3]=(uiNnz32b2|uiNnz32b3)<<iLShiftFactor;
}

// // // pNonZeroCount[16+8] mapping scan index
const uint8_t g_kuiMbCountScan4Idx[24]={
	// 0 1 | 4 5 luma 8*8 block pNonZeroCount[16+8]
	0,1,4,5,		// 2 3 | 6 7 0 | 1 0 1 2 3
	2,3,6,7,		// --------------- --------- 4 5 6 7
	8,9,12,13,		// 8 9 | 12 13 2 | 3 8 9 10 11
	10,11,14,15,		// 10 11 | 14 15-----------------------------> 12 13 14 15
	16,17,20,21,		// ---------------- chroma 8*8 block 16 17 18 19
	18,19,22,23		// 16 17 | 20 21 0 1 20 21 22 23
};

void PlainH264Decoder::DeblockingBSInsideMBAvsbase8x8(int8_t* pNnzTab,uint8_t nBS[2][4][4],int32_t iLShiftFactor){
	int8_t i8x8NnzTab[4];
	for(int32_t i=0; i<4; i++){
		int32_t iBlkIdx=i<<2;
		i8x8NnzTab[i]=(pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+1]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+2]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+3]]);
	}
// vertical
	nBS[0][2][0]=nBS[0][2][1]=(i8x8NnzTab[0]|i8x8NnzTab[1])<<iLShiftFactor;
	nBS[0][2][2]=nBS[0][2][3]=(i8x8NnzTab[2]|i8x8NnzTab[3])<<iLShiftFactor;
// horizontal
	nBS[1][2][0]=nBS[1][2][1]=(i8x8NnzTab[0]|i8x8NnzTab[2])<<iLShiftFactor;
	nBS[1][2][2]=nBS[1][2][3]=(i8x8NnzTab[1]|i8x8NnzTab[3])<<iLShiftFactor;
}

void PlainH264Decoder::DeblockingBSliceBSInsideMBNormal(SDeblockingFilter* pFilter,SDqLayer* layer,uint8_t nBS[2][4][4],int8_t* pNnzTab,int32_t iMbXy){
	uint32_t uiNnz32b0,uiNnz32b1,uiNnz32b2,uiNnz32b3;
	void* iRefs[LIST_A][MB_BLOCK4x4_NUM];
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBsx4,4,4);
	int8_t i8x8NnzTab[4];
	int l;
	for(l=0; l<LIST_A; l++){
		int8_t* iRefIdx=layer->pDec->pRefIndex[l][iMbXy];
		int i;
		// Look up each reference picture based on indices
		for(i=0; i<MB_BLOCK4x4_NUM; i++){
			if(iRefIdx[i]>REF_NOT_IN_LIST)
				iRefs[l][i]=pFilter->pRefPics[l][iRefIdx[i]];
			else
				iRefs[l][i]=NULL;
		}
	}
	if(layer->pTransformSize8x8Flag[iMbXy]){
		for(int32_t i=0; i<4; i++){
			int32_t iBlkIdx=i<<2;
			i8x8NnzTab[i]=(pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+1]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+2]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+3]]);
		}
		// vertical
		int8_t iIndex=g_kuiMbCountScan4Idx[1<<2];
		int8_t iNeigborIndex=g_kuiMbCountScan4Idx[0];
		nBS[0][2][0]=nBS[0][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[0][2][0]=nBS[0][2][1]=BS_EDGE((i8x8NnzTab[0]|i8x8NnzTab[1]),iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}
		iIndex=g_kuiMbCountScan4Idx[3<<2];
		iNeigborIndex=g_kuiMbCountScan4Idx[2<<2];
		nBS[0][2][2]=nBS[0][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[0][2][2]=nBS[0][2][3]=BS_EDGE((i8x8NnzTab[2]|i8x8NnzTab[3]),iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}

		// horizontal
		iIndex=g_kuiMbCountScan4Idx[2<<2];
		iNeigborIndex=g_kuiMbCountScan4Idx[0];
		nBS[1][2][0]=nBS[1][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[1][2][0]=nBS[1][2][1]=BS_EDGE((i8x8NnzTab[0]|i8x8NnzTab[2]),iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}

		iIndex=g_kuiMbCountScan4Idx[3<<2];
		iNeigborIndex=g_kuiMbCountScan4Idx[1<<2];
		nBS[1][2][2]=nBS[1][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[1][2][2]=nBS[1][2][3]=BS_EDGE((i8x8NnzTab[1]|i8x8NnzTab[3]),iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}
	}else{
		uiNnz32b0=*(uint32_t*)(pNnzTab+0);
		uiNnz32b1=*(uint32_t*)(pNnzTab+4);
		uiNnz32b2=*(uint32_t*)(pNnzTab+8);
		uiNnz32b3=*(uint32_t*)(pNnzTab+12);

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[i]|pNnzTab[i+1];
		nBS[0][1][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][1] && iRefs[listIdx][0]){
				nBS[0][1][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],1,0);
				break;
			}
		}
		nBS[0][2][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][2] && iRefs[listIdx][1]){
				nBS[0][2][0]=BS_EDGE(uiBsx4[1],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],2,1);
				break;
			}
		}
		nBS[0][3][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][3] && iRefs[listIdx][2]){
				nBS[0][3][0]=BS_EDGE(uiBsx4[2],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],3,2);
				break;
			}
		}

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[4+i]|pNnzTab[4+i+1];
		nBS[0][1][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][5] && iRefs[listIdx][4]){
				nBS[0][1][1]=BS_EDGE(uiBsx4[0],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],5,4);
				break;
			}
		}
		nBS[0][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][6] && iRefs[listIdx][5]){
				nBS[0][2][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],6,5);
				break;
			}
		}
		nBS[0][3][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][7] && iRefs[listIdx][6]){
				nBS[0][3][1]=BS_EDGE(uiBsx4[2],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],7,6);
				break;
			}
		}

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[8+i]|pNnzTab[8+i+1];
		nBS[0][1][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][9] && iRefs[listIdx][8]){
				nBS[0][1][2]=BS_EDGE(uiBsx4[0],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],9,8);
				break;
			}
		}
		nBS[0][2][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][10] && iRefs[listIdx][9]){
				nBS[0][2][2]=BS_EDGE(uiBsx4[1],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],10,9);
				break;
			}
		}
		nBS[0][3][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][11] && iRefs[listIdx][10]){
				nBS[0][3][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],11,10);
				break;
			}
		}

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[12+i]|pNnzTab[12+i+1];
		nBS[0][1][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][13] && iRefs[listIdx][12]){
				nBS[0][1][3]=BS_EDGE(uiBsx4[0],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],13,12);
				break;
			}
		}
		nBS[0][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][14] && iRefs[listIdx][13]){
				nBS[0][2][3]=BS_EDGE(uiBsx4[1],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],14,13);
				break;
			}
		}
		nBS[0][3][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][15] && iRefs[listIdx][14]){
				nBS[0][3][3]=BS_EDGE(uiBsx4[2],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],15,14);
				break;
			}
		}

		// horizontal
		*(uint32_t*)uiBsx4=(uiNnz32b0|uiNnz32b1);
		nBS[1][1][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][4] && iRefs[listIdx][0]){
				nBS[1][1][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],4,0);
				break;
			}
		}
		nBS[1][1][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][5] && iRefs[listIdx][1]){
				nBS[1][1][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],5,1);
				break;
			}
		}
		nBS[1][1][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][6] && iRefs[listIdx][2]){
				nBS[1][1][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],6,2);
				break;
			}
		}
		nBS[1][1][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][7] && iRefs[listIdx][3]){
				nBS[1][1][3]=BS_EDGE(uiBsx4[3],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],7,3);
				break;
			}
		}

		*(uint32_t*)uiBsx4=(uiNnz32b1|uiNnz32b2);
		nBS[1][2][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][8] && iRefs[listIdx][4]){
				nBS[1][2][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],8,4);
				break;
			}
		}
		nBS[1][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][9] && iRefs[listIdx][5]){
				nBS[1][2][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],9,5);
				break;
			}
		}
		nBS[1][2][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][10] && iRefs[listIdx][6]){
				nBS[1][2][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],10,6);
				break;
			}
		}
		nBS[1][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][11] && iRefs[listIdx][7]){
				nBS[1][2][3]=BS_EDGE(uiBsx4[3],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],11,7);
				break;
			}
		}

		*(uint32_t*)uiBsx4=(uiNnz32b2|uiNnz32b3);
		nBS[1][3][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][12] && iRefs[listIdx][8]){
				nBS[1][3][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],12,8);
				break;
			}
		}
		nBS[1][3][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][13] && iRefs[listIdx][9]){
				nBS[1][3][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],13,9);
				break;
			}
		}
		nBS[1][3][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][14] && iRefs[listIdx][10]){
				nBS[1][3][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],14,10);
				break;
			}
		}
		nBS[1][3][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][15] && iRefs[listIdx][11]){
				nBS[1][3][3]=BS_EDGE(uiBsx4[3],iRefs[listIdx],layer->pDec->pMv[listIdx][iMbXy],15,11);
				break;
			}
		}
	}
}

void PlainH264Decoder::FilteringEdgeLumaV(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
	if(iAlpha|iBeta){
		TC0_TBL_LOOKUP(tc,iIndexA,pBS,0);
		DeblockLumaLt4H_c(pPix,iStride,iAlpha,iBeta,tc);
	}
}

void PlainH264Decoder::FilteringEdgeChromaV(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
			DeblockChromaLt4H_c(pPixCb,pPixCr,iStride,iAlpha,iBeta,tc);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
				DeblockChromaLt4H2_c(pPixCbCr,iStride,iAlpha,iBeta,tc);
			}
		}
	}
}
void PlainH264Decoder::FilteringEdgeLumaH(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
	if(iAlpha|iBeta){
		TC0_TBL_LOOKUP(tc,iIndexA,pBS,0);
		DeblockLumaLt4V_c(pPix,iStride,iAlpha,iBeta,tc);
	}
}
void PlainH264Decoder::FilteringEdgeChromaH(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
			DeblockChromaLt4V_c(pPixCb,pPixCr,iStride,iAlpha,iBeta,tc);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
				DeblockChromaLt4V2_c(pPixCbCr,iStride,iAlpha,iBeta,tc);
			}
		}
	}
}

void PlainH264Decoder::DeblockingInterMb(SDqLayer* layer,SDeblockingFilter* pFilter,uint8_t nBS[2][4][4],int32_t iBoundryFlag){
	int32_t iMbXyIndex=layer->iMbXyIndex;
	int32_t iMbX=layer->iMbX;
	int32_t iMbY=layer->iMbY;
	int32_t iCurLumaQp=layer->pLumaQp[iMbXyIndex];
	// int32_t* iCurChromaQp=layer->pChromaQp[iMbXyIndex];
	int8_t* pCurChromaQp=layer->pChromaQp[iMbXyIndex];
	int32_t iLineSize=pFilter->iCsStride[0];
	int32_t iLineSizeUV=pFilter->iCsStride[1];
	uint8_t* pDestY,* pDestCb,* pDestCr;
	pDestY=pFilter->pCsData[0]+((iMbY*iLineSize+iMbX)<<4);
	pDestCb=pFilter->pCsData[1]+((iMbY*iLineSizeUV+iMbX)<<3);
	pDestCr=pFilter->pCsData[2]+((iMbY*iLineSizeUV+iMbX)<<3);
	// Vertical margin
	if(iBoundryFlag&LEFT_FLAG_MASK){
		int32_t iLeftXyIndex=iMbXyIndex-1;
		pFilter->iLumaQP=(iCurLumaQp+layer->pLumaQp[iLeftXyIndex]+1)>>1;
		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurChromaQp[i]+layer->pChromaQp[iLeftXyIndex][i]+1)>>1;
		}
		if(nBS[0][0][0]==0x04){
			FilteringEdgeLumaIntraV(pFilter,pDestY,iLineSize,NULL);
			FilteringEdgeChromaIntraV(pFilter,pDestCb,pDestCr,iLineSizeUV,NULL);
		}else{
			if(*(uint32_t*)nBS[0][0]!=0){
				FilteringEdgeLumaV(pFilter,pDestY,iLineSize,nBS[0][0]);
				FilteringEdgeChromaV(pFilter,pDestCb,pDestCr,iLineSizeUV,nBS[0][0]);
			}
		}
	}
	pFilter->iLumaQP=iCurLumaQp;
	pFilter->iChromaQP[0]=pCurChromaQp[0];
	pFilter->iChromaQP[1]=pCurChromaQp[1];
	if(*(uint32_t*)nBS[0][1]!=0 && !layer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaV(pFilter,&pDestY[1<<2],iLineSize,nBS[0][1]);
	}
	if(*(uint32_t*)nBS[0][2]!=0){
		FilteringEdgeLumaV(pFilter,&pDestY[2<<2],iLineSize,nBS[0][2]);
		FilteringEdgeChromaV(pFilter,&pDestCb[2<<1],&pDestCr[2<<1],iLineSizeUV,nBS[0][2]);
	}
	if(*(uint32_t*)nBS[0][3]!=0 && !layer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaV(pFilter,&pDestY[3<<2],iLineSize,nBS[0][3]);
	}
	if(iBoundryFlag&TOP_FLAG_MASK){
		int32_t iTopXyIndex=iMbXyIndex-layer->iMbWidth;
		pFilter->iLumaQP=(iCurLumaQp+layer->pLumaQp[iTopXyIndex]+1)>>1;
		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurChromaQp[i]+layer->pChromaQp[iTopXyIndex][i]+1)>>1;
		}
		if(nBS[1][0][0]==0x04){
			FilteringEdgeLumaIntraH(pFilter,pDestY,iLineSize,NULL);
			FilteringEdgeChromaIntraH(pFilter,pDestCb,pDestCr,iLineSizeUV,NULL);
		}else{
			if(*(uint32_t*)nBS[1][0]!=0){
				FilteringEdgeLumaH(pFilter,pDestY,iLineSize,nBS[1][0]);
				FilteringEdgeChromaH(pFilter,pDestCb,pDestCr,iLineSizeUV,nBS[1][0]);
			}
		}
	}
	pFilter->iLumaQP=iCurLumaQp;
	pFilter->iChromaQP[0]=pCurChromaQp[0];
	pFilter->iChromaQP[1]=pCurChromaQp[1];
	if(*(uint32_t*)nBS[1][1]!=0 && !layer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaH(pFilter,&pDestY[(1<<2)*iLineSize],iLineSize,nBS[1][1]);
	}
	if(*(uint32_t*)nBS[1][2]!=0){
		FilteringEdgeLumaH(pFilter,&pDestY[(2<<2)*iLineSize],iLineSize,nBS[1][2]);
		FilteringEdgeChromaH(pFilter,&pDestCb[(2<<1)*iLineSizeUV],&pDestCr[(2<<1)*iLineSizeUV],iLineSizeUV,nBS[1][2]);
	}
	if(*(uint32_t*)nBS[1][3]!=0 && !layer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaH(pFilter,&pDestY[(3<<2)*iLineSize],iLineSize,nBS[1][3]);
	}
}

void PlainH264Decoder::DeblockingMb(SDecoderContext* pCtx,SDqLayer* layer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	uint8_t nBS[2][4][4]={{{0}}};
	int32_t iMbXyIndex=layer->iMbXyIndex;
	uint32_t iCurMbType=layer->pDec!=NULL ? layer->pDec->pMbType[iMbXyIndex] : layer->pMbType[iMbXyIndex];
	int32_t iMbNb;
	//SSlice* slice=&pCtx->m_slice;
	switch(iCurMbType){
		case MB_TYPE_INTRA4x4:
		case MB_TYPE_INTRA8x8:
		case MB_TYPE_INTRA16x16:
		case MB_TYPE_INTRA_PCM:
			DeblockingIntraMb(layer,pFilter,iBoundryFlag);
			break;
		default:
			if(iBoundryFlag&LEFT_FLAG_MASK){
				iMbNb=iMbXyIndex-1;
				uint32_t uiMbType=layer->pDec!=NULL ? layer->pDec->pMbType[iMbNb] : layer->pMbType[iMbNb];
				*(uint32_t*)nBS[0][0]=IS_INTRA(uiMbType) ? 0x04040404 : DeblockingBsMarginalMBAvcbase(pFilter,layer,0,iMbNb,iMbXyIndex);
			}else{
				*(uint32_t*)nBS[0][0]=0;
			}
			if(iBoundryFlag&TOP_FLAG_MASK){
				iMbNb=iMbXyIndex-layer->iMbWidth;
				uint32_t uiMbType=layer->pDec!=NULL ? layer->pDec->pMbType[iMbNb] : layer->pMbType[iMbNb];
				*(uint32_t*)nBS[1][0]=IS_INTRA(uiMbType) ? 0x04040404 : DeblockingBsMarginalMBAvcbase(pFilter,layer,1,iMbNb,iMbXyIndex);
			}else{
				*(uint32_t*)nBS[1][0]=0;
			}
			// SKIP MB_16x16 or others
			if(IS_SKIP(iCurMbType)){
				*(uint32_t*)nBS[0][1]=*(uint32_t*)nBS[0][2]=*(uint32_t*)nBS[0][3]=
					*(uint32_t*)nBS[1][1]=*(uint32_t*)nBS[1][2]=*(uint32_t*)nBS[1][3]=0;
			}else{
				if(IS_INTER_16x16(iCurMbType)){
					if(!layer->pTransformSize8x8Flag[layer->iMbXyIndex]){
						DeblockingBSInsideMBAvsbase(GetPNzc(layer,iMbXyIndex),nBS,1);
					}else{
						DeblockingBSInsideMBAvsbase8x8(GetPNzc(layer,iMbXyIndex),nBS,1);
					}
				}else{
					DeblockingBSliceBSInsideMBNormal(pFilter,layer,nBS,GetPNzc(layer,iMbXyIndex),iMbXyIndex);
				}
			}
			DeblockingInterMb(layer,pFilter,nBS,iBoundryFlag);
			break;
	}
}

void PlainH264Decoder::FillRecNeededMbInfo(SDecoderContext* pCtx,SDqLayer* layer){
	SPicture* pCurPic=pCtx->pDec;
	int32_t iLumaStride=pCurPic->iLinesize[0];
	int32_t iChromaStride=pCurPic->iLinesize[1];
	int32_t iMbX=layer->iMbX;
	int32_t iMbY=layer->iMbY;
	layer->iLumaStride=iLumaStride;
	layer->iChromaStride=iChromaStride;
	layer->pPred[0]=pCurPic->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
	layer->pPred[1]=pCurPic->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
	layer->pPred[2]=pCurPic->pData[2]+((iMbY*iChromaStride+iMbX)<<3);
}

// NOTE::: p_RS should NOT be modified and it will lead to mismatch with JSVM.
// so should allocate kA array to store the temporary value (idct).
void IdctResAddPred_c(uint8_t* pPred,const int32_t kiStride,int16_t* pRs){
	int16_t iSrc[16];
	uint8_t* pDst=pPred;
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	int32_t i;
	for(i=0; i<4; i++){
		const int32_t kiY=i<<2;
		const int32_t kiT0=pRs[kiY]+pRs[kiY+2];
		const int32_t kiT1=pRs[kiY]-pRs[kiY+2];
		const int32_t kiT2=(pRs[kiY+1]>>1)-pRs[kiY+3];
		const int32_t kiT3=pRs[kiY+1]+(pRs[kiY+3]>>1);
		iSrc[kiY]=kiT0+kiT3;
		iSrc[kiY+1]=kiT1+kiT2;
		iSrc[kiY+2]=kiT1-kiT2;
		iSrc[kiY+3]=kiT0-kiT3;
	}
	for(i=0; i<4; i++){
		int32_t kT1=iSrc[i]+iSrc[i+8];
		int32_t kT2=iSrc[i+4]+(iSrc[i+12]>>1);
		int32_t kT3=(32+kT1+kT2)>>6;
		int32_t kT4=(32+kT1-kT2)>>6;
		pDst[i]=Clip1(kT3+pPred[i]);
		pDst[i+kiStride3]=Clip1(kT4+pPred[i+kiStride3]);
		kT1=iSrc[i]-iSrc[i+8];
		kT2=(iSrc[i+4]>>1)-iSrc[i+12];
		pDst[i+kiStride]=Clip1(((32+kT1+kT2)>>6)+pDst[i+kiStride]);
		pDst[i+kiStride2]=Clip1(((32+kT1-kT2)>>6)+pDst[i+kiStride2]);
	}
}

void IdctFourResAddPred(uint8_t* pPred,int32_t iStride,int16_t* pRs,const int8_t* pNzc){
	if(pNzc[0] || pRs[0*16])
		IdctResAddPred_c(pPred+0*iStride+0,iStride,pRs+0*16);
	if(pNzc[1] || pRs[1*16])
		IdctResAddPred_c(pPred+0*iStride+4,iStride,pRs+1*16);
	if(pNzc[4] || pRs[2*16])
		IdctResAddPred_c(pPred+4*iStride+0,iStride,pRs+2*16);
	if(pNzc[5] || pRs[3*16])
		IdctResAddPred_c(pPred+4*iStride+4,iStride,pRs+3*16);
}

void PlainH264Decoder::RecChroma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer){
	int32_t iChromaStride=pCtx->m_layer.pDec->iLinesize[1];
	uint8_t i=0;
	uint8_t uiCbpC=pDqLayer->pCbp[iMBXY]>>4;
	if(1==uiCbpC || 2==uiCbpC){
		for(i=0; i<2; i++){
			int16_t* pRS=pScoeffLevel+256+(i<<6);
			uint8_t* pPred=pDqLayer->pPred[i+1];
			const int8_t* pNzc=pDqLayer->pNzc[iMBXY]+16+2*i;
			// 1 chroma is divided 4 4x4_block to idct
			IdctFourResAddPred(pPred,iChromaStride,pRS,pNzc);
		}
	}
}

void PlainH264Decoder::RecI16x16Mb(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer){
	// decoder use,encoder no use
	int8_t iI16x16PredMode=pDqLayer->pIntraPredMode[iMBXY][7];
	int8_t iChromaPredMode=pDqLayer->pChromaPredMode[iMBXY];
	PGetIntraPredFunc* pGetIChromaPredFunc=pCtx->pGetIChromaPredFunc;
	PGetIntraPredFunc* pGetI16x16LumaPredFunc=pCtx->pGetI16x16LumaPredFunc;
	int32_t iUVStride=pCtx->m_layer.pDec->iLinesize[1];
	// common use by decoder&encoder
	int32_t iYStride=pDqLayer->iLumaStride;
	int16_t* pRS=pScoeffLevel;
	uint8_t* pPred=pDqLayer->pPred[0];
	// decode i16x16 y
	pGetI16x16LumaPredFunc[iI16x16PredMode](pPred,iYStride);
	// 1 mb is divided 16 4x4_block to idct
	const int8_t* pNzc=pDqLayer->pNzc[iMBXY];
	IdctFourResAddPred(pPred+0*iYStride+0,iYStride,pRS+0*64,pNzc+0);
	IdctFourResAddPred(pPred+0*iYStride+8,iYStride,pRS+1*64,pNzc+2);
	IdctFourResAddPred(pPred+8*iYStride+0,iYStride,pRS+2*64,pNzc+8);
	IdctFourResAddPred(pPred+8*iYStride+8,iYStride,pRS+3*64,pNzc+10);
	// decode intra mb cb&cr
	pPred=pDqLayer->pPred[1];
	pGetIChromaPredFunc[iChromaPredMode](pPred,iUVStride);
	pPred=pDqLayer->pPred[2];
	pGetIChromaPredFunc[iChromaPredMode](pPred,iUVStride);
	RecChroma(iMBXY,pCtx,pScoeffLevel,pDqLayer);
}


void IdctResAddPred8x8_c(uint8_t* pPred,const int32_t kiStride,int16_t* pRs){
	// To make the ASM code easy to write,should using one funciton to apply hor and ver together,such as we did on HEVC
	// Ugly code,just for easy debug,the final version need optimization
	int16_t p[8],b[8];
	int16_t a[4];
	int16_t iTmp[64];
	int16_t iRes[64];
	// Horizontal
	for(int i=0; i<8; i++){
		for(int j=0; j<8; j++){
			p[j]=pRs[j+(i<<3)];
		}
		a[0]=p[0]+p[4];
		a[1]=p[0]-p[4];
		a[2]=p[6]-(p[2]>>1);
		a[3]=p[2]+(p[6]>>1);
		b[0]=a[0]+a[3];
		b[2]=a[1]-a[2];
		b[4]=a[1]+a[2];
		b[6]=a[0]-a[3];
		a[0]=-p[3]+p[5]-p[7]-(p[7]>>1);
		a[1]=p[1]+p[7]-p[3]-(p[3]>>1);
		a[2]=-p[1]+p[7]+p[5]+(p[5]>>1);
		a[3]=p[3]+p[5]+p[1]+(p[1]>>1);
		b[1]=a[0]+(a[3]>>2);
		b[3]=a[1]+(a[2]>>2);
		b[5]=a[2]-(a[1]>>2);
		b[7]=a[3]-(a[0]>>2);
		iTmp[0+(i<<3)]=b[0]+b[7];
		iTmp[1+(i<<3)]=b[2]-b[5];
		iTmp[2+(i<<3)]=b[4]+b[3];
		iTmp[3+(i<<3)]=b[6]+b[1];
		iTmp[4+(i<<3)]=b[6]-b[1];
		iTmp[5+(i<<3)]=b[4]-b[3];
		iTmp[6+(i<<3)]=b[2]+b[5];
		iTmp[7+(i<<3)]=b[0]-b[7];
	}
	// Vertical
	for(int i=0; i<8; i++){
		for(int j=0; j<8; j++){
			p[j]=iTmp[i+(j<<3)];
		}
		a[0]=p[0]+p[4];
		a[1]=p[0]-p[4];
		a[2]=p[6]-(p[2]>>1);
		a[3]=p[2]+(p[6]>>1);
		b[0]=a[0]+a[3];
		b[2]=a[1]-a[2];
		b[4]=a[1]+a[2];
		b[6]=a[0]-a[3];
		a[0]=-p[3]+p[5]-p[7]-(p[7]>>1);
		a[1]=p[1]+p[7]-p[3]-(p[3]>>1);
		a[2]=-p[1]+p[7]+p[5]+(p[5]>>1);
		a[3]=p[3]+p[5]+p[1]+(p[1]>>1);
		b[1]=a[0]+(a[3]>>2);
		b[7]=a[3]-(a[0]>>2);
		b[3]=a[1]+(a[2]>>2);
		b[5]=a[2]-(a[1]>>2);
		iRes[(0<<3)+i]=b[0]+b[7];
		iRes[(1<<3)+i]=b[2]-b[5];
		iRes[(2<<3)+i]=b[4]+b[3];
		iRes[(3<<3)+i]=b[6]+b[1];
		iRes[(4<<3)+i]=b[6]-b[1];
		iRes[(5<<3)+i]=b[4]-b[3];
		iRes[(6<<3)+i]=b[2]+b[5];
		iRes[(7<<3)+i]=b[0]-b[7];
	}
	uint8_t* pDst=pPred;
	for(int i=0; i<8; i++){
		for(int j=0; j<8; j++){
			pDst[i*kiStride+j]=Clip1(((32+iRes[(i<<3)+j])>>6)+pDst[i*kiStride+j]);
		}
	}
}

void PlainH264Decoder::RecI8x8Luma(int32_t iMbXy,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer){
	// prediction info
	uint8_t* pPred=pDqLayer->pPred[0];
	int32_t iLumaStride=pDqLayer->iLumaStride;
	int32_t* pBlockOffset=pCtx->iDecBlockOffsetArray;
	PGetIntraPred8x8Func* pGetI8x8LumaPredFunc=pCtx->pGetI8x8LumaPredFunc;
	int8_t* pIntra8x8PredMode=pDqLayer->pIntra4x4FinalMode[iMbXy];		// I_NxN
	int16_t* pRS=pScoeffLevel;
	// itransform info
	uint8_t i=0;
	bool bTLAvail[4],bTRAvail[4];
	// Top-Right : Left : Top-Left : Top
	bTLAvail[0]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x02);
	bTLAvail[1]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x01);
	bTLAvail[2]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x04);
	bTLAvail[3]=true;
	bTRAvail[0]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x01);
	bTRAvail[1]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x08);
	bTRAvail[2]=true;
	bTRAvail[3]=false;
	for(i=0; i<4; i++){
		uint8_t* pPredI8x8=pPred+pBlockOffset[i<<2];
		uint8_t uiMode=pIntra8x8PredMode[g_kuiScan4[i<<2]];
		pGetI8x8LumaPredFunc[uiMode](pPredI8x8,iLumaStride,bTLAvail[i],bTRAvail[i]);
		int32_t iIndex=g_kuiMbCountScan4Idx[i<<2];
		if(pDqLayer->pNzc[iMbXy][iIndex] || pDqLayer->pNzc[iMbXy][iIndex+1] || pDqLayer->pNzc[iMbXy][iIndex+4] || pDqLayer->pNzc[iMbXy][iIndex+5]){
			int16_t* pRSI8x8=&pRS[i<<6];
			IdctResAddPred8x8_c(pPredI8x8,iLumaStride,pRSI8x8);
		}
	}
}
int32_t PlainH264Decoder::RecI4x4Chroma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer){
	int32_t iChromaStride=pCtx->m_layer.pDec->iLinesize[1];
	int8_t iChromaPredMode=pDqLayer->pChromaPredMode[iMBXY];
	PGetIntraPredFunc* pGetIChromaPredFunc=pCtx->pGetIChromaPredFunc;
	uint8_t* pPred=pDqLayer->pPred[1];
	pGetIChromaPredFunc[iChromaPredMode](pPred,iChromaStride);
	pPred=pDqLayer->pPred[2];
	pGetIChromaPredFunc[iChromaPredMode](pPred,iChromaStride);
	RecChroma(iMBXY,pCtx,pScoeffLevel,pDqLayer);
	return ERR_NONE;
}
int32_t PlainH264Decoder::RecI8x8Mb(int32_t iMbXy,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer){
	RecI8x8Luma(iMbXy,pCtx,pScoeffLevel,pDqLayer);
	RecI4x4Chroma(iMbXy,pCtx,pScoeffLevel,pDqLayer);
	return ERR_NONE;
}

int32_t PlainH264Decoder::RecI4x4Luma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer){
	// prediction info
	uint8_t* pPred=pDqLayer->pPred[0];
	int32_t iLumaStride=pDqLayer->iLumaStride;
	int32_t* pBlockOffset=pCtx->iDecBlockOffsetArray;
	PGetIntraPredFunc* pGetI4x4LumaPredFunc=pCtx->pGetI4x4LumaPredFunc;
	int8_t* pIntra4x4PredMode=pDqLayer->pIntra4x4FinalMode[iMBXY];
	int16_t* pRS=pScoeffLevel;
	// itransform info
	uint8_t i=0;
	for(i=0; i<16; i++){
		uint8_t* pPredI4x4=pPred+pBlockOffset[i];
		uint8_t uiMode=pIntra4x4PredMode[g_kuiScan4[i]];
		pGetI4x4LumaPredFunc[uiMode](pPredI4x4,iLumaStride);
		if(pDqLayer->pNzc[iMBXY][g_kuiMbCountScan4Idx[i]]){
			int16_t* pRSI4x4=&pRS[i<<4];
			IdctResAddPred_c(pPredI4x4,iLumaStride,pRSI4x4);
		}
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::RecI4x4Mb(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,SDqLayer* pDqLayer){
	RecI4x4Luma(iMBXY,pCtx,pScoeffLevel,pDqLayer);
	RecI4x4Chroma(iMBXY,pCtx,pScoeffLevel,pDqLayer);
	return ERR_NONE;
}

int32_t PlainH264Decoder::MbIntraPredictionConstruction(SDecoderContext* pCtx,SDqLayer* layer){
	int32_t iMbXy=layer->iMbXyIndex;
	FillRecNeededMbInfo(pCtx,layer);
	if(IS_INTRA16x16(layer->pDec->pMbType[iMbXy])){
		RecI16x16Mb(iMbXy,pCtx,layer->pScaledTCoeff[iMbXy],layer);
	}else
	if(IS_INTRA8x8(layer->pDec->pMbType[iMbXy])){
		RecI8x8Mb(iMbXy,pCtx,layer->pScaledTCoeff[iMbXy],layer);
	}else
	if(IS_INTRA4x4(layer->pDec->pMbType[iMbXy])){
		RecI4x4Mb(iMbXy,pCtx,layer->pScaledTCoeff[iMbXy],layer);
	}
	return ERR_NONE;
}

#define WELS_B_MB_REC_VERIFY(uiRet) do{ \
	uint32_t uiRetTmp=(uint32_t)uiRet; \
	if(uiRetTmp!=ERR_NONE) \
		return uiRetTmp; \
}while(0)

// according to current 8*8 block ref_index to gain reference picture
static inline int32_t GetRefPic(sMCRefMember* pMCRefMem,SDecoderContext* pCtx,const int8_t& iRefIdx,int32_t listIdx){
	SPicture* pRefPic=pCtx->pRef;
	pMCRefMem->iSrcLineLuma=pRefPic->iLinesize[0];
	pMCRefMem->iSrcLineChroma=pRefPic->iLinesize[1];
	pMCRefMem->pSrcY=pRefPic->pData[0];
	pMCRefMem->pSrcU=pRefPic->pData[1];
	pMCRefMem->pSrcV=pRefPic->pData[2];
	if(!pMCRefMem->pSrcY || !pMCRefMem->pSrcU || !pMCRefMem->pSrcV){
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_DATA,ERR_INFO_REFERENCE_PIC_LOST);
	}
	return ERR_NONE;
}

// iA=(8-dx) * (8-dy);
// iB=dx * (8-dy);
// iC=(8-dx) * dy;
// iD=dx * dy
static const uint8_t g_kuiABCD[8][8][4]={		// g_kA[dy][dx],g_kB[dy][dx],g_kC[dy][dx],g_kD[dy][dx]
	{{64,0,0,0},{56,8,0,0},{48,16,0,0},{40,24,0,0},		{32,32,0,0},{24,40,0,0},{16,48,0,0},{8,56,0,0}},
	{{56,0,8,0},{49,7,7,1},{42,14,6,2},{35,21,5,3},		{28,28,4,4},{21,35,3,5},{14,42,2,6},{7,49,1,7}},
	{{48,0,16,0},{42,6,14,2},{36,12,12,4},{30,18,10,6},	{24,24,8,8},{18,30,6,10},{12,36,4,12},{6,42,2,14}},
	{{40,0,24,0},{35,5,21,3},{30,10,18,6},{25,15,15,9},	{20,20,12,12},{15,25,9,15},{10,30,6,18},{5,35,3,21}},
	{{32,0,32,0},{28,4,28,4},{24,8,24,8},{20,12,20,12},	{16,16,16,16},{12,20,12,20},{8,24,8,24},{4,28,4,28}},
	{{24,0,40,0},{21,3,35,5},{18,6,30,10},{15,9,25,15},	{12,12,20,20},{9,15,15,25},{6,18,10,30},{3,21,5,35}},
	{{16,0,48,0},{14,2,42,6},{12,4,36,12},{10,6,30,18},	{8,8,24,24},{6,10,18,30},{4,12,12,36},{2,14,6,42}},
	{{8,0,56,0},{7,1,49,7},{6,2,42,14},{5,3,35,21},		{4,4,28,28},{3,5,21,35},{2,6,14,42},{1,7,7,49}}
};

static inline void McChromaWithFragMv_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	int32_t iA,iB,iC,iD;
	const uint8_t* pSrcNext=pSrc+iSrcStride;
	const uint8_t* pABCD=g_kuiABCD[iMvY&0x07][iMvX&0x07];
	iA=pABCD[0];
	iB=pABCD[1];
	iC=pABCD[2];
	iD=pABCD[3];
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=(iA*pSrc[j]+iB*pSrc[j+1]+iC*pSrcNext[j]+iD*pSrcNext[j+1]+32)>>6;
		}
		pDst+=iDstStride;
		pSrc=pSrcNext;
		pSrcNext+=iSrcStride;
	}
}

static inline void McCopyWidthEq16_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){
		ST64A8(pDst,LD64(pSrc));
		ST64A8(pDst+8,LD64(pSrc+8));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq8_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){
		ST64A8(pDst,LD64(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq4_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){
		ST32A4(pDst,LD32(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq2_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){		// iWidth==2 only for chroma
		ST16A2(pDst,LD16(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopy_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	if(iWidth==16)
		McCopyWidthEq16_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else if(iWidth==8)
		McCopyWidthEq8_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else if(iWidth==4)
		McCopyWidthEq4_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else		// here iWidth==2
		McCopyWidthEq2_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
}

void McChroma_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight) {	// pSrc has been added the offset of mv
	const int32_t kiD8x=iMvX&0x07;
	const int32_t kiD8y=iMvY&0x07;
	if(0==kiD8x && 0==kiD8y)
		McCopy_c(pSrc,iSrcStride,pDst,iDstStride,iWidth,iHeight);
	else
		McChromaWithFragMv_c(pSrc,iSrcStride,pDst,iDstStride,iMvX,iMvY,iWidth,iHeight);
}

// h: iOffset=1 / v: iOffset=iSrcStride
static inline int32_t FilterInput8bitWithStride_c(const uint8_t* pSrc,const int32_t kiOffset){
	const int32_t kiOffset1=kiOffset;
	const int32_t kiOffset2=(kiOffset<<1);
	const int32_t kiOffset3=kiOffset+kiOffset2;
	const uint32_t kuiPix05=*(pSrc-kiOffset2)+*(pSrc+kiOffset3);
	const uint32_t kuiPix14=*(pSrc-kiOffset1)+*(pSrc+kiOffset2);
	const uint32_t kuiPix23=*(pSrc)+*(pSrc+kiOffset1);
	return (kuiPix05-((kuiPix14<<2)+kuiPix14)+(kuiPix23<<4)+(kuiPix23<<2));
}

// horizontal filter to gain half sample,that is (2,0) location in quarter sample
static inline void McHorVer20_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=Clip1((FilterInput8bitWithStride_c(pSrc+j,1)+16)>>5);
		}
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

// vertical filter to gain half sample,that is (0,2) location in quarter sample
static inline void McHorVer02_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=Clip1((FilterInput8bitWithStride_c(pSrc+j,iSrcStride)+16)>>5);
		}
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void PixelAvg_c(uint8_t* pDst,int32_t iDstStride,const uint8_t* pSrcA,int32_t iSrcAStride,const uint8_t* pSrcB,int32_t iSrcBStride,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=(pSrcA[j]+pSrcB[j]+1)>>1;
		}
		pDst+=iDstStride;
		pSrcA+=iSrcAStride;
		pSrcB+=iSrcBStride;
	}
}

static inline int32_t HorFilterInput16bit_c(const int16_t* pSrc){
	int32_t iPix05=pSrc[0]+pSrc[5];
	int32_t iPix14=pSrc[1]+pSrc[4];
	int32_t iPix23=pSrc[2]+pSrc[3];
	return (iPix05-(iPix14*5)+(iPix23*20));
}

// horizontal and vertical filter to gain half sample,that is (2,2) location in quarter sample
static inline void McHorVer22_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	int16_t iTmp[17+5];
	int32_t i,j,k;

	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth+5; j++){
			iTmp[j]=FilterInput8bitWithStride_c(pSrc-2+j,iSrcStride);
		}
		for(k=0; k<iWidth; k++){
			pDst[k]=Clip1((HorFilterInput16bit_c(&iTmp[k])+512)>>10);
		}
		pSrc+=iSrcStride;
		pDst+=iDstStride;
	}
}

//luma MC
static inline void McHorVer01_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer03_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc+iSrcStride,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer10_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer11_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer12_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiVerTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiVerTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer13_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer21_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer23_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer30_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc+1,iSrcStride,uiHorTmp,16,iWidth,iHeight);
}
static inline void McHorVer31_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer32_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiVerTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiVerTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer33_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}

typedef void (*PMcWidthHeightFunc) (const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight);

void McLuma_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight){	// pSrc has been added the offset of mv
	static const PMcWidthHeightFunc pMcFunc[4][4]={		// [x][y]
		{McCopy_c,McHorVer01_c,McHorVer02_c,McHorVer03_c},
		{McHorVer10_c,McHorVer11_c,McHorVer12_c,McHorVer13_c},
		{McHorVer20_c,McHorVer21_c,McHorVer22_c,McHorVer23_c},
		{McHorVer30_c,McHorVer31_c,McHorVer32_c,McHorVer33_c},
	};
	pMcFunc[iMvX&0x03][iMvY&0x03](pSrc,iSrcStride,pDst,iDstStride,iWidth,iHeight);
}

void PlainH264Decoder::BaseMC(SDecoderContext* pCtx,sMCRefMember* pMCRefMem,const int32_t& listIdx,const int8_t& iRefIdx,int32_t iXOffset,int32_t iYOffset,int32_t iBlkWidth,int32_t iBlkHeight,int16_t iMVs[2]){
	int32_t iFullMVx=(iXOffset<<2)+iMVs[0];		// quarter pixel
	int32_t iFullMVy=(iYOffset<<2)+iMVs[1];
	iFullMVx=WELS_CLIP3(iFullMVx,((-PADDING_LENGTH+2)*(1<<2)),((m_videoWidth+PADDING_LENGTH-19)*(1<<2)));
	iFullMVy=WELS_CLIP3(iFullMVy,((-PADDING_LENGTH+2)*(1<<2)),((m_videoHeight+PADDING_LENGTH-19)*(1<<2)));

	int32_t iSrcPixOffsetLuma=(iFullMVx>>2)+(iFullMVy>>2)*pMCRefMem->iSrcLineLuma;
	int32_t iSrcPixOffsetChroma=(iFullMVx>>3)+(iFullMVy>>3)*pMCRefMem->iSrcLineChroma;

	int32_t iBlkWidthChroma=iBlkWidth>>1;
	int32_t iBlkHeightChroma=iBlkHeight>>1;

	uint8_t* pSrcY=pMCRefMem->pSrcY+iSrcPixOffsetLuma;
	uint8_t* pSrcU=pMCRefMem->pSrcU+iSrcPixOffsetChroma;
	uint8_t* pSrcV=pMCRefMem->pSrcV+iSrcPixOffsetChroma;
	uint8_t* pDstY=pMCRefMem->pDstY;
	uint8_t* pDstU=pMCRefMem->pDstU;
	uint8_t* pDstV=pMCRefMem->pDstV;

	McLuma_c(pSrcY,pMCRefMem->iSrcLineLuma,pDstY,pMCRefMem->iDstLineLuma,iFullMVx,iFullMVy,iBlkWidth,iBlkHeight);
	McChroma_c(pSrcU,pMCRefMem->iSrcLineChroma,pDstU,pMCRefMem->iDstLineChroma,iFullMVx,iFullMVy,iBlkWidthChroma,iBlkHeightChroma);
	McChroma_c(pSrcV,pMCRefMem->iSrcLineChroma,pDstV,pMCRefMem->iDstLineChroma,iFullMVx,iFullMVy,iBlkWidthChroma,iBlkHeightChroma);
}

int32_t PlainH264Decoder::GetInterPred(uint8_t* pPredY,uint8_t* pPredCb,uint8_t* pPredCr,SDecoderContext* pCtx){
	sMCRefMember pMCRefMem;
	SDqLayer* layer=&pCtx->m_layer;
	int32_t iMBXY=layer->iMbXyIndex;
	int16_t iMVs[2]={0};
	uint32_t iMBType=layer->pDec->pMbType[iMBXY];
	int32_t iMBOffsetX=layer->iMbX<<4;
	int32_t iMBOffsetY=layer->iMbY<<4;
	int32_t iDstLineLuma=pCtx->pDec->iLinesize[0];
	int32_t iDstLineChroma=pCtx->pDec->iLinesize[1];
	int32_t iBlk8X,iBlk8Y,iBlk4X,iBlk4Y,i,j,iIIdx,iJIdx;
	pMCRefMem.pDstY=pPredY;
	pMCRefMem.pDstU=pPredCb;
	pMCRefMem.pDstV=pPredCr;
	pMCRefMem.iDstLineLuma=iDstLineLuma;
	pMCRefMem.iDstLineChroma=iDstLineChroma;
	int8_t iRefIndex=0;
	switch(iMBType){
		case MB_TYPE_SKIP:
		case MB_TYPE_16x16:
			iMVs[0]=layer->pDec->pMv[0][iMBXY][0][0];
			iMVs[1]=layer->pDec->pMv[0][iMBXY][0][1];
			iRefIndex=layer->pDec->pRefIndex[0][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY,16,16,iMVs);
			break;
		case MB_TYPE_16x8:
			iMVs[0]=layer->pDec->pMv[0][iMBXY][0][0];
			iMVs[1]=layer->pDec->pMv[0][iMBXY][0][1];
			iRefIndex=layer->pDec->pRefIndex[0][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY,16,8,iMVs);
			iMVs[0]=layer->pDec->pMv[0][iMBXY][8][0];
			iMVs[1]=layer->pDec->pMv[0][iMBXY][8][1];
			iRefIndex=layer->pDec->pRefIndex[0][iMBXY][8];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			pMCRefMem.pDstY=pPredY+(iDstLineLuma<<3);
			pMCRefMem.pDstU=pPredCb+(iDstLineChroma<<2);
			pMCRefMem.pDstV=pPredCr+(iDstLineChroma<<2);
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY+8,16,8,iMVs);
			break;
		case MB_TYPE_8x16:
			iMVs[0]=layer->pDec->pMv[0][iMBXY][0][0];
			iMVs[1]=layer->pDec->pMv[0][iMBXY][0][1];
			iRefIndex=layer->pDec->pRefIndex[0][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY,8,16,iMVs);
			iMVs[0]=layer->pDec->pMv[0][iMBXY][2][0];
			iMVs[1]=layer->pDec->pMv[0][iMBXY][2][1];
			iRefIndex=layer->pDec->pRefIndex[0][iMBXY][2];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			pMCRefMem.pDstY=pPredY+8;
			pMCRefMem.pDstU=pPredCb+4;
			pMCRefMem.pDstV=pPredCr+4;
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX+8,iMBOffsetY,8,16,iMVs);
			break;
		case MB_TYPE_8x8:
		case MB_TYPE_8x8_REF0:
		{
			uint32_t iSubMBType;
			int32_t iXOffset,iYOffset;
			uint8_t* pDstY,* pDstU,* pDstV;
			for(i=0; i<4; i++){
				iSubMBType=layer->pSubMbType[iMBXY][i];
				iBlk8X=(i&1)<<3;
				iBlk8Y=(i>>1)<<3;
				iXOffset=iMBOffsetX+iBlk8X;
				iYOffset=iMBOffsetY+iBlk8Y;

				iIIdx=((i>>1)<<3)+((i&1)<<1);
				iRefIndex=layer->pDec->pRefIndex[0][iMBXY][iIIdx];
				WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
				pDstY=pPredY+iBlk8X+iBlk8Y*iDstLineLuma;
				pDstU=pPredCb+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;
				pDstV=pPredCr+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;
				pMCRefMem.pDstY=pDstY;
				pMCRefMem.pDstU=pDstU;
				pMCRefMem.pDstV=pDstV;
				switch(iSubMBType){
					case SUB_MB_TYPE_8x8:
						iMVs[0]=layer->pDec->pMv[0][iMBXY][iIIdx][0];
						iMVs[1]=layer->pDec->pMv[0][iMBXY][iIIdx][1];
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset,8,8,iMVs);
						break;
					case SUB_MB_TYPE_8x4:
						iMVs[0]=layer->pDec->pMv[0][iMBXY][iIIdx][0];
						iMVs[1]=layer->pDec->pMv[0][iMBXY][iIIdx][1];
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset,8,4,iMVs);
						iMVs[0]=layer->pDec->pMv[0][iMBXY][iIIdx+4][0];
						iMVs[1]=layer->pDec->pMv[0][iMBXY][iIIdx+4][1];
						pMCRefMem.pDstY+=(iDstLineLuma<<2);
						pMCRefMem.pDstU+=(iDstLineChroma<<1);
						pMCRefMem.pDstV+=(iDstLineChroma<<1);
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset+4,8,4,iMVs);
						break;
					case SUB_MB_TYPE_4x8:
						iMVs[0]=layer->pDec->pMv[0][iMBXY][iIIdx][0];
						iMVs[1]=layer->pDec->pMv[0][iMBXY][iIIdx][1];
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset,4,8,iMVs);
						iMVs[0]=layer->pDec->pMv[0][iMBXY][iIIdx+1][0];
						iMVs[1]=layer->pDec->pMv[0][iMBXY][iIIdx+1][1];
						pMCRefMem.pDstY+=4;
						pMCRefMem.pDstU+=2;
						pMCRefMem.pDstV+=2;
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset+4,iYOffset,4,8,iMVs);
						break;
					case SUB_MB_TYPE_4x4:
					{
						for(j=0; j<4; j++){
							int32_t iUVLineStride;
							iJIdx=((j>>1)<<2)+(j&1);
							iBlk4X=(j&1)<<2;
							iBlk4Y=(j>>1)<<2;
							iUVLineStride=(iBlk4X>>1)+(iBlk4Y>>1)*iDstLineChroma;
							pMCRefMem.pDstY=pDstY+iBlk4X+iBlk4Y*iDstLineLuma;
							pMCRefMem.pDstU=pDstU+iUVLineStride;
							pMCRefMem.pDstV=pDstV+iUVLineStride;
							iMVs[0]=layer->pDec->pMv[0][iMBXY][iIIdx+iJIdx][0];
							iMVs[1]=layer->pDec->pMv[0][iMBXY][iIIdx+iJIdx][1];
							BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset+iBlk4X,iYOffset+iBlk4Y,4,4,iMVs);
						}
					}
					break;
					default:
						break;
				}
			}
		}
		break;
		default:
			break;
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::MbInterPrediction(SDecoderContext* pCtx,SDqLayer* layer){
	int32_t iMbX=layer->iMbX;
	int32_t iMbY=layer->iMbY;
	uint8_t* pDstY,* pDstCb,* pDstCr;
	int32_t iLumaStride=pCtx->pDec->iLinesize[0];
	int32_t iChromaStride=pCtx->pDec->iLinesize[1];
	pDstY=layer->pDec->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
	pDstCb=layer->pDec->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
	pDstCr=layer->pDec->pData[2]+((iMbY*iChromaStride+iMbX)<<3);
	if(pCtx->eSliceType==P_SLICE){
		WELS_B_MB_REC_VERIFY(GetInterPred(pDstY,pDstCb,pDstCr,pCtx));
	}else{
		FATAL("CODE REMOVED");
	}
	return ERR_NONE;
}

int32_t PlainH264Decoder::MbInterSampleConstruction(SDecoderContext* pCtx,SDqLayer* layer,uint8_t* pDstY,uint8_t* pDstU,uint8_t* pDstV,int32_t iStrideL,int32_t iStrideC){
	int32_t iMbXy=layer->iMbXyIndex;
	int32_t i,iIndex,iOffset;

	if(layer->pTransformSize8x8Flag[iMbXy]){
		for(i=0; i<4; i++){
			iIndex=g_kuiMbCountScan4Idx[i<<2];
			if(layer->pNzc[iMbXy][iIndex] || layer->pNzc[iMbXy][iIndex+1] || layer->pNzc[iMbXy][iIndex+4]
				 || layer->pNzc[iMbXy][iIndex+5]){
				iOffset=((iIndex>>2)<<2)*iStrideL+((iIndex%4)<<2);
				IdctResAddPred8x8_c(pDstY+iOffset,iStrideL,layer->pScaledTCoeff[iMbXy]+(i<<6));
			}
		}
	}else{
		// luma.
		const int8_t* pNzc=layer->pNzc[iMbXy];
		int16_t* pScaledTCoeff=layer->pScaledTCoeff[iMbXy];
		IdctFourResAddPred(pDstY+0*iStrideL+0,iStrideL,pScaledTCoeff+0*64,pNzc+0);
		IdctFourResAddPred(pDstY+0*iStrideL+8,iStrideL,pScaledTCoeff+1*64,pNzc+2);
		IdctFourResAddPred(pDstY+8*iStrideL+0,iStrideL,pScaledTCoeff+2*64,pNzc+8);
		IdctFourResAddPred(pDstY+8*iStrideL+8,iStrideL,pScaledTCoeff+3*64,pNzc+10);
	}

	const int8_t* pNzc=layer->pNzc[iMbXy];
	int16_t* pScaledTCoeff=layer->pScaledTCoeff[iMbXy];
	// Cb.
	IdctFourResAddPred(pDstU,iStrideC,pScaledTCoeff+4*64,pNzc+16);
	// Cr.
	IdctFourResAddPred(pDstV,iStrideC,pScaledTCoeff+5*64,pNzc+18);

	return ERR_NONE;
}

void NonZeroCount_c(int8_t* pNonZeroCount){
	int32_t i;
	for(i=0;i<24;i++){
		pNonZeroCount[i]=!!pNonZeroCount[i];
	}
}

int32_t PlainH264Decoder::MbInterConstruction(SDecoderContext* pCtx,SDqLayer* layer){
	int32_t iMbX=layer->iMbX;
	int32_t iMbY=layer->iMbY;
	uint8_t* pDstY,* pDstCb,* pDstCr;
	int32_t iLumaStride=pCtx->pDec->iLinesize[0];
	int32_t iChromaStride=pCtx->pDec->iLinesize[1];
	pDstY=layer->pDec->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
	pDstCb=layer->pDec->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
	pDstCr=layer->pDec->pData[2]+((iMbY*iChromaStride+iMbX)<<3);
	if(pCtx->eSliceType==P_SLICE){
		WELS_B_MB_REC_VERIFY(GetInterPred(pDstY,pDstCb,pDstCr,pCtx));
	}else{
		FATAL("CODE REMOVED");
	}
	MbInterSampleConstruction(pCtx,layer,pDstY,pDstCb,pDstCr,iLumaStride,iChromaStride);
	NonZeroCount_c(layer->pNzc[layer->iMbXyIndex]);		// set all none-zero nzc to 1; dbk can be opti!
	return ERR_NONE;
}

void PlainH264Decoder::TargetSliceConstruction(SDecoderContext* pCtx){
	SDqLayer* layer=&pCtx->m_layer;
	for(int32_t y=0;y!=m_blockHeight;y++) {
		for(int32_t x=0;x!=m_blockWidth;x++) {
			layer->iMbX=x;
			layer->iMbY=y;
			layer->iMbXyIndex=y*m_blockWidth+x;
			if(MB_TYPE_INTRA_PCM==layer->pDec->pMbType[layer->iMbXyIndex]){
				FATAL("Not supported");
			}else
			if(IS_INTRA(layer->pDec->pMbType[layer->iMbXyIndex])){
				MbIntraPredictionConstruction(pCtx,layer);
			}else
			if(IS_INTER(layer->pDec->pMbType[layer->iMbXyIndex])){		// InterMB
				if(!layer->pCbp[layer->iMbXyIndex]){					// uiCbp==0 include SKIP
					MbInterPrediction(pCtx,layer);
				}else{
					MbInterConstruction(pCtx,layer);
				}
			}else{
				FATAL("TargetSliceConstruction MB(%d,%d) construction error. pCurSlice_type:%d",layer->iMbX,layer->iMbY,pCtx->eSliceType);
			}
		}
	}
	SDeblockingFilter pFilter;
	memset(&pFilter,0,sizeof(pFilter));
	pFilter.pCsData[0]=pCtx->pDec->pData[0];
	pFilter.pCsData[1]=pCtx->pDec->pData[1];
	pFilter.pCsData[2]=pCtx->pDec->pData[2];
	pFilter.iCsStride[0]=pCtx->pDec->iLinesize[0];
	pFilter.iCsStride[1]=pCtx->pDec->iLinesize[1];
	pFilter.iSliceAlphaC0Offset=pCtx->m_slice.iSliceAlphaC0Offset;
	pFilter.iSliceBetaOffset=pCtx->m_slice.iSliceBetaOffset;
	pFilter.pRefPics[0]=&pCtx->pRef;
	pFilter.pRefPics[1]=&pCtx->pDec;
	for(int32_t y=0;y!=m_blockHeight;y++) {
		for(int32_t x=0;x!=m_blockWidth;x++) {
			layer->iMbX=x;
			layer->iMbY=y;
			layer->iMbXyIndex=y*m_blockWidth+x;
			int32_t iBoundryFlag=(x>0 ? (1<<LEFT_FLAG_BIT):0)|(y>0 ? (1<<TOP_FLAG_BIT):0);
			DeblockingMb(pCtx,layer,&pFilter,iBoundryFlag);
		}
	}
}

int32_t PlainH264Decoder::DecodeSliceNal(SDecoderContext* pCtx,const uint8_t* payload,int32_t payloadBitsize,uint8_t** ppDst){
	SBitStringAux m_bitstream;
	DecInitBits(&m_bitstream,payload,payloadBitsize);
	int32_t iErr=ParseSliceHeaderSyntaxs(m_pCtx,&m_bitstream);
	if(iErr!=ERR_NONE){
		FATAL("RESET NAL!");
	}
	InitCurDqLayerData(pCtx,&pCtx->m_layer);
	pCtx->pDec=pCtx->m_pics[m_frameNumber&1];
	pCtx->pRef=pCtx->m_pics[m_frameNumber&1 ? 0:1];
	GetI4LumaIChromaAddrTable(pCtx->iDecBlockOffsetArray,pCtx->pDec->iLinesize[0],pCtx->pDec->iLinesize[1]);
	pCtx->pDec->eSliceType=pCtx->eSliceType;
	pCtx->m_slice.iLastMbQp=pCtx->m_slice.iSliceQp;
	InitDqLayerInfo(pCtx,&pCtx->m_layer,&pCtx->m_nalUnit,pCtx->pDec);
	START_TIMER(decodeBlocks,&m_profiler,"Decode MB",0x60ff20);
	SSlice* slice=&pCtx->m_slice;
	slice->iLastDeltaQp=0;
	SCabacDecEngine cabacDecEngine;
	cabacDecEngine.InitCabac(slice->iSliceQp,pCtx->eSliceType==I_SLICE ? 0:1);
	if(!cabacDecEngine.InitCabacDecEngineFromBS(&m_bitstream))
		FATAL("Init failed, bitstream incorrect");
	SDqLayer* layer=&pCtx->m_layer;
	for(int32_t y=0;y!=m_blockHeight;y++) {
		for(int32_t x=0;x!=m_blockWidth;x++) {
			layer->iMbX=x;
			layer->iMbY=y;
			layer->iMbXyIndex=y*m_blockWidth+x;
			if(pCtx->eSliceType==P_SLICE) {
				DecodeMbCabacPSlice(&cabacDecEngine,pCtx,&m_bitstream);
			}else{
				DecodeMbCabacISlice(&cabacDecEngine,pCtx,&m_bitstream);
			}
			if(cabacDecEngine.DecodeTerminateCabac()!=(layer->iMbXyIndex+1==m_blockHeight*m_blockWidth)){		// end of slice
				FATAL("Last MB sanity check failed");
			}
		}
	}
// CABAC decoding finished,changing to SBitStringAux
	cabacDecEngine.InitBsFromCabacDecEngine(&m_bitstream);
	END_TIMER(decodeBlocks,&m_profiler);
	START_TIMER(constructSlice,&m_profiler,"Construct Slice",0x2040ff);
	TargetSliceConstruction(pCtx);
	END_TIMER(constructSlice,&m_profiler);
	ppDst[0]=pCtx->pDec->pData[0];
	ppDst[1]=pCtx->pDec->pData[1];
	ppDst[2]=pCtx->pDec->pData[2];
	ExpandReferencingPicture(pCtx->pDec->pData,m_videoWidth,m_videoHeight,pCtx->pDec->iLinesize);
	return ERR_NONE;
}

// free memory dynamically allocated during decoder
void PlainH264Decoder::FreeDynamicMemory(SDecoderContext* pCtx){
	FreePicture(pCtx->m_pics[0]);
	FreePicture(pCtx->m_pics[1]);
}

void PlainH264Decoder::FreeStaticMemory(SDecoderContext* pCtx){
	if(pCtx->sRawData.pHead){
		Free(pCtx->sRawData.pHead);
	}
	pCtx->sRawData.pHead=NULL;
	pCtx->sRawData.pEnd=NULL;
	pCtx->sRawData.pCurPos=NULL;
}

void PlainH264Decoder::CloseDecoder(SDecoderContext* pCtx){
	FreeDynamicMemory(pCtx);
	FreeStaticMemory(pCtx);
}

void I16x16LumaPredV_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	const uint64_t kuiTop1=LD64A8(pPred-kiStride);
	const uint64_t kuiTop2=LD64A8(pPred-kiStride+8);
	uint8_t i=15;
	do{
		ST64A8(pPred+iTmp,kuiTop1);
		ST64A8(pPred+iTmp+8,kuiTop2);
		iTmp-=kiStride;
	} while(i-->0);
}

void I16x16LumaPredH_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	uint8_t i=15;
	do{
		const uint8_t kuiVal8=pPred[iTmp-1];
		const uint64_t kuiVal64=0x0101010101010101ULL*kuiVal8;
		ST64A8(pPred+iTmp,kuiVal64);
		ST64A8(pPred+iTmp+8,kuiVal64);
		iTmp-=kiStride;
	} while(i-->0);
}

#define I4x4_COUNT 4
#define I8x8_COUNT 8
#define I16x16_COUNT 16

void I16x16LumaPredDc_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t uiMean=0;
	// caculate the kMean value
	do{
		iSum+=pPred[-1+iTmp]+pPred[-kiStride+i];
		iTmp-=kiStride;
	} while(i-->0);
	uiMean=(16+iSum)>>5;
	iTmp=(kiStride<<4)-kiStride;
	i=15;
	do{
		memset(&pPred[iTmp],uiMean,I16x16_COUNT);
		iTmp-=kiStride;
	} while(i-->0);
}

void I16x16LumaPredPlane_c(uint8_t* pPred,const int32_t kiStride){
	int32_t a=0,b=0,c=0,H=0,V=0;
	int32_t i,j;
	uint8_t* pTop=&pPred[-kiStride];
	uint8_t* pLeft=&pPred[-1];
	for(i=0; i<8; i++){
		H+=(i+1)*(pTop[8+i]-pTop[6-i]);
		V+=(i+1)*(pLeft[(8+i)*kiStride]-pLeft[(6-i)*kiStride]);
	}
	a=(pLeft[15*kiStride]+pTop[15])<<4;
	b=(5*H+32)>>6;
	c=(5*V+32)>>6;
	for(i=0; i<16; i++){
		for(j=0; j<16; j++){
			int32_t iTmp=(a+b*(j-7)+c*(i-7)+16)>>5;
			iTmp=Clip1(iTmp);
			pPred[j]=iTmp;
		}
		pPred+=kiStride;
	}
}

void I16x16LumaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint64_t uiMean64=0;
	uint8_t uiMean=0;
	uint8_t i=15;
	// caculate the kMean value
	do{
		iSum+=pPred[-1+iTmp];
		iTmp-=kiStride;
	} while(i-->0);
	uiMean=(8+iSum)>>4;
	uiMean64=0x0101010101010101ULL*uiMean;
	iTmp=(kiStride<<4)-kiStride;
	i=15;
	do{
		ST64A8(pPred+iTmp,uiMean64);
		ST64A8(pPred+iTmp+8,uiMean64);
		iTmp-=kiStride;
	} while(i-->0);
}

void I16x16LumaPredDcTop_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t uiMean=0;
	// caculate the kMean value
	do{
		iSum+=pPred[-kiStride+i];
	} while(i-->0);
	uiMean=(8+iSum)>>4;
	i=15;
	do{
		memset(&pPred[iTmp],uiMean,I16x16_COUNT);
		iTmp-=kiStride;
	} while(i-->0);
}

void I16x16LumaPredDcNA_c(uint8_t* pPred,const int32_t kiStride){
	const uint64_t kuiDC64=0x8080808080808080ULL;
	int32_t iTmp=(kiStride<<4)-kiStride;
	uint8_t i=15;
	do{
		ST64A8(pPred+iTmp,kuiDC64);
		ST64A8(pPred+iTmp+8,kuiDC64);

		iTmp-=kiStride;
	} while(i-->0);
}

static inline void FillingPred8to16_c(uint8_t* pPred,uint8_t* pSrc){
	ST64(pPred,LD64(pSrc));
	ST64(pPred+8,LD64(pSrc));
}
static inline void FillingPred8x2to16_c(uint8_t* pPred,uint8_t* pSrc){
	ST64(pPred,LD64(pSrc));
	ST64(pPred+8,LD64(pSrc+8));
}
static inline void FillingPred1to16_c(uint8_t* pPred,const uint8_t kuiSrc){
	const uint8_t kuiSrc8[8]={kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc};
	ST64(pPred,LD64(kuiSrc8));
	ST64(pPred+8,LD64(kuiSrc8));
}

void IChromaPredDc_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiL1=kiStride-1;
	const int32_t kiL2=kiL1+kiStride;
	const int32_t kiL3=kiL2+kiStride;
	const int32_t kiL4=kiL3+kiStride;
	const int32_t kiL5=kiL4+kiStride;
	const int32_t kiL6=kiL5+kiStride;
	const int32_t kiL7=kiL6+kiStride;
	// caculate the kMean value
	const uint8_t kuiM1=(pPred[-kiStride]+pPred[1-kiStride]+pPred[2-kiStride]+pPred[3-kiStride]+pPred[-1]+pPred[kiL1]+pPred[kiL2]+pPred[kiL3]+4)>>3;
	const uint32_t kuiSum2=pPred[4-kiStride]+pPred[5-kiStride]+pPred[6-kiStride]+pPred[7-kiStride];
	const uint32_t kuiSum3=pPred[kiL4]+pPred[kiL5]+pPred[kiL6]+pPred[kiL7];
	const uint8_t kuiM2=(kuiSum2+2)>>2;
	const uint8_t kuiM3=(kuiSum3+2)>>2;
	const uint8_t kuiM4=(kuiSum2+kuiSum3+4)>>3;
	const uint8_t kuiMUP[8]={kuiM1,kuiM1,kuiM1,kuiM1,kuiM2,kuiM2,kuiM2,kuiM2};
	const uint8_t kuiMDown[8]={kuiM3,kuiM3,kuiM3,kuiM3,kuiM4,kuiM4,kuiM4,kuiM4};
	const uint64_t kuiUP64=LD64(kuiMUP);
	const uint64_t kuiDN64=LD64(kuiMDown);
	ST64A8(pPred,kuiUP64);
	ST64A8(pPred+kiL1+1,kuiUP64);
	ST64A8(pPred+kiL2+1,kuiUP64);
	ST64A8(pPred+kiL3+1,kuiUP64);
	ST64A8(pPred+kiL4+1,kuiDN64);
	ST64A8(pPred+kiL5+1,kuiDN64);
	ST64A8(pPred+kiL6+1,kuiDN64);
	ST64A8(pPred+kiL7+1,kuiDN64);
}

void I16x16LumaPredPlane_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iLTshift=0,iTopshift=0,iLeftshift=0,iTopSum=0,iLeftSum=0;
	int32_t i,j;
	uint8_t* pTop=&pRef[-kiStride];
	uint8_t* pLeft=&pRef[-1];
	int32_t iPredStride=16;
	for(i=0; i<8; i++){
		iTopSum+=(i+1)*(pTop[8+i]-pTop[6-i]);
		iLeftSum+=(i+1)*(pLeft[(8+i)*kiStride]-pLeft[(6-i)*kiStride]);
	}
	iLTshift=(pLeft[15*kiStride]+pTop[15])<<4;
	iTopshift=(5*iTopSum+32)>>6;
	iLeftshift=(5*iLeftSum+32)>>6;
	for(i=0; i<16; i++){
		for(j=0; j<16; j++){
			pPred[j]=Clip1((iLTshift+iTopshift*(j-7)+iLeftshift*(i-7)+16)>>5);
		}
		pPred+=iPredStride;
	}
}

void I16x16LumaPredDc_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iStridex15=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	// caculate the iMean value
	do{
		iSum+=pRef[-1+iStridex15]+pRef[-kiStride+i];
		iStridex15-=kiStride;
	} while(i-->0);
	iMean=(16+iSum)>>5;
	memset(pPred,iMean,256);
}


void I16x16LumaPredDcTop_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	// caculate the iMean value
	do{
		iSum+=pRef[-kiStride+i];
	} while(i-->0);
	iMean=(8+iSum)>>4;
	memset(pPred,iMean,256);
}

void I16x16LumaPredDcLeft_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iStridex15=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	// caculate the iMean value
	do{
		iSum+=pRef[-1+iStridex15];
		iStridex15-=kiStride;
	} while(i-->0);
	iMean=(8+iSum)>>4;
	memset(pPred,iMean,256);
}

void I16x16LumaPredDcNA_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	memset(pPred,0x80,256);
}

void I4x4LumaPredV_c(uint8_t* pPred,const int32_t kiStride){
	const uint32_t kuiVal=LD32A4(pPred-kiStride);
	ST32A4(pPred,kuiVal);
	ST32A4(pPred+kiStride,kuiVal);
	ST32A4(pPred+(kiStride<<1),kuiVal);
	ST32A4(pPred+(kiStride<<1)+kiStride,kuiVal);
}

void I4x4LumaPredH_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint32_t kuiL0=0x01010101U*pPred[-1];
	const uint32_t kuiL1=0x01010101U*pPred[-1+kiStride];
	const uint32_t kuiL2=0x01010101U*pPred[-1+kiStride2];
	const uint32_t kuiL3=0x01010101U*pPred[-1+kiStride3];
	ST32A4(pPred,kuiL0);
	ST32A4(pPred+kiStride,kuiL1);
	ST32A4(pPred+kiStride2,kuiL2);
	ST32A4(pPred+kiStride3,kuiL3);
}

void I4x4LumaPredDc_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint8_t kuiMean=(pPred[-1]+pPred[-1+kiStride]+pPred[-1+kiStride2]+pPred[-1+kiStride3]+pPred[-kiStride]+pPred[-kiStride+1]+pPred[-kiStride+2]+pPred[-kiStride+3]+4)>>3;
	const uint32_t kuiMean32=0x01010101U*kuiMean;
	ST32A4(pPred,kuiMean32);
	ST32A4(pPred+kiStride,kuiMean32);
	ST32A4(pPred+kiStride2,kuiMean32);
	ST32A4(pPred+kiStride3,kuiMean32);
}

void I4x4LumaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint8_t kuiMean=(pPred[-1]+pPred[-1+kiStride]+pPred[-1+kiStride2]+pPred[-1+kiStride3]+2)>>2;
	const uint32_t kuiMean32=0x01010101U*kuiMean;
	ST32A4(pPred,kuiMean32);
	ST32A4(pPred+kiStride,kuiMean32);
	ST32A4(pPred+kiStride2,kuiMean32);
	ST32A4(pPred+kiStride3,kuiMean32);
}

void I4x4LumaPredDcTop_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint8_t kuiMean=(pPred[-kiStride]+pPred[-kiStride+1]+pPred[-kiStride+2]+pPred[-kiStride+3]+2)>>2;
	const uint32_t kuiMean32=0x01010101U*kuiMean;
	ST32A4(pPred,kuiMean32);
	ST32A4(pPred+kiStride,kuiMean32);
	ST32A4(pPred+kiStride2,kuiMean32);
	ST32A4(pPred+kiStride3,kuiMean32);
}

void I4x4LumaPredDcNA_c(uint8_t* pPred,const int32_t kiStride){
	const uint32_t kuiDC32=0x80808080U;
	ST32A4(pPred,kuiDC32);
	ST32A4(pPred+kiStride,kuiDC32);
	ST32A4(pPred+(kiStride<<1),kuiDC32);
	ST32A4(pPred+(kiStride<<1)+kiStride,kuiDC32);
}

// down pLeft
void I4x4LumaPredDDL_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	// get pTop
	uint8_t* ptop=&pPred[-kiStride];
	const uint8_t kuiT0=*ptop;
	const uint8_t kuiT1=*(ptop+1);
	const uint8_t kuiT2=*(ptop+2);
	const uint8_t kuiT3=*(ptop+3);
	const uint8_t kuiT4=*(ptop+4);
	const uint8_t kuiT5=*(ptop+5);
	const uint8_t kuiT6=*(ptop+6);
	const uint8_t kuiT7=*(ptop+7);
	const uint8_t kuiDDL0=(2+kuiT0+kuiT2+(kuiT1<<1))>>2;		// kDDL0
	const uint8_t kuiDDL1=(2+kuiT1+kuiT3+(kuiT2<<1))>>2;		// kDDL1
	const uint8_t kuiDDL2=(2+kuiT2+kuiT4+(kuiT3<<1))>>2;		// kDDL2
	const uint8_t kuiDDL3=(2+kuiT3+kuiT5+(kuiT4<<1))>>2;		// kDDL3
	const uint8_t kuiDDL4=(2+kuiT4+kuiT6+(kuiT5<<1))>>2;		// kDDL4
	const uint8_t kuiDDL5=(2+kuiT5+kuiT7+(kuiT6<<1))>>2;		// kDDL5
	const uint8_t kuiDDL6=(2+kuiT6+kuiT7+(kuiT7<<1))>>2;		// kDDL6
	const uint8_t kuiList[8]={kuiDDL0,kuiDDL1,kuiDDL2,kuiDDL3,kuiDDL4,kuiDDL5,kuiDDL6,0};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+1));
	ST32A4(pPred+kiStride2,LD32(kuiList+2));
	ST32A4(pPred+kiStride3,LD32(kuiList+3));
}

// down pLeft
void I4x4LumaPredDDLTop_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	// get pTop
	uint8_t* ptop=&pPred[-kiStride];
	const uint8_t kuiT0=*ptop;
	const uint8_t kuiT1=*(ptop+1);
	const uint8_t kuiT2=*(ptop+2);
	const uint8_t kuiT3=*(ptop+3);
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiT33=1+(kuiT3<<1);
	const uint8_t kuiDLT0=(kuiT01+kuiT12)>>2;		// kDLT0
	const uint8_t kuiDLT1=(kuiT12+kuiT23)>>2;		// kDLT1
	const uint8_t kuiDLT2=(kuiT23+kuiT33)>>2;		// kDLT2
	const uint8_t kuiDLT3=kuiT33>>1; 	// kDLT3
	const uint8_t kuiList[8]={kuiDLT0,kuiDLT1,kuiDLT2,kuiDLT3,kuiDLT3,kuiDLT3,kuiDLT3,kuiDLT3};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+1));
	ST32A4(pPred+kiStride2,LD32(kuiList+2));
	ST32A4(pPred+kiStride3,LD32(kuiList+3));
}

// down right
void I4x4LumaPredDDR_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	uint8_t* ptopleft=&pPred[-(kiStride+1)];
	uint8_t* pleft=&pPred[-1];
	const uint8_t kuiLT=*ptopleft;
	// get pLeft and pTop
	const uint8_t kuiL0=*pleft;
	const uint8_t kuiL1=*(pleft+kiStride);
	const uint8_t kuiL2=*(pleft+kiStride2);
	const uint8_t kuiL3=*(pleft+kiStride3);
	const uint8_t kuiT0=*(ptopleft+1);
	const uint8_t kuiT1=*(ptopleft+2);
	const uint8_t kuiT2=*(ptopleft+3);
	const uint8_t kuiT3=*(ptopleft+4);
	const uint16_t kuiTL0=1+kuiLT+kuiL0;
	const uint16_t kuiLT0=1+kuiLT+kuiT0;
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiL01=1+kuiL0+kuiL1;
	const uint16_t kuiL12=1+kuiL1+kuiL2;
	const uint16_t kuiL23=1+kuiL2+kuiL3;
	const uint8_t kuiDDR0=(kuiTL0+kuiLT0)>>2;		// kuiDDR0
	const uint8_t kuiDDR1=(kuiLT0+kuiT01)>>2;		// kuiDDR1
	const uint8_t kuiDDR2=(kuiT01+kuiT12)>>2;		// kuiDDR2
	const uint8_t kuiDDR3=(kuiT12+kuiT23)>>2;		// kuiDDR3
	const uint8_t kuiDDR4=(kuiTL0+kuiL01)>>2;		// kuiDDR4
	const uint8_t kuiDDR5=(kuiL01+kuiL12)>>2;		// kuiDDR5
	const uint8_t kuiDDR6=(kuiL12+kuiL23)>>2;		// kuiDDR6
	const uint8_t kuiList[8]={kuiDDR6,kuiDDR5,kuiDDR4,kuiDDR0,kuiDDR1,kuiDDR2,kuiDDR3,0};

	ST32A4(pPred,LD32(kuiList+3));
	ST32A4(pPred+kiStride,LD32(kuiList+2));
	ST32A4(pPred+kiStride2,LD32(kuiList+1));
	ST32A4(pPred+kiStride3,LD32(kuiList));
}

// vertical pLeft
void I4x4LumaPredVL_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	uint8_t* ptopleft=&pPred[-(kiStride+1)];
	// get pTop
	const uint8_t kuiT0=*(ptopleft+1);
	const uint8_t kuiT1=*(ptopleft+2);
	const uint8_t kuiT2=*(ptopleft+3);
	const uint8_t kuiT3=*(ptopleft+4);
	const uint8_t kuiT4=*(ptopleft+5);
	const uint8_t kuiT5=*(ptopleft+6);
	const uint8_t kuiT6=*(ptopleft+7);
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiT34=1+kuiT3+kuiT4;
	const uint16_t kuiT45=1+kuiT4+kuiT5;
	const uint16_t kuiT56=1+kuiT5+kuiT6;
	const uint8_t kuiVL0=kuiT01>>1; 	// kuiVL0
	const uint8_t kuiVL1=kuiT12>>1; 	// kuiVL1
	const uint8_t kuiVL2=kuiT23>>1; 	// kuiVL2
	const uint8_t kuiVL3=kuiT34>>1; 	// kuiVL3
	const uint8_t kuiVL4=kuiT45>>1; 	// kuiVL4
	const uint8_t kuiVL5=(kuiT01+kuiT12)>>2;		// kuiVL5
	const uint8_t kuiVL6=(kuiT12+kuiT23)>>2;		// kuiVL6
	const uint8_t kuiVL7=(kuiT23+kuiT34)>>2;		// kuiVL7
	const uint8_t kuiVL8=(kuiT34+kuiT45)>>2;		// kuiVL8
	const uint8_t kuiVL9=(kuiT45+kuiT56)>>2;		// kuiVL9
	const uint8_t kuiList[10]={kuiVL0,kuiVL1,kuiVL2,kuiVL3,kuiVL4,kuiVL5,kuiVL6,kuiVL7,kuiVL8,kuiVL9};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+5));
	ST32A4(pPred+kiStride2,LD32(kuiList+1));
	ST32A4(pPred+kiStride3,LD32(kuiList+6));
}

// vertical pLeft
void I4x4LumaPredVLTop_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	uint8_t* ptopleft=&pPred[-(kiStride+1)];
	// get pTop
	const uint8_t kuiT0=*(ptopleft+1);
	const uint8_t kuiT1=*(ptopleft+2);
	const uint8_t kuiT2=*(ptopleft+3);
	const uint8_t kuiT3=*(ptopleft+4);
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiT33=1+(kuiT3<<1);
	const uint8_t kuiVL0=kuiT01>>1;
	const uint8_t kuiVL1=kuiT12>>1;
	const uint8_t kuiVL2=kuiT23>>1;
	const uint8_t kuiVL3=kuiT33>>1;
	const uint8_t kuiVL4=(kuiT01+kuiT12)>>2;
	const uint8_t kuiVL5=(kuiT12+kuiT23)>>2;
	const uint8_t kuiVL6=(kuiT23+kuiT33)>>2;
	const uint8_t kuiVL7=kuiVL3;
	const uint8_t kuiList[10]={kuiVL0,kuiVL1,kuiVL2,kuiVL3,kuiVL3,kuiVL4,kuiVL5,kuiVL6,kuiVL7,kuiVL7};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+5));
	ST32A4(pPred+kiStride2,LD32(kuiList+1));
	ST32A4(pPred+kiStride3,LD32(kuiList+6));
}


// vertical right
void I4x4LumaPredVR_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	const uint8_t kuiLT=pPred[-kiStride-1];
	// get pLeft and pTop
	const uint8_t kuiL0=pPred[-1];
	const uint8_t kuiL1=pPred[kiStride-1];
	const uint8_t kuiL2=pPred[kiStride2-1];
	const uint8_t kuiT0=pPred[-kiStride];
	const uint8_t kuiT1=pPred[1-kiStride];
	const uint8_t kuiT2=pPred[2-kiStride];
	const uint8_t kuiT3=pPred[3-kiStride];
	const uint8_t kuiVR0=(1+kuiLT+kuiT0)>>1;		// kuiVR0
	const uint8_t kuiVR1=(1+kuiT0+kuiT1)>>1;		// kuiVR1
	const uint8_t kuiVR2=(1+kuiT1+kuiT2)>>1;		// kuiVR2
	const uint8_t kuiVR3=(1+kuiT2+kuiT3)>>1;		// kuiVR3
	const uint8_t kuiVR4=(2+kuiL0+(kuiLT<<1)+kuiT0)>>2;		// kuiVR4
	const uint8_t kuiVR5=(2+kuiLT+(kuiT0<<1)+kuiT1)>>2;		// kuiVR5
	const uint8_t kuiVR6=(2+kuiT0+(kuiT1<<1)+kuiT2)>>2;		// kuiVR6
	const uint8_t kuiVR7=(2+kuiT1+(kuiT2<<1)+kuiT3)>>2;		// kuiVR7
	const uint8_t kuiVR8=(2+kuiLT+(kuiL0<<1)+kuiL1)>>2;		// kuiVR8
	const uint8_t kuiVR9=(2+kuiL0+(kuiL1<<1)+kuiL2)>>2;		// kuiVR9
	const uint8_t kuiList[10]={kuiVR8,kuiVR0,kuiVR1,kuiVR2,kuiVR3,kuiVR9,kuiVR4,kuiVR5,kuiVR6,kuiVR7};

	ST32A4(pPred,LD32(kuiList+1));
	ST32A4(pPred+kiStride,LD32(kuiList+6));
	ST32A4(pPred+kiStride2,LD32(kuiList));
	ST32A4(pPred+kiStride3,LD32(kuiList+5));
}

// horizontal up
void I4x4LumaPredHU_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	// get pLeft
	const uint8_t kuiL0=pPred[-1];
	const uint8_t kuiL1=pPred[kiStride-1];
	const uint8_t kuiL2=pPred[kiStride2-1];
	const uint8_t kuiL3=pPred[kiStride3-1];
	const uint16_t kuiL01=1+kuiL0+kuiL1;
	const uint16_t kuiL12=1+kuiL1+kuiL2;
	const uint16_t kuiL23=1+kuiL2+kuiL3;
	const uint8_t kuiHU0=kuiL01>>1;
	const uint8_t kuiHU1=(kuiL01+kuiL12)>>2;
	const uint8_t kuiHU2=kuiL12>>1;
	const uint8_t kuiHU3=(kuiL12+kuiL23)>>2;
	const uint8_t kuiHU4=kuiL23>>1;
	const uint8_t kuiHU5=(1+kuiL23+(kuiL3<<1))>>2;
	const uint8_t kuiList[10]={kuiHU0,kuiHU1,kuiHU2,kuiHU3,kuiHU4,kuiHU5,kuiL3,kuiL3,kuiL3,kuiL3};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+2));
	ST32A4(pPred+kiStride2,LD32(kuiList+4));
	ST32A4(pPred+kiStride3,LD32(kuiList+6));
}

// horizontal down
void I4x4LumaPredHD_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	const uint8_t kuiLT=pPred[-(kiStride+1)];
	// get pLeft and pTop
	const uint8_t kuiL0=pPred[-1];
	const uint8_t kuiL1=pPred[-1+kiStride];
	const uint8_t kuiL2=pPred[-1+kiStride2];
	const uint8_t kuiL3=pPred[-1+kiStride3];
	const uint8_t kuiT0=pPred[-kiStride];
	const uint8_t kuiT1=pPred[-kiStride+1];
	const uint8_t kuiT2=pPred[-kiStride+2];
	const uint16_t kuiTL0=1+kuiLT+kuiL0;
	const uint16_t kuiLT0=1+kuiLT+kuiT0;
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiL01=1+kuiL0+kuiL1;
	const uint16_t kuiL12=1+kuiL1+kuiL2;
	const uint16_t kuiL23=1+kuiL2+kuiL3;
	const uint8_t kuiHD0=kuiTL0>>1;
	const uint8_t kuiHD1=(kuiTL0+kuiLT0)>>2;
	const uint8_t kuiHD2=(kuiLT0+kuiT01)>>2;
	const uint8_t kuiHD3=(kuiT01+kuiT12)>>2;
	const uint8_t kuiHD4=kuiL01>>1;
	const uint8_t kuiHD5=(kuiTL0+kuiL01)>>2;
	const uint8_t kuiHD6=kuiL12>>1;
	const uint8_t kuiHD7=(kuiL01+kuiL12)>>2;
	const uint8_t kuiHD8=kuiL23>>1;
	const uint8_t kuiHD9=(kuiL12+kuiL23)>>2;
	const uint8_t kuiList[10]={kuiHD8,kuiHD9,kuiHD6,kuiHD7,kuiHD4,kuiHD5,kuiHD0,kuiHD1,kuiHD2,kuiHD3};

	ST32A4(pPred,LD32(kuiList+6));
	ST32A4(pPred+kiStride,LD32(kuiList+4));
	ST32A4(pPred+kiStride2,LD32(kuiList+2));
	ST32A4(pPred+kiStride3,LD32(kuiList));
}

void I8x8LumaPredV_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	uint64_t uiTop=0;
	int32_t iStride[8];
	uint8_t uiPixelFilterT[8];
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	// 8-89
	for(i=7; i>=0; i--){
		uiTop=((uiTop<<8)|uiPixelFilterT[i]);
	}
	for(i=0; i<8; i++){
		ST64A8(pPred+kiStride*i,uiTop);
	}
}

void I8x8LumaPredH_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	uint64_t uiLeft;
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	// 8-90
	for(i=0; i<8; i++){
		uiLeft=0x0101010101010101ULL*uiPixelFilterL[i];
		ST64A8(pPred+iStride[i],uiLeft);
	}
}

void I8x8LumaPredDc_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	uint16_t uiTotal=0;
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	// 8-91
	for(i=0; i<8; i++){
		uiTotal+=uiPixelFilterL[i];
		uiTotal+=uiPixelFilterT[i];
	}
	const uint8_t kuiMean=((uiTotal+8)>>4);
	const uint64_t kuiMean64=0x0101010101010101ULL*kuiMean;
	for(i=0; i<8; i++){
		ST64A8(pPred+iStride[i],kuiMean64);
	}
}

void I8x8LumaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	uint16_t uiTotal=0;
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	// 8-92
	for(i=0; i<8; i++){
		uiTotal+=uiPixelFilterL[i];
	}
	const uint8_t kuiMean=((uiTotal+4)>>3);
	const uint64_t kuiMean64=0x0101010101010101ULL*kuiMean;
	for(i=0; i<8; i++){
		ST64A8(pPred+iStride[i],kuiMean64);
	}
}

void I8x8LumaPredDcTop_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterT[8];
	uint16_t uiTotal=0;
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	// 8-93
	for(i=0; i<8; i++){
		uiTotal+=uiPixelFilterT[i];
	}
	const uint8_t kuiMean=((uiTotal+4)>>3);
	const uint64_t kuiMean64=0x0101010101010101ULL*kuiMean;
	for(i=0; i<8; i++){
		ST64A8(pPred+iStride[i],kuiMean64);
	}
}

void I8x8LumaPredDcNA_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// for normal 8 bit depth,8-94
	const uint64_t kuiDC64=0x8080808080808080ULL;
	int32_t iStride[8];
	int32_t i;
	ST64A8(pPred,kuiDC64);
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
		ST64A8(pPred+iStride[i],kuiDC64);
	}
}

// down pLeft
void I8x8LumaPredDDL_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top and Top-right available
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<15; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[15]=((pPred[14-kiStride]+pPred[15-kiStride]*3+2)>>2);
	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			if(i==7 && j==7){		// 8-95
				pPred[j+iStride[i]]=(uiPixelFilterT[14]+3*uiPixelFilterT[15]+2)>>2;
			}else{		// 8-96
				pPred[j+iStride[i]]=(uiPixelFilterT[i+j]+(uiPixelFilterT[i+j+1]<<1)+uiPixelFilterT[i+j+2]+2)>>2;
			}
		}
	}
}

// down pLeft
void I8x8LumaPredDDLTop_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top available and Top-right unavailable
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((
		pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	// p[x,-1] x=8...15 are replaced with p[7,-1]
	uiPixelFilterT[7]=((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	for(i=8; i<16; i++){
		uiPixelFilterT[i]=pPred[7-kiStride];
	}

	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			if(i==7 && j==7){		// 8-95
				pPred[j+iStride[i]]=(uiPixelFilterT[14]+3*uiPixelFilterT[15]+2)>>2;
			}else{		// 8-96
				pPred[j+iStride[i]]=(uiPixelFilterT[i+j]+(uiPixelFilterT[i+j+1]<<1)+uiPixelFilterT[i+j+2]+2)>>2;
			}
		}
	}
}

// down right
void I8x8LumaPredDDR_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// The TopLeft,Top,Left are all available under this mode
	int32_t iStride[8];
	uint8_t uiPixelFilterTL;
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	int32_t i,j;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterTL=(pPred[-1]+(pPred[-1-kiStride]<<1)+pPred[-kiStride]+2)>>2;
	uiPixelFilterL[0]=((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((
		pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		// 8-98,x < y-1
		for(j=0; j<(i-1); j++){
			pPred[j+iStride[i]]=(uiPixelFilterL[i-j-2]+(uiPixelFilterL[i-j-1]<<1)+uiPixelFilterL[i-j]+2)>>2;
		}
		// 8-98,special case,x==y-1
		if(i>=1){
			j=i-1;
			pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterL[0]<<1)+uiPixelFilterL[1]+2)>>2;
		}
		// 8-99,x==y
		j=i;
		pPred[j+iStride[i]]=(uiPixelFilterT[0]+(uiPixelFilterTL<<1)+uiPixelFilterL[0]+2)>>2;
		// 8-97,special case,x==y+1
		if(i<7){
			j=i+1;
			pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterT[0]<<1)+uiPixelFilterT[1]+2)>>2;
		}
		for(j=i+2; j<8; j++){		// 8-97,x > y+1
			pPred[j+iStride[i]]=(uiPixelFilterT[j-i-2]+(uiPixelFilterT[j-i-1]<<1)+uiPixelFilterT[j-i]+2)>>2;
		}
	}
}

// vertical pLeft
void I8x8LumaPredVL_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top and Top-right available
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((
		pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<15; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[15]=((pPred[14-kiStride]+pPred[15-kiStride]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		if((i&0x01)==0){		// 8-108
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+uiPixelFilterT[j+(i>>1)+1]+1)>>1;
			}
		}else{		// 8-109
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+(uiPixelFilterT[j+(i>>1)+1]<<1)+uiPixelFilterT[j+(i>>1)+2]+2)>>2;
			}
		}
	}
}

// vertical pLeft
void I8x8LumaPredVLTop_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top available and Top-right unavailable
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((
		pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	// p[x,-1] x=8...15 are replaced with p[7,-1]
	uiPixelFilterT[7]=((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	for(i=8; i<16; i++){
		uiPixelFilterT[i]=pPred[7-kiStride];
	}

	for(i=0; i<8; i++){		// y
		if((i&0x01)==0){		// 8-108
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+uiPixelFilterT[j+(i>>1)+1]+1)>>1;
			}
		}else{		// 8-109
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+(uiPixelFilterT[j+(i>>1)+1]<<1)+uiPixelFilterT[j+(i>>1)+2]+2)>>2;
			}
		}
	}
}

// vertical right
void I8x8LumaPredVR_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// The TopLeft,Top,Left are always available under this mode
	int32_t iStride[8];
	uint8_t uiPixelFilterTL;
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	int32_t i,j;
	int32_t izVR,izVRDiv;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterTL=(pPred[-1]+(pPred[-1-kiStride]<<1)+pPred[-kiStride]+2)>>2;

	uiPixelFilterL[0]=((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			izVR=(j<<1)-i;		// 2 * x-y
			izVRDiv=j-(i>>1);
			if(izVR>=0){
				if((izVR&0x01)==0){		// 8-100
					if(izVRDiv>0){
						pPred[j+iStride[i]]=(uiPixelFilterT[izVRDiv-1]+uiPixelFilterT[izVRDiv]+1)>>1;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterTL+uiPixelFilterT[0]+1)>>1;
					}
				}else{		// 8-101
					if(izVRDiv>1){
						pPred[j+iStride[i]]=(uiPixelFilterT[izVRDiv-2]+(uiPixelFilterT[izVRDiv-1]<<1)+uiPixelFilterT[izVRDiv]+2)
							>>2;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterT[0]<<1)+uiPixelFilterT[1]+2)>>2;
					}
				}
			}else
			if(izVR==-1){		// 8-102
				pPred[j+iStride[i]]=(uiPixelFilterL[0]+(uiPixelFilterTL<<1)+uiPixelFilterT[0]+2)>>2;
			}else
			if(izVR<-2){		// 8-103
				pPred[j+iStride[i]]=(uiPixelFilterL[-izVR-1]+(uiPixelFilterL[-izVR-2]<<1)+uiPixelFilterL[-izVR-3]+2)
					>>2;
			}else{		// izVR==-2,8-103,special case
				pPred[j+iStride[i]]=(uiPixelFilterL[1]+(uiPixelFilterL[0]<<1)+uiPixelFilterTL+2)>>2;
			}
		}
	}
}

// horizontal up
void I8x8LumaPredHU_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	int32_t i,j;
	int32_t izHU;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			izHU=j+(i<<1);		// x+2 * y
			if(izHU<13){
				if((izHU&0x01)==0){		// 8-110
					pPred[j+iStride[i]]=(uiPixelFilterL[izHU>>1]+uiPixelFilterL[1+(izHU>>1)]+1)>>1;
				}else{		// 8-111
					pPred[j+iStride[i]]=(uiPixelFilterL[izHU>>1]+(uiPixelFilterL[1+(izHU>>1)]<<1)+uiPixelFilterL[2+
											 (izHU>>1)]+2)>>2;
				}
			}else
			if(izHU==13){		// 8-112
				pPred[j+iStride[i]]=(uiPixelFilterL[6]+3*uiPixelFilterL[7]+2)>>2;
			}else{		// 8-113
				pPred[j+iStride[i]]=uiPixelFilterL[7];
			}
		}
	}
}

// horizontal down
void I8x8LumaPredHD_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// The TopLeft,Top,Left are all available under this mode
	int32_t iStride[8];
	uint8_t uiPixelFilterTL;
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	int32_t i,j;
	int32_t izHD,izHDDiv;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterTL=(pPred[-1]+(pPred[-1-kiStride]<<1)+pPred[-kiStride]+2)>>2;
	uiPixelFilterL[0]=((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			izHD=(i<<1)-j;		// 2*y-x
			izHDDiv=i-(j>>1);
			if(izHD>=0){
				if((izHD&0x01)==0){		// 8-104
					if(izHDDiv==0){
						pPred[j+iStride[i]]=(uiPixelFilterTL+uiPixelFilterL[0]+1)>>1;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterL[izHDDiv-1]+uiPixelFilterL[izHDDiv]+1)>>1;
					}
				}else{		// 8-105
					if(izHDDiv==1){
						pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterL[0]<<1)+uiPixelFilterL[1]+2)>>2;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterL[izHDDiv-2]+(uiPixelFilterL[izHDDiv-1]<<1)+uiPixelFilterL[izHDDiv]+2)
							>>2;
					}
				}
			}else
			if(izHD==-1){		// 8-106
				pPred[j+iStride[i]]=(uiPixelFilterL[0]+(uiPixelFilterTL<<1)+uiPixelFilterT[0]+2)>>2;
			}else
			if(izHD<-2){		// 8-107
				pPred[j+iStride[i]]=(uiPixelFilterT[-izHD-1]+(uiPixelFilterT[-izHD-2]<<1)+uiPixelFilterT[-izHD-3]+2)
					>>2;
			}else{		// 8-107 special case,izHD==-2
				pPred[j+iStride[i]]=(uiPixelFilterT[1]+(uiPixelFilterT[0]<<1)+uiPixelFilterTL+2)>>2;
			}
		}
	}
}

void IChromaPredH_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<3)-kiStride;
	uint8_t i=7;

	do{
		const uint8_t kuiVal8=pPred[iTmp-1];
		const uint64_t kuiVal64=0x0101010101010101ULL*kuiVal8;

		ST64A8(pPred+iTmp,kuiVal64);

		iTmp-=kiStride;
	} while(i-->0);
}

void IChromaPredV_c(uint8_t* pPred,const int32_t kiStride){
	const uint64_t kuiVal64=LD64A8(&pPred[-kiStride]);
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride4=kiStride2<<1;

	ST64A8(pPred,kuiVal64);
	ST64A8(pPred+kiStride,kuiVal64);
	ST64A8(pPred+kiStride2,kuiVal64);
	ST64A8(pPred+kiStride2+kiStride,kuiVal64);
	ST64A8(pPred+kiStride4,kuiVal64);
	ST64A8(pPred+kiStride4+kiStride,kuiVal64);
	ST64A8(pPred+kiStride4+kiStride2,kuiVal64);
	ST64A8(pPred+(kiStride<<3)-kiStride,kuiVal64);
}

void IChromaPredPlane_c(uint8_t* pPred,const int32_t kiStride){
	int32_t a=0,b=0,c=0,H=0,V=0;
	int32_t i,j;
	uint8_t* pTop=&pPred[-kiStride];
	uint8_t* pLeft=&pPred[-1];

	for(i=0; i<4; i++){
		H+=(i+1)*(pTop[4+i]-pTop[2-i]);
		V+=(i+1)*(pLeft[(4+i)*kiStride]-pLeft[(2-i)*kiStride]);
	}

	a=(pLeft[7*kiStride]+pTop[7])<<4;
	b=(17*H+16)>>5;
	c=(17*V+16)>>5;

	for(i=0; i<8; i++){
		for(j=0; j<8; j++){
			int32_t iTmp=(a+b*(j-3)+c*(i-3)+16)>>5;
			iTmp=Clip1(iTmp);
			pPred[j]=iTmp;
		}
		pPred+=kiStride;
	}
}

void IChromaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiL1=-1+kiStride;
	const int32_t kiL2=kiL1+kiStride;
	const int32_t kiL3=kiL2+kiStride;
	const int32_t kiL4=kiL3+kiStride;
	const int32_t kiL5=kiL4+kiStride;
	const int32_t kiL6=kiL5+kiStride;
	const int32_t kiL7=kiL6+kiStride;
	// caculate the kMean value
	const uint8_t kuiMUP=(pPred[-1]+pPred[kiL1]+pPred[kiL2]+pPred[kiL3]+2)>>2;
	const uint8_t kuiMDown=(pPred[kiL4]+pPred[kiL5]+pPred[kiL6]+pPred[kiL7]+2)>>2;
	const uint64_t kuiUP64=0x0101010101010101ULL*kuiMUP;
	const uint64_t kuiDN64=0x0101010101010101ULL*kuiMDown;

	ST64A8(pPred,kuiUP64);
	ST64A8(pPred+kiL1+1,kuiUP64);
	ST64A8(pPred+kiL2+1,kuiUP64);
	ST64A8(pPred+kiL3+1,kuiUP64);
	ST64A8(pPred+kiL4+1,kuiDN64);
	ST64A8(pPred+kiL5+1,kuiDN64);
	ST64A8(pPred+kiL6+1,kuiDN64);
	ST64A8(pPred+kiL7+1,kuiDN64);
}

void IChromaPredDcTop_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<3)-kiStride;
	// caculate the kMean value
	const uint8_t kuiM1=(pPred[-kiStride]+pPred[1-kiStride]+pPred[2-kiStride]+pPred[3-kiStride]+2)>>2;
	const uint8_t kuiM2=(pPred[4-kiStride]+pPred[5-kiStride]+pPred[6-kiStride]+pPred[7-kiStride]+2)>>2;
	const uint8_t kuiM[8]={kuiM1,kuiM1,kuiM1,kuiM1,kuiM2,kuiM2,kuiM2,kuiM2};
	uint8_t i=7;
	do{
		ST64A8(pPred+iTmp,LD64(kuiM));
		iTmp-=kiStride;
	} while(i-->0);
}

void IChromaPredDcNA_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<3)-kiStride;
	const uint64_t kuiDC64=0x8080808080808080ULL;
	uint8_t i=7;
	do{
		ST64A8(pPred+iTmp,kuiDC64);
		iTmp-=kiStride;
	} while(i-->0);
}

void PlainH264Decoder::InitPredFunc(SDecoderContext* pCtx){
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_V]=I16x16LumaPredV_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_H]=I16x16LumaPredH_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC]=I16x16LumaPredDc_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_P]=I16x16LumaPredPlane_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC_L]=I16x16LumaPredDcLeft_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC_T]=I16x16LumaPredDcTop_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC_128]=I16x16LumaPredDcNA_c;

	pCtx->pGetI4x4LumaPredFunc[I4_PRED_V]=I4x4LumaPredV_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_H]=I4x4LumaPredH_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC]=I4x4LumaPredDc_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC_L]=I4x4LumaPredDcLeft_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC_T]=I4x4LumaPredDcTop_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC_128]=I4x4LumaPredDcNA_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DDL]=I4x4LumaPredDDL_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DDL_TOP]=I4x4LumaPredDDLTop_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DDR]=I4x4LumaPredDDR_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_VL]=I4x4LumaPredVL_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_VL_TOP]=I4x4LumaPredVLTop_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_VR]=I4x4LumaPredVR_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_HU]=I4x4LumaPredHU_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_HD]=I4x4LumaPredHD_c;

	pCtx->pGetI8x8LumaPredFunc[I4_PRED_V]=I8x8LumaPredV_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_H]=I8x8LumaPredH_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC]=I8x8LumaPredDc_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC_L]=I8x8LumaPredDcLeft_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC_T]=I8x8LumaPredDcTop_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC_128]=I8x8LumaPredDcNA_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DDL]=I8x8LumaPredDDL_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DDL_TOP]=I8x8LumaPredDDLTop_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DDR]=I8x8LumaPredDDR_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_VL]=I8x8LumaPredVL_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_VL_TOP]=I8x8LumaPredVLTop_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_VR]=I8x8LumaPredVR_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_HU]=I8x8LumaPredHU_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_HD]=I8x8LumaPredHD_c;

	pCtx->pGetIChromaPredFunc[C_PRED_DC]=IChromaPredDc_c;
	pCtx->pGetIChromaPredFunc[C_PRED_H]=IChromaPredH_c;
	pCtx->pGetIChromaPredFunc[C_PRED_V]=IChromaPredV_c;
	pCtx->pGetIChromaPredFunc[C_PRED_P]=IChromaPredPlane_c;
	pCtx->pGetIChromaPredFunc[C_PRED_DC_L]=IChromaPredDcLeft_c;
	pCtx->pGetIChromaPredFunc[C_PRED_DC_T]=IChromaPredDcTop_c;
	pCtx->pGetIChromaPredFunc[C_PRED_DC_128]=IChromaPredDcNA_c;
}

// Predeclared function routines ..
int32_t PlainH264Decoder::ParseRefPicListReordering(SDecoderContext* pCtx,SBitStringAux* pBs,SSlice* pSh){
	// Reference picture list reordering syntax,refer to page 64 in JVT X201wcm
	struct SRefPicListReorderSyn{
		struct{
			uint32_t uiAbsDiffPicNumMinus1;
			uint16_t uiReorderingOfPicNumsIdc;
		} sReorderingSyn[LIST_A][MAX_REF_PIC_COUNT];
		bool bRefPicListReorderingFlag[LIST_A];
	};
	int32_t iList=0;
	const ESliceType keSt=pCtx->eSliceType;
	SRefPicListReorderSyn sRefPicListReordering;		// Reference picture list reordering syntaxs
	SRefPicListReorderSyn* pRefPicListReordering=&sRefPicListReordering;
	SSps* pSps=&pCtx->m_sps;
	uint32_t uiCode;
	if(keSt==I_SLICE)
		return ERR_NONE;
// Common syntaxs for P or B slices: list0,list1 followed if B slices used.
	do{
		READ_VERIFY(BsGetOneBit(pBs,&uiCode));						// ref_pic_list_modification_flag_l0
		pRefPicListReordering->bRefPicListReorderingFlag[iList]=!!uiCode;
		if(pRefPicListReordering->bRefPicListReorderingFlag[iList]){
			int32_t iIdx=0;
			do{
				READ_VERIFY(BsGetUe(pBs,&uiCode));					// modification_of_pic_nums_idc
				const uint32_t kuiIdc=uiCode;
// Fixed the referrence list reordering crash issue.(fault kIdc value > 3 case)---
				if((iIdx>=MAX_REF_PIC_COUNT) || (kuiIdc>3)){
					return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_REF_REORDERING);
				}
				pRefPicListReordering->sReorderingSyn[iList][iIdx].uiReorderingOfPicNumsIdc=kuiIdc;
				if(kuiIdc==3)
					break;
				if(iIdx>1)
					FATAL("REF NOT PREV");
				if(kuiIdc==0 || kuiIdc==1){
// abs_diff_pic_num_minus1 should be in range 0 to MaxPicNum-1,MaxPicNum is derived as 2^(4+log2_max_frame_num_minus4)
					READ_VERIFY(BsGetUe(pBs,&uiCode));				// abs_diff_pic_num_minus1
					WELS_CHECK_SE_UPPER_ERROR_NOLOG(uiCode,(uint32_t)(1<<pSps->uiLog2MaxFrameNum),"abs_diff_pic_num_minus1",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_REF_REORDERING));
					pRefPicListReordering->sReorderingSyn[iList][iIdx].uiAbsDiffPicNumMinus1=uiCode;		// uiAbsDiffPicNumMinus1
				}else
				if(kuiIdc==2){
					READ_VERIFY(BsGetUe(pBs,&uiCode));				// long_term_pic_num
				}
				++iIdx;
			} while(true);
		}
		break;
	} while(iList<LIST_A);

	return ERR_NONE;
}

#define SLICE_HEADER_IDR_PIC_ID_MAX 65535
#define SLICE_HEADER_REDUNDANT_PIC_CNT_MAX 127
#define SLICE_HEADER_ALPHAC0_BETA_OFFSET_MIN -12
#define SLICE_HEADER_ALPHAC0_BETA_OFFSET_MAX 12
#define SLICE_HEADER_CABAC_INIT_IDC_MAX 2

// Parse slice header of bitstream in avc for storing data structure
int32_t PlainH264Decoder::ParseSliceHeaderSyntaxs(SDecoderContext* pCtx,SBitStringAux* pBs){
	uint32_t uiCode;
	int32_t iCode;
	SSlice* slice=&pCtx->m_slice;
	ENalUnitType eNalType=pCtx->m_nalUnit.eNalUnitType;
// first_mb_in_slice
	READ_VERIFY(BsGetUe(pBs,&uiCode));				// first_mb_in_slice
	if(uiCode)
		FATAL("First mb must be zero");
	READ_VERIFY(BsGetUe(pBs,&uiCode));				// slice_type
	uint8_t uiSliceType=uiCode;
	if(uiSliceType>9){
		uprintf("slice type too large (%d)",uiSliceType);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_TYPE);
	}
	if(uiSliceType>4)
		uiSliceType-=5;
	if((NAL_UNIT_CODED_SLICE_IDR==eNalType) && (I_SLICE!=uiSliceType)){
		uprintf("Invalid slice type(%d) in IDR picture. ",uiSliceType);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_TYPE);
	}
	pCtx->eSliceType=(ESliceType)uiSliceType;
	READ_VERIFY(BsGetUe(pBs,&uiCode));				// pic_parameter_set_id
	WELS_CHECK_SE_UPPER_ERROR(uiCode,(MAX_PPS_COUNT-1),"iPpsId out of range",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_PPS_ID_OVERFLOW));
	int32_t iPpsId=uiCode;
	if(iPpsId)
		FATAL("MULTIPLE PPS NOT SUPPORTED");
	SPps* pPps=&pCtx->m_pps;
	SSps* pSps=&pCtx->m_sps;
	if(pSps->iNumRefFrames==0){
		if(uiSliceType!=I_SLICE){
			uprintf("slice_type (%d) not supported for num_ref_frames=0.",uiSliceType);
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_TYPE);
		}
	}
	bool bIdrFlag=eNalType==NAL_UNIT_CODED_SLICE_IDR ? true:false;
	if(pSps->uiLog2MaxFrameNum==0){
		uprintf("non existing SPS referenced");
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_NO_PARAM_SETS);
	}
	READ_VERIFY(BsGetBits(pBs,pSps->uiLog2MaxFrameNum,&uiCode));		// frame_num
	if(!pSps->bFrameMbsOnlyFlag){
		uprintf("ParseSliceHeaderSyntaxs(): frame_mbs_only_flag=%d not supported. ",pSps->bFrameMbsOnlyFlag);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_MBAFF);
	}
	if(bIdrFlag){
		READ_VERIFY(BsGetUe(pBs,&uiCode));							// idr_pic_id  standard 7.4.3 idr_pic_id should be in range 0 to 65535,inclusive.
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SLICE_HEADER_IDR_PIC_ID_MAX,"idr_pic_id",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_IDR_PIC_ID));
	}
	if(pSps->uiPocType==0){
		READ_VERIFY(BsGetBits(pBs,pSps->iLog2MaxPocLsb,&uiCode));	// pic_order_cnt_lsb
		//const int32_t iMaxPocLsb=1<<(pSps->iLog2MaxPocLsb);
		if(pPps->bPicOrderPresentFlag){
			READ_VERIFY(BsGetSe(pBs,&iCode));						// delta_pic_order_cnt_bottom
			//slice->iDeltaPicOrderCntBottom=iCode;
		}
// End of Calculating poc
	}else
	if(pSps->uiPocType==1 && !pSps->bDeltaPicOrderAlwaysZeroFlag){
		READ_VERIFY(BsGetSe(pBs,&iCode));							// delta_pic_order_cnt[ 0 ]
		if(pPps->bPicOrderPresentFlag){
			READ_VERIFY(BsGetSe(pBs,&iCode));						// delta_pic_order_cnt[ 1 ]
		}
	}
	if(pPps->bRedundantPicCntPresentFlag){
		READ_VERIFY(BsGetUe(pBs,&uiCode));							// redundant_pic_cnt standard section 7.4.3,redundant_pic_cnt should be in range 0 to 127,inclusive.
		if(uiCode>0){
			FATAL("Redundant picture not supported!");
		}
	}
// set defaults,might be overriden a few line later
	bool bReadNumRefFlag=P_SLICE==uiSliceType;
	if(bReadNumRefFlag){
		READ_VERIFY(BsGetOneBit(pBs,&uiCode));						// num_ref_idx_active_override_flag
		bool bNumRefIdxActiveOverrideFlag=!!uiCode;
		if(bNumRefIdxActiveOverrideFlag){
			READ_VERIFY(BsGetUe(pBs,&uiCode));						// num_ref_idx_l0_active_minus1
			if(uiCode)
				FATAL("REF PIC NOT PREV");
		}
	}
	int32_t iRet=ParseRefPicListReordering(pCtx,pBs,slice);
	if(iRet!=ERR_NONE){
		uprintf("invalid ref pPic list reordering syntaxs!");
		return iRet;
	}
	if(pCtx->m_nalUnit.uiNalRefIdc){
		uint32_t uiCode;
		if(bIdrFlag){
			READ_VERIFY(BsGetOneBit(pBs,&uiCode));				// no_output_of_prior_pics_flag
			READ_VERIFY(BsGetOneBit(pBs,&uiCode));				// long_term_reference_flag
		}else{
			READ_VERIFY(BsGetOneBit(pBs,&uiCode));				// adaptive_ref_pic_marking_mode_flag
		}
	}
	if(pCtx->eSliceType!=I_SLICE){
		READ_VERIFY(BsGetUe(pBs,&uiCode));
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SLICE_HEADER_CABAC_INIT_IDC_MAX,"cabac_init_idc",ERR_INFO_INVALID_CABAC_INIT_IDC);
		slice->iCabacInitIdc=uiCode;
		if(slice->iCabacInitIdc)
			FATAL("JUST Testing");
	}else
		slice->iCabacInitIdc=0;
	READ_VERIFY(BsGetSe(pBs,&iCode));								// slice_qp_delta
	int32_t iSliceQpDelta=iCode;
	slice->iSliceQp=pPps->iPicInitQp+iSliceQpDelta;
	if(slice->iSliceQp<0 || slice->iSliceQp > 51){
		uprintf("QP %d out of range",slice->iSliceQp);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_QP);
	}
	slice->iSliceAlphaC0Offset=0;
	slice->iSliceBetaOffset=0;
	if(pPps->bDeblockingFilterControlPresentFlag){
		READ_VERIFY(BsGetUe(pBs,&uiCode));							// disable_deblocking_filter_idc
		if(uiCode)
			FATAL("BLOCKING FILTER DISABLED, NOT SUPPORTED");
		READ_VERIFY(BsGetSe(pBs,&iCode));							// slice_alpha_c0_offset_div2
		slice->iSliceAlphaC0Offset=iCode*2;
		WELS_CHECK_SE_BOTH_ERROR(slice->iSliceAlphaC0Offset,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MIN,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MAX,"slice_alpha_c0_offset_div2 * 2",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_ALPHA_C0_OFFSET_DIV2));
		READ_VERIFY(BsGetSe(pBs,&iCode));							// slice_beta_offset_div2
		slice->iSliceBetaOffset=iCode*2;
		WELS_CHECK_SE_BOTH_ERROR(slice->iSliceBetaOffset,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MIN,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MAX,"slice_beta_offset_div2 * 2",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_BETA_OFFSET_DIV2));
	}
	return ERR_NONE;
}

// return decoded bytes payload,might be (pSrcRbsp+1) if no escapes
const uint8_t* PlainH264Decoder::ParseNalHeader(int32_t* iNalSizeOut,SNalUnit* pNalUnitHeader,const uint8_t* pSrcRbsp,int32_t iSrcRbspLen,const uint8_t* pSrcNal,int32_t iSrcNalLen,int32_t* pConsumedBytes){
	//SNalUnit* pCurNal=NULL;
	const uint8_t* pNal=pSrcRbsp;
	int32_t iNalSize=iSrcRbspLen;
	//int32_t iBitSize=0;
// remove the consecutive ZERO at the end of current NAL in the reverse order.--2011.6.1
	{
		int32_t iIndex=iSrcRbspLen-1;
		uint8_t uiBsZero=0;
		while(iIndex>=0){
			uiBsZero=pSrcRbsp[iIndex];
			if(0==uiBsZero){
				--iNalSize;
				++(*pConsumedBytes);
				--iIndex;
			}else{
				break;
			}
		}
	}
	pNalUnitHeader->uiNalRefIdc=(uint8_t)(pNal[0]>>5);					// uiNalRefIdc
	pNalUnitHeader->eNalUnitType=(ENalUnitType)(pNal[0]&0x1f);		// eNalUnitType
	++pNal;
	--iNalSize;
	++(*pConsumedBytes);
	*iNalSizeOut=iNalSize;
	return pNal;
}
// table A-1-Level limits
const SLevelLimits g_ksLevelLimits[]={
	{LEVEL_1_0,1485,99,396,64,175,-256,255,2,0x7fff},				// level 1
	{LEVEL_1_B,1485,99,396,128,350,-256,255,2,0x7fff},				// level 1.b
	{LEVEL_1_1,3000,396,900,192,500,-512,511,2,0x7fff},				// level 1.1
	{LEVEL_1_2,6000,396,2376,384,1000,-512,511,2,0x7fff},			// level 1.2
	{LEVEL_1_3,11880,396,2376,768,2000,-512,511,2,0x7fff},			// level 1.3
	{LEVEL_2_0,11880,396,2376,2000,2000,-512,511,2,0x7fff},			// level 2
	{LEVEL_2_1,19800,792,4752,4000,4000,-1024,1023,2,0x7fff},		// level 2.1
	{LEVEL_2_2,20250,1620,8100,4000,4000,-1024,1023,2,0x7fff},		// level 2.2
	{LEVEL_3_0,40500,1620,8100,10000,10000,-1024,1023,2,32 },		// level 3
	{LEVEL_3_1,108000,3600,18000,14000,14000,-2048,2047,4,16},		// level 3.1
	{LEVEL_3_2,216000,5120,20480,20000,20000,-2048,2047,4,16},		// level 3.2
	{LEVEL_4_0,245760,8192,32768,20000,25000,-2048,2047,4,16},		// level 4
	{LEVEL_4_1,245760,8192,32768,50000,62500,-2048,2047,2,16},		// level 4.1
	{LEVEL_4_2,522240,8704,34816,50000,62500,-2048,2047,2,16},		// level 4.2
	{LEVEL_5_0,589824,22080,110400,135000,135000,-2048,2047,2,16},	// level 5
	{LEVEL_5_1,983040,36864,184320,240000,240000,-2048,2047,2,16},	// level 5.1
	{LEVEL_5_2,2073600,36864,184320,240000,240000,-2048,2047,2,16}	// level 5.2
};

const SLevelLimits* GetLevelLimits(int32_t iLevelIdx,bool bConstraint3){
	switch(iLevelIdx){
		case 9:
			return &g_ksLevelLimits[1];
		case 10:
			return &g_ksLevelLimits[0];
		case 11:
			if(bConstraint3)
				return &g_ksLevelLimits[1];
			else
				return &g_ksLevelLimits[2];
		case 12:
			return &g_ksLevelLimits[3];
		case 13:
			return &g_ksLevelLimits[4];
		case 20:
			return &g_ksLevelLimits[5];
		case 21:
			return &g_ksLevelLimits[6];
		case 22:
			return &g_ksLevelLimits[7];
		case 30:
			return &g_ksLevelLimits[8];
		case 31:
			return &g_ksLevelLimits[9];
		case 32:
			return &g_ksLevelLimits[10];
		case 40:
			return &g_ksLevelLimits[11];
		case 41:
			return &g_ksLevelLimits[12];
		case 42:
			return &g_ksLevelLimits[13];
		case 50:
			return &g_ksLevelLimits[14];
		case 51:
			return &g_ksLevelLimits[15];
		case 52:
			return &g_ksLevelLimits[16];
		default:
			return NULL;
	}
	return NULL;
}

#define SPS_LOG2_MAX_FRAME_NUM_MINUS4_MAX 12
#define SPS_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4_MAX 12
#define SPS_NUM_REF_FRAMES_IN_PIC_ORDER_CNT_CYCLE_MAX 255
#define SPS_MAX_NUM_REF_FRAMES_MAX 16
#define PPS_PIC_INIT_QP_QS_MIN 0
#define PPS_PIC_INIT_QP_QS_MAX 51
#define PPS_CHROMA_QP_INDEX_OFFSET_MIN -12
#define PPS_CHROMA_QP_INDEX_OFFSET_MAX 12
#define SCALING_LIST_DELTA_SCALE_MAX 127
#define SCALING_LIST_DELTA_SCALE_MIN -128

int32_t PlainH264Decoder::ParseSps(SSps* pSps,SBitStringAux* pBsAux){
	SBitStringAux* pBs=pBsAux;
	ProfileIdc uiProfileIdc;
	uint8_t uiLevelIdc;
	uint32_t uiCode;
	int32_t iCode;
	//int32_t iRet=ERR_NONE;
	bool bConstraintSetFlags[6]={false};
	READ_VERIFY(BsGetBits(pBs,8,&uiCode));		// profile_idc
	uiProfileIdc=uiCode;
	if(uiProfileIdc!=PRO_BASELINE && uiProfileIdc!=PRO_MAIN && uiProfileIdc!=PRO_SCALABLE_BASELINE && uiProfileIdc!=PRO_SCALABLE_HIGH && uiProfileIdc!=PRO_EXTENDED && uiProfileIdc!=PRO_HIGH){
		uprintf("SPS ID can not be supported!\n");
		return false;
	}
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set0_flag
	bConstraintSetFlags[0]=!!uiCode;
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set1_flag
	bConstraintSetFlags[1]=!!uiCode;
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set2_flag
	bConstraintSetFlags[2]=!!uiCode;
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set3_flag
	bConstraintSetFlags[3]=!!uiCode;
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set4_flag
	bConstraintSetFlags[4]=!!uiCode;
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set5_flag
	bConstraintSetFlags[5]=!!uiCode;
	READ_VERIFY(BsGetBits(pBs,2,&uiCode));		// reserved_zero_2bits,equal to 0
	READ_VERIFY(BsGetBits(pBs,8,&uiCode));		// level_idc
	uiLevelIdc=uiCode;
	READ_VERIFY(BsGetUe(pBs,&uiCode));		// seq_parameter_set_id
	if(uiCode){		// Modified to check invalid negative iSpsId,12/1/2009
		uprintf(" iSpsId is out of range! \n");
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_SPS_ID_OVERFLOW);
	}
	if(uiCode)
		FATAL("MULTIPLE SPS NOT SUPPORTED");
	memset(pSps,0,sizeof(SSps));
	// Use the level 5.2 for compatibility
	const SLevelLimits* pSMaxLevelLimits=GetLevelLimits(52,false);
	const SLevelLimits* pSLevelLimits=GetLevelLimits(uiLevelIdc,bConstraintSetFlags[3]);
	if(NULL==pSLevelLimits){
		uprintf("ParseSps: level_idx (%d).\n",uiLevelIdc);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);
	}else pSps->pSLevelLimits=pSLevelLimits;
	// syntax elements in default
	pSps->uiChromaFormatIdc=1;
	pSps->uiProfileIdc=uiProfileIdc;
	pSps->uiLevelIdc=uiLevelIdc;

	if(PRO_SCALABLE_BASELINE==uiProfileIdc || PRO_SCALABLE_HIGH==uiProfileIdc || PRO_HIGH==uiProfileIdc || PRO_HIGH10==uiProfileIdc || PRO_HIGH422==uiProfileIdc || PRO_HIGH444==uiProfileIdc || PRO_CAVLC444==uiProfileIdc || 44==uiProfileIdc){

		READ_VERIFY(BsGetUe(pBs,&uiCode));			// chroma_format_idc
		pSps->uiChromaFormatIdc=uiCode;
		if(pSps->uiChromaFormatIdc>1){
			uprintf("ParseSps: chroma_format_idc (%d) <=1 supported.",pSps->uiChromaFormatIdc);
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);

		}// To support 4:0:0; 4:2:0
		READ_VERIFY(BsGetUe(pBs,&uiCode));			// bit_depth_luma_minus8
		if(uiCode!=0){
			uprintf("ParseSps: bit_depth_luma (%d) Only 8 bit supported.",8+uiCode);
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);
		}
		READ_VERIFY(BsGetUe(pBs,&uiCode));			// bit_depth_chroma_minus8
		if(uiCode!=0){
			uprintf("ParseSps: bit_depth_chroma (%d). Only 8 bit supported.",8+uiCode);
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);
		}
		READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// qpprime_y_zero_transform_bypass_flag
		pSps->bQpPrimeYZeroTransfBypassFlag=!!uiCode;
		READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// seq_scaling_matrix_present_flag
		bool bSeqScalingMatrixPresentFlag=!!uiCode;
		if(bSeqScalingMatrixPresentFlag){
			FATAL("REMOVED");
		}
	}
	READ_VERIFY(BsGetUe(pBs,&uiCode));				// log2_max_frame_num_minus4
	WELS_CHECK_SE_UPPER_ERROR(uiCode,SPS_LOG2_MAX_FRAME_NUM_MINUS4_MAX,"log2_max_frame_num_minus4",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_LOG2_MAX_FRAME_NUM_MINUS4));
	pSps->uiLog2MaxFrameNum=LOG2_MAX_FRAME_NUM_OFFSET+uiCode;
	READ_VERIFY(BsGetUe(pBs,&uiCode));				// pic_order_cnt_type
	pSps->uiPocType=uiCode;

	if(0==pSps->uiPocType){
		READ_VERIFY(BsGetUe(pBs,&uiCode));			// log2_max_pic_order_cnt_lsb_minus4
		// log2_max_pic_order_cnt_lsb_minus4 should be in range 0 to 12,inclusive. (sec. 7.4.3)
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SPS_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4_MAX,"log2_max_pic_order_cnt_lsb_minus4",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4));
		pSps->iLog2MaxPocLsb=LOG2_MAX_PIC_ORDER_CNT_LSB_OFFSET+uiCode;		// log2_max_pic_order_cnt_lsb_minus4

	}else
	if(1==pSps->uiPocType){
		int32_t i;
		READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// delta_pic_order_always_zero_flag
		pSps->bDeltaPicOrderAlwaysZeroFlag=!!uiCode;
		READ_VERIFY(BsGetSe(pBs,&iCode));			// offset_for_non_ref_pic
		pSps->iOffsetForNonRefPic=iCode;
		READ_VERIFY(BsGetSe(pBs,&iCode));			// offset_for_top_to_bottom_field
		pSps->iOffsetForTopToBottomField=iCode;
		READ_VERIFY(BsGetUe(pBs,&uiCode));			// num_ref_frames_in_pic_order_cnt_cycle
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SPS_NUM_REF_FRAMES_IN_PIC_ORDER_CNT_CYCLE_MAX,"num_ref_frames_in_pic_order_cnt_cycle",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_NUM_REF_FRAME_IN_PIC_ORDER_CNT_CYCLE));
		pSps->iNumRefFramesInPocCycle=uiCode;
		for(i=0; i<pSps->iNumRefFramesInPocCycle; i++){
			READ_VERIFY(BsGetSe(pBs,&iCode));		// offset_for_ref_frame[ i ]
			pSps->iOffsetForRefFrame[i]=iCode;
		}
	}
	if(pSps->uiPocType>2){
		uprintf(" illegal pic_order_cnt_type: %d ! ",pSps->uiPocType);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_POC_TYPE);
	}

	READ_VERIFY(BsGetUe(pBs,&uiCode));				// max_num_ref_frames
	pSps->iNumRefFrames=uiCode;
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));			// gaps_in_frame_num_value_allowed_flag
	pSps->bGapsInFrameNumValueAllowedFlag=!!uiCode;
	READ_VERIFY(BsGetUe(pBs,&uiCode));				// pic_width_in_mbs_minus1
	pSps->iMbWidth=PIC_WIDTH_IN_MBS_OFFSET+uiCode;
	if(pSps->iMbWidth>MAX_MB_SIZE || pSps->iMbWidth==0){
		FATAL("pic_width_in_mbs(%d) invalid!",pSps->iMbWidth);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
	}
	if(((uint64_t)pSps->iMbWidth*(uint64_t)pSps->iMbWidth)>(uint64_t)(8*pSLevelLimits->uiMaxFS)){
		if(((uint64_t)pSps->iMbWidth*(uint64_t)pSps->iMbWidth)>(uint64_t)(8*pSMaxLevelLimits->uiMaxFS)){
			FATAL("the pic_width_in_mbs exceeds the level limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
		}else{
			uprintf("the pic_width_in_mbs exceeds the level limits!");
		}
	}
	READ_VERIFY(BsGetUe(pBs,&uiCode));				// pic_height_in_map_units_minus1
	pSps->iMbHeight=PIC_HEIGHT_IN_MAP_UNITS_OFFSET+uiCode;
	if(pSps->iMbHeight>MAX_MB_SIZE || pSps->iMbHeight==0){
		FATAL("pic_height_in_mbs(%d) invalid!",pSps->iMbHeight);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
	}
	if(((uint64_t)pSps->iMbHeight*(uint64_t)pSps->iMbHeight)>(uint64_t)(8*pSLevelLimits->uiMaxFS)){
		if(((uint64_t)pSps->iMbHeight*(uint64_t)pSps->iMbHeight)>(uint64_t)(8*pSMaxLevelLimits->uiMaxFS)){
			FATAL("the pic_height_in_mbs exceeds the level limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
		}else{
			uprintf("the pic_height_in_mbs exceeds the level limits!");
		}
	}
	uint64_t uiTmp64=(uint64_t)pSps->iMbWidth*(uint64_t)pSps->iMbHeight;
	if(uiTmp64>(uint64_t)pSLevelLimits->uiMaxFS){
		if(uiTmp64>(uint64_t)pSMaxLevelLimits->uiMaxFS){
			FATAL("the total count of mb exceeds the level limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
		}else{
			uprintf("the total count of mb exceeds the level limits!");
		}
	}
	pSps->uiTotalMbCount=(uint32_t)uiTmp64;
	WELS_CHECK_SE_UPPER_ERROR(pSps->iNumRefFrames,SPS_MAX_NUM_REF_FRAMES_MAX,"max_num_ref_frames",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_NUM_REF_FRAMES));
	// here we check max_num_ref_frames
	uint32_t uiMaxDpbMbs=pSLevelLimits->uiMaxDPBMbs;
	uint32_t uiMaxDpbFrames=uiMaxDpbMbs/pSps->uiTotalMbCount;
	if(uiMaxDpbFrames>SPS_MAX_NUM_REF_FRAMES_MAX)
		uiMaxDpbFrames=SPS_MAX_NUM_REF_FRAMES_MAX;
	if((uint32_t)pSps->iNumRefFrames>uiMaxDpbFrames){
		uprintf(" max_num_ref_frames exceeds level limits!");
	}
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// frame_mbs_only_flag
	pSps->bFrameMbsOnlyFlag=!!uiCode;
	if(!pSps->bFrameMbsOnlyFlag){
		uprintf("ParseSps: frame_mbs_only_flag (%d) not supported.",pSps->bFrameMbsOnlyFlag);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_MBAFF);
	}
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// direct_8x8_inference_flag
	pSps->bDirect8x8InferenceFlag=!!uiCode;
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// frame_cropping_flag
	bool bFrameCroppingFlag=!!uiCode;
	if(bFrameCroppingFlag){
		uprintf("WARNING: Frame crop not supported, ignore!\n");
		READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_left_offset
		READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_right_offset
		READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_top_offset
		READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_bottom_offset
	}
	READ_VERIFY(BsGetOneBit(pBs,&uiCode));	// vui_parameters_present_flag
	return ERR_NONE;
}

int32_t PlainH264Decoder::ParsePps(SPps* pPps,SBitStringAux* pBsAux){
	uint32_t uiCode;
	int32_t iCode;
	READ_VERIFY(BsGetUe(pBsAux,&uiCode));			// pic_parameter_set_id
	if(uiCode)
		FATAL("ONLY ONE PPS SUPPORTED");
	memset(pPps,0,sizeof(SPps));
	READ_VERIFY(BsGetUe(pBsAux,&uiCode));			// seq_parameter_set_id
	if(uiCode)
		FATAL("MULTIPLE SPS NOT SUPPORTED");
	READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// entropy_coding_mode_flag
	bool bEntropyCodingModeFlag=!!uiCode;
	if(!bEntropyCodingModeFlag)
		FATAL("CAVLC Not supported, only CABAC");
	READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// bottom_field_pic_order_in_frame_present_flag
	pPps->bPicOrderPresentFlag=!!uiCode;
	READ_VERIFY(BsGetUe(pBsAux,&uiCode));			// num_slice_groups_minus1
	uint32_t uiNumSliceGroups=NUM_SLICE_GROUPS_OFFSET+uiCode;
	if(uiNumSliceGroups>1){
		FATAL("CODE REMOVED");
	}
	READ_VERIFY(BsGetUe(pBsAux,&uiCode));				// num_ref_idx_l0_default_active_minus1
	uint32_t uiNumRefIdxL0Active=NUM_REF_IDX_L0_DEFAULT_ACTIVE_OFFSET+uiCode;
	READ_VERIFY(BsGetUe(pBsAux,&uiCode));				// num_ref_idx_l1_default_active_minus1
	uint32_t uiNumRefIdxL1Active=NUM_REF_IDX_L1_DEFAULT_ACTIVE_OFFSET+uiCode;
	if(uiNumRefIdxL0Active!=1 || uiNumRefIdxL1Active!=1)
		FATAL("REF PIC NOT PREV");
	READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// weighted_pred_flag
	READ_VERIFY(BsGetBits(pBsAux,2,&uiCode));			// weighted_bipred_idc
	READ_VERIFY(BsGetSe(pBsAux,&iCode));				// pic_init_qp_minus26
	pPps->iPicInitQp=PIC_INIT_QP_OFFSET+iCode;
	WELS_CHECK_SE_BOTH_ERROR(pPps->iPicInitQp,PPS_PIC_INIT_QP_QS_MIN,PPS_PIC_INIT_QP_QS_MAX,"pic_init_qp_minus26+26",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_PIC_INIT_QP));
	READ_VERIFY(BsGetSe(pBsAux,&iCode));				// pic_init_qs_minus26
	pPps->iPicInitQs=PIC_INIT_QS_OFFSET+iCode;
	WELS_CHECK_SE_BOTH_ERROR(pPps->iPicInitQs,PPS_PIC_INIT_QP_QS_MIN,PPS_PIC_INIT_QP_QS_MAX,"pic_init_qs_minus26+26",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_PIC_INIT_QS));
	READ_VERIFY(BsGetSe(pBsAux,&iCode));				// chroma_qp_index_offset,cb
	pPps->iChromaQpIndexOffset[0]=iCode;
	WELS_CHECK_SE_BOTH_ERROR(pPps->iChromaQpIndexOffset[0],PPS_CHROMA_QP_INDEX_OFFSET_MIN,PPS_CHROMA_QP_INDEX_OFFSET_MAX,"chroma_qp_index_offset",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_CHROMA_QP_INDEX_OFFSET));
	pPps->iChromaQpIndexOffset[1]=pPps->iChromaQpIndexOffset[0];	// init cr qp offset
	READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// deblocking_filter_control_present_flag
	pPps->bDeblockingFilterControlPresentFlag=!!uiCode;
	READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// constrained_intra_pred_flag
	bool bConstainedIntraPredFlag=!!uiCode;
	if(bConstainedIntraPredFlag)
		FATAL("NOT SUPPORTED");
	READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// redundant_pic_cnt_present_flag
	pPps->bRedundantPicCntPresentFlag=!!uiCode;
	if(CheckMoreRBSPData(pBsAux)){
		READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// transform_8x8_mode_flag
		pPps->bTransform8x8ModeFlag=!!uiCode;
		READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// pic_scaling_matrix_present_flag
		bool bPicScalingMatrixPresentFlag=!!uiCode;
		if(bPicScalingMatrixPresentFlag){
			FATAL("REMOVED");
		}
		READ_VERIFY(BsGetSe(pBsAux,&iCode));		// second_chroma_qp_index_offset
		pPps->iChromaQpIndexOffset[1]=iCode;
		WELS_CHECK_SE_BOTH_ERROR(pPps->iChromaQpIndexOffset[1],PPS_CHROMA_QP_INDEX_OFFSET_MIN,PPS_CHROMA_QP_INDEX_OFFSET_MAX,"chroma_qp_index_offset",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_CHROMA_QP_INDEX_OFFSET));
	}
	return ERR_NONE;
}

uint8_t* PlainH264Decoder::DetectStartCodePrefix(const uint8_t* kpBuf,int32_t* pOffset,int32_t iBufSize){
	uint8_t* pBits=(uint8_t*)kpBuf;
	do{
		int32_t iIdx=0;
		while((iIdx<iBufSize) && (!(*pBits))){
			++pBits;
			++iIdx;
		}
		if(iIdx>=iBufSize) break;
		++iIdx;
		++pBits;
		if((iIdx>=3) && ((*(pBits-1))==0x1)){
			*pOffset=(int32_t)(((uintptr_t)pBits)-((uintptr_t)kpBuf));
			return pBits;
		}
		iBufSize-=iIdx;
	} while(1);
	return NULL;
}

const uint8_t* PlainH264Decoder::DecodeBsInit(int32_t* piNalBitSize,SDecoderContext* pCtx,const uint8_t* kpBsBuf,const int32_t kiBsLen){
	SDataBuffer* pRawData=&pCtx->sRawData;
	int32_t iDstIdx=0;											// the size of current NAL after 0x03 removal and 00 00 01 removal
	int32_t iOffset=0;
	if(NULL==DetectStartCodePrefix(kpBsBuf,&iOffset,kiBsLen)){
		FATAL("Unable to find start 0,0,0,1 prefix");
	}
	const uint8_t* pSrcNal=kpBsBuf+iOffset;
	int32_t iSrcLength=kiBsLen-iOffset;							// the total size of current AU or NAL
	if((kiBsLen+4)>(pRawData->pEnd-pRawData->pCurPos)){
		pRawData->pCurPos=pRawData->pHead;
	}
	// copy raw data from source buffer (application) to raw data buffer (codec inside) 0x03 removal and extract all of NAL Unit from current raw data
	uint8_t* pDstNal=pRawData->pCurPos;
	bool bNalStartBytes=false;
	SNalUnit sCurNalHead1;
	memset(&sCurNalHead1,0,sizeof(sCurNalHead1));
	SNalUnit* pCurNalHead=&sCurNalHead1;
	int32_t iSrcIdx=0;											// the index of source bit-stream till now after parsing one or more NALs
	int32_t iSrcConsumed=0;										// consumed bit count of source bs
	while(iSrcConsumed<iSrcLength){
		if((2+iSrcConsumed<iSrcLength) && (0==LD16(pSrcNal+iSrcIdx)) && (pSrcNal[2+iSrcIdx]<=0x03)){
			if(bNalStartBytes && (pSrcNal[2+iSrcIdx]!=0x00 && pSrcNal[2+iSrcIdx]!=0x01)){
				FATAL("Nal tag fail");
			}
			if(pSrcNal[2+iSrcIdx]==0x02){
				FATAL("Nal tag fail");
			}else
			if(pSrcNal[2+iSrcIdx]==0x00){
				pDstNal[iDstIdx++]=pSrcNal[iSrcIdx++];
				iSrcConsumed++;
				bNalStartBytes=true;
			}else
			if(pSrcNal[2+iSrcIdx]==0x03){
				if((3+iSrcConsumed<iSrcLength) && pSrcNal[3+iSrcIdx]>0x03){
					FATAL("Nal tag fail");
				}else{
					ST16(pDstNal+iDstIdx,0);
					iDstIdx+=2;
					iSrcIdx+=3;
					iSrcConsumed+=3;
				}
			}else{		// 0x01
				bNalStartBytes=false;
				int32_t iConsumedBytes=0;
				pDstNal[iDstIdx]=pDstNal[iDstIdx+1]=pDstNal[iDstIdx+2]=pDstNal[iDstIdx+3]=0;		// set 4 reserved bytes to zero
				int32_t iNalSize;
				const uint8_t* pNalPayload=ParseNalHeader(&iNalSize,pCurNalHead,pDstNal,iDstIdx,pSrcNal-3,iSrcIdx+3,&iConsumedBytes);
				if(pCurNalHead->eNalUnitType==NAL_UNIT_CODED_SLICE || pCurNalHead->eNalUnitType==NAL_UNIT_CODED_SLICE_IDR)
					FATAL("CODED SLICE SHOULD NOT BE HERE");
				pCtx->m_nalUnit.eNalUnitType=pCurNalHead->eNalUnitType;
				if(IS_PARAM_SETS_NALS(pCurNalHead->eNalUnitType)){
					const int32_t kiSrcLen=iDstIdx-iConsumedBytes;
					int32_t iBitSize=(kiSrcLen<<3)-BsGetTrailingBits(pNalPayload+kiSrcLen-1);		// convert into bit
					if(!iBitSize)
						FATAL("NO DATA!");
					SBitStringAux sBs;
					DecInitBits(&sBs,pNalPayload,iBitSize);
					if(pCurNalHead->eNalUnitType==NAL_UNIT_SPS) {
						ParseSps(&pCtx->m_sps,&sBs);
					}else
					if(pCurNalHead->eNalUnitType==NAL_UNIT_PPS) {
						ParsePps(&pCtx->m_pps,&sBs);
					}else{
						FATAL("Unexpected nal type %d",pCurNalHead->eNalUnitType);
					}
				}
				pDstNal+=(iDstIdx+4);								// init,increase 4 reserved zero bytes,used to store the next NAL
				if((iSrcLength-iSrcConsumed+4)>(pRawData->pEnd-pDstNal)){
					pDstNal=pRawData->pCurPos=pRawData->pHead;
				}else{
					pRawData->pCurPos=pDstNal;
				}
				pSrcNal+=iSrcIdx+3;
				iSrcConsumed+=3;
				iSrcIdx=0;
				iDstIdx=0;											// reset 0,used to statistic the length of next NAL
			}
			continue;
		}
		pDstNal[iDstIdx++]=pSrcNal[iSrcIdx++];
		iSrcConsumed++;
	}
	int32_t iConsumedBytes=0;
	pDstNal[iDstIdx]=pDstNal[iDstIdx+1]=pDstNal[iDstIdx+2]=pDstNal[iDstIdx+3]=0;		// set 4 reserved bytes to zero
	pRawData->pCurPos=pDstNal+iDstIdx+4;												// init,increase 4 reserved zero bytes,used to store the next NAL
	int32_t iNalSize;
	const uint8_t* pNalPayload=ParseNalHeader(&iNalSize,pCurNalHead,pDstNal,iDstIdx,pSrcNal-3,iSrcIdx+3,&iConsumedBytes);
	if(pCurNalHead->eNalUnitType!=NAL_UNIT_CODED_SLICE && pCurNalHead->eNalUnitType!=NAL_UNIT_CODED_SLICE_IDR)
		FATAL("CODED SLICE SHOULD BE HERE");
	SNalUnit* pCurNal=&pCtx->m_nalUnit;													// ready for next nal position
	memset(pCurNal,0,sizeof(SNalUnit));													// Please do not remove this for cache intend!!
	pCurNal->uiNalRefIdc=pCurNalHead->uiNalRefIdc;
	pCurNal->eNalUnitType=pCurNalHead->eNalUnitType;
	if(pCurNalHead->eNalUnitType==NAL_UNIT_CODED_SLICE_EXT){
		FATAL("REMOVED");
	}
	int32_t iBitSize=(iNalSize<<3)-BsGetTrailingBits(pNalPayload+iNalSize-1);			// convert into bit
	*piNalBitSize=iBitSize;
	if(IS_PARAM_SETS_NALS(pCurNalHead->eNalUnitType)){
		FATAL("WTF");
	}
	return pNalPayload;
}

PlainH264Decoder::PlainH264Decoder() {
	m_pCtx=0;
	m_iFrameNumber=-1;
	m_frameNumber=0;
	m_intraFrameIndex=0;											// Count since last I frame
	m_videoWidth=0;
	m_videoHeight=0;
	m_blockWidth=0;													// m_videoWidth/16
	m_blockHeight=0;												// m_videoHeight/16
	CreateMalloc();
}

PlainH264Decoder::~PlainH264Decoder(){
	CloseDecoder(m_pCtx);
	Free(m_pCtx);
	m_pCtx=NULL;
	DestroyMalloc();
}

#define MIN_ACCESS_UNIT_CAPACITY (1024*1024)		// Min AU capacity in bytes: 1024 KB predefined

bool PlainH264Decoder::Initialize(const SDecodingParam* pParam){
	uprintf( "PlainH264Decoder::Initialize()");
	m_pCtx=(SDecoderContext*)Mallocz(sizeof(SDecoderContext));
	m_pCtx->pDec=NULL;
	InitPredFunc(m_pCtx);
	int32_t iMaxBsBufferSizeInByte=MIN_ACCESS_UNIT_CAPACITY;
	m_pCtx->sRawData.pHead=(uint8_t*)Mallocz(iMaxBsBufferSizeInByte);
	m_pCtx->sRawData.pCurPos=m_pCtx->sRawData.pHead;
	m_pCtx->sRawData.pEnd=m_pCtx->sRawData.pHead+iMaxBsBufferSizeInByte;
	memset(&m_pCtx->m_slice,0,sizeof(m_pCtx->m_slice));
	memset(&m_pCtx->m_nalUnit,0,sizeof(m_pCtx->m_nalUnit));
	m_pCtx->m_nalUnit.eNalUnitType=NAL_UNIT_UNSPEC_0;
	return true;
}

bool PlainH264Decoder::DecodeFrame(const unsigned char* kpSrc,const int kiSrcLen,unsigned char** ppDst){
	m_profiler.SetRange(40);
	START_TIMER(decodeFrame,&m_profiler,"decode",0x7fff60);
	ppDst[0]=ppDst[1]=ppDst[2]=NULL;
	int32_t iNalBitSize=0;
	const uint8_t* payload=DecodeBsInit(&iNalBitSize,m_pCtx,kpSrc,kiSrcLen);
	if(!m_frameNumber){
		m_videoWidth=m_pCtx->m_sps.iMbWidth<<4;
		m_videoHeight=m_pCtx->m_sps.iMbHeight<<4;
		m_blockWidth=m_videoWidth>>4;
		m_blockHeight=m_videoHeight>>4;
		m_stride[0]=WELS_ALIGN(m_videoWidth+(PADDING_LENGTH<<1),PICTURE_RESOLUTION_ALIGNMENT);
		m_stride[1]=m_stride[0]>>1;
		m_pCtx->m_pics[0]=AllocPicture(m_pCtx,m_videoWidth,m_videoHeight);
		m_pCtx->m_pics[1]=AllocPicture(m_pCtx,m_videoWidth,m_videoHeight);
	}
	uint32_t memoryBlockPos=m_memoryBlockPos;
	if(m_pCtx->eSliceType==I_SLICE)
		m_iFrameNumber++;
	m_pCtx->pDec=0;
	if(DecodeSliceNal(m_pCtx,payload,iNalBitSize,ppDst)!=ERR_NONE)
		FATAL("Decode failed");
	m_pCtx->pDec=0;
	END_TIMER(decodeFrame,&m_profiler);
	m_profilerDisplay.clear();
	m_profiler.GetDisplayTimers(m_profilerDisplay);
	m_profiler.Reset();
	m_frameNumber++;
	m_memoryBlockPos=memoryBlockPos;				// Reset allocator
	return true;
}























//SCabacDecEngine
const uint8_t g_kuiCabacRangeLps[64][4]={
	{128,176,208,240},{128,167,197,227},{128,158,187,216},{123,150,178,205},{116,142,169,195},{111,135,160,185},{105,128,152,175},{100,122,144,166},
	{95,116,137,158},{90,110,130,150},{85,104,123,142},{81,99,117,135},{77,94,111,128},{73,89,105,122},{69,85,100,116},{66,80,95,110},
	{62,76,90,104},{59,72,86,99},{56,69,81,94},{53,65,77,89},{51,62,73,85},{48,59,69,80},{46,56,66,76},{43,53,63,72},
	{41,50,59,69},{39,48,56,65},{37,45,54,62},{35,43,51,59},{33,41,48,56},{32,39,46,53},{30,37,43,50},{29,35,41,48},
	{27,33,39,45},{26,31,37,43},{24,30,35,41},{23,28,33,39},{22,27,32,37},{21,26,30,35},{20,24,29,33},{19,23,27,31},
	{18,22,26,30},{17,21,25,28},{16,20,23,27},{15,19,22,25},{14,18,21,24},{14,17,20,23},{13,16,19,22},{12,15,18,21},
	{12,14,17,20},{11,14,16,19},{11,13,15,18},{10,12,15,17},{10,12,14,16},{9,11,13,15},{9,11,12,14},{8,10,12,14},
	{8,9,11,13},{7,9,11,12},{7,9,10,12},{7,8,10,11},{6,8,9,11},{6,7,9,10},{6,7,8,9},{2,2,2,2}
};

//Table 9-45 – State transition table
const uint8_t g_kuiStateTransTable[64][2]={
	{0,1},{0,2},{1,3},{2,4},{2,5},{4,6},{4,7},{5,8},{6,9},{7,10},
	{8,11},{9,12},{9,13},{11,14},{11,15},{12,16},{13,17},{13,18},{15,19},{15,20},
	{16,21},{16,22},{18,23},{18,24},{19,25},{19,26},{21,27},{21,28},{22,29},{22,30},
	{23,31},{24,32},{24,33},{25,34},{26,35},{26,36},{27,37},{27,38},{28,39},{29,40},
	{29,41},{30,42},{30,43},{30,44},{31,45},{32,46},{32,47},{33,48},{33,49},{33,50},
	{34,51},{34,52},{35,53},{35,54},{35,55},{36,56},{36,57},{36,58},{37,59},{37,60},
	{37,61},{38,62},{38,62},{63,63}
};

static const uint8_t g_kRenormTable256[256]={
	6,6,6,6,6,6,6,6,
	5,5,5,5,5,5,5,5,
	4,4,4,4,4,4,4,4,
	4,4,4,4,4,4,4,4,
	3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1
};

static const int16_t g_kMvdBinPos2Ctx[8]={0,1,2,3,3,3,3,3};

#define WELS_CABAC_HALF 0x01FE
#define WELS_CABAC_QUARTER 0x0100

bool SCabacDecEngine::InitCabacDecEngineFromBS(SBitStringAux* bitstream){
	int32_t iRemainingBits=-bitstream->iLeftBits;			// bitstream->iLeftBits < 0
	int32_t iRemainingBytes=(iRemainingBits>>3)+2;			// +2: indicating the pre-read 2 bytes
	uint8_t* pCurr=bitstream->pCurBuf-iRemainingBytes;
	if(pCurr>=(bitstream->pEndBuf-1)){
		return false;
	}
	m_uiOffset=((pCurr[0]<<16)|(pCurr[1]<<8)|pCurr[2]);
	m_uiOffset<<=16;
	m_uiOffset|=(pCurr[3]<<8)|pCurr[4];
	m_iBitsLeft=31;
	m_pBuffCurr=pCurr+5;
	m_uiRange=WELS_CABAC_HALF;
	m_pBuffStart=bitstream->pStartBuf;
	m_pBuffEnd=bitstream->pEndBuf;
	bitstream->iLeftBits=0;
	return true;
}

void SCabacDecEngine::InitBsFromCabacDecEngine(SBitStringAux* bitstream) {
	bitstream->iLeftBits=0;
	bitstream->pStartBuf=m_pBuffStart;
	bitstream->pCurBuf=m_pBuffCurr-(m_iBitsLeft>>3);
	bitstream->uiCurBits=0;
	m_uiRange=0;
	m_uiOffset=0;
	m_pBuffEnd=0;
	m_pBuffCurr=0;
	m_iBitsLeft=0;
	m_pBuffStart=0;
}

uint32_t SCabacDecEngine::DecodeUEGLevelCabac(SCabacCtx* pBinCtx,uint32_t& uiCode){
	uiCode=0;
	READ_VERIFY(DecodeBinCabac(pBinCtx,uiCode));
	if(uiCode==0)
		return ERR_NONE;
	uint32_t uiTmp,uiCount=1;
	uiCode=0;
	do{
		READ_VERIFY(DecodeBinCabac(pBinCtx,uiTmp));
		++uiCode;
		++uiCount;
	} while(uiTmp!=0 && uiCount!=13);

	if(uiTmp!=0){
		READ_VERIFY(DecodeExpBypassCabac(0,uiTmp));
		uiCode+=uiTmp+1;
	}
	return ERR_NONE;
}

int32_t SCabacDecEngine::DecodeBypassCabac(uint32_t& uiBinVal){
	int32_t iBitsLeft=m_iBitsLeft;
	uint64_t uiOffset=m_uiOffset;
	uint64_t uiRangeValue;
	if(iBitsLeft<=0){
		uint32_t uiVal=0;
		int32_t iNumBitsRead=0;
		int32_t iErrorInfo=Read32BitsCabac(uiVal,iNumBitsRead);
		uiOffset=(uiOffset<<iNumBitsRead)|uiVal;
		iBitsLeft=iNumBitsRead;
		if(iErrorInfo && iBitsLeft==0){
			return iErrorInfo;
		}
	}
	iBitsLeft--;
	uiRangeValue=(m_uiRange<<iBitsLeft);
	if(uiOffset>=uiRangeValue){
		m_iBitsLeft=iBitsLeft;
		m_uiOffset=uiOffset-uiRangeValue;
		uiBinVal=1;
		return ERR_NONE;
	}
	m_iBitsLeft=iBitsLeft;
	m_uiOffset=uiOffset;
	uiBinVal=0;
	return ERR_NONE;
}

int32_t SCabacDecEngine::DecodeExpBypassCabac(int32_t iCount,uint32_t& uiSymVal){
	uint32_t uiCode;
	int32_t iSymTmp=0;
	int32_t iSymTmp2=0;
	uiSymVal=0;
	do{
		READ_VERIFY(DecodeBypassCabac(uiCode));
		if(uiCode==1){
			iSymTmp+=(1<<iCount);
			++iCount;
		}
	} while(uiCode!=0 && iCount!=16);
	if(iCount==16){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_CABAC_UNEXPECTED_VALUE);
	}
	while(iCount--){
		READ_VERIFY(DecodeBypassCabac(uiCode));
		if(uiCode==1){
			iSymTmp2|=(1<<iCount);
		}
	}
	uiSymVal=(uint32_t)(iSymTmp+iSymTmp2);
	return ERR_NONE;
}

int32_t SCabacDecEngine::DecodeUEGMvCabac(SCabacCtx* pBinCtx,uint32_t iMaxBin,uint32_t& uiCode){
	READ_VERIFY(DecodeBinCabac(pBinCtx+g_kMvdBinPos2Ctx[0],uiCode));
	if(uiCode==0)
		return ERR_NONE;
	uint32_t uiTmp,uiCount=1;
	uiCode=0;
	do{
		READ_VERIFY(DecodeBinCabac(pBinCtx+g_kMvdBinPos2Ctx[uiCount++],uiTmp));
		uiCode++;
	} while(uiTmp!=0 && uiCount!=8);

	if(uiTmp!=0){
		READ_VERIFY(DecodeExpBypassCabac(3,uiTmp));
		uiCode+=(uiTmp+1);
	}
	return ERR_NONE;
}

uint32_t SCabacDecEngine::DecodeUEGLevelCabac(uint32_t uiBin,uint32_t& uiCode) {
	return DecodeUEGLevelCabac(m_cabacCtx+uiBin,uiCode);
}
int32_t SCabacDecEngine::DecodeUEGMvCabac(uint32_t uiBin,uint32_t iMaxBin,uint32_t& uiCode) {
	return DecodeUEGMvCabac(m_cabacCtx+uiBin,iMaxBin,uiCode);
}
int32_t SCabacDecEngine::DecodeUnaryBinCabac(uint32_t uiBin,int32_t iCtxOffset,uint32_t& uiSymVal) {
	return DecodeUnaryBinCabac(m_cabacCtx+uiBin,iCtxOffset,uiSymVal);
}

int32_t SCabacDecEngine::DecodeUnaryBinCabac(SCabacCtx* pBinCtx,int32_t iCtxOffset,uint32_t& uiSymVal){
	uiSymVal=0;
	READ_VERIFY(DecodeBinCabac(pBinCtx,uiSymVal));
	if(uiSymVal==0)
		return ERR_NONE;
	uint32_t uiCode;
	pBinCtx+=iCtxOffset;
	uiSymVal=0;
	do{
		READ_VERIFY(DecodeBinCabac(pBinCtx,uiCode));
		++uiSymVal;
	} while(uiCode!=0);
	return ERR_NONE;
}

bool SCabacDecEngine::DecodeTerminateCabac(){
	uint64_t uiRange=m_uiRange-2;
	uint64_t uiOffset=m_uiOffset;
	if(uiOffset>=(uiRange<<m_iBitsLeft)){
		return true;
	}
	// Renorm
	if(uiRange<WELS_CABAC_QUARTER){
		int32_t iRenorm=g_kRenormTable256[uiRange];
		m_uiRange=(uiRange<<iRenorm);
		m_iBitsLeft-=iRenorm;
		if(m_iBitsLeft<0){
			uint32_t uiVal=0;
			int32_t iNumBitsRead=0;
			int32_t iErrorInfo=Read32BitsCabac(uiVal,iNumBitsRead);
			m_uiOffset=(m_uiOffset<<iNumBitsRead)|uiVal;
			m_iBitsLeft+=iNumBitsRead;
			if(iErrorInfo && m_iBitsLeft<0){
				return true;
			}
		}
	}else{
		m_uiRange=uiRange;
	}
	return false;
}

int32_t SCabacDecEngine::Read32BitsCabac(uint32_t& uiValue,int32_t& iNumBitsRead){
	int32_t iLeftBytes=(int32_t)(m_pBuffEnd-m_pBuffCurr);
	iNumBitsRead=0;
	uiValue=0;
	if(iLeftBytes<=0){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_CABAC_NO_BS_TO_READ);
	}
	switch(iLeftBytes){
		case 3:
			uiValue=((m_pBuffCurr[0])<<16|(m_pBuffCurr[1])<<8|(m_pBuffCurr[2]));
			m_pBuffCurr+=3;
			iNumBitsRead=24;
			break;
		case 2:
			uiValue=((m_pBuffCurr[0])<<8|(m_pBuffCurr[1]));
			m_pBuffCurr+=2;
			iNumBitsRead=16;
			break;
		case 1:
			uiValue=m_pBuffCurr[0];
			m_pBuffCurr+=1;
			iNumBitsRead=8;
			break;
		default:
			uiValue=((m_pBuffCurr[0]<<24)|(m_pBuffCurr[1])<<16|(m_pBuffCurr[2])<<8|(m_pBuffCurr[3]));
			m_pBuffCurr+=4;
			iNumBitsRead=32;
			break;
	}
	return ERR_NONE;
}
void SCabacDecEngine::InitCabac(int32_t iQp,int32_t iModel) {
	for(int32_t i=0;i<CABAC_CONTEXT_COUNT;i++){
		int32_t m=g_kiCabacGlobalContextIdx[i][iModel][0];
		int32_t n=g_kiCabacGlobalContextIdx[i][iModel][1];
		int32_t iPreCtxState=WELS_CLIP3((((m*iQp)>>4)+n),1,126);
		if(iPreCtxState<=63){
			m_cabacCtx[i].uiState=63-iPreCtxState;
			m_cabacCtx[i].uiMPS=0;
		}else{
			m_cabacCtx[i].uiState=iPreCtxState-64;
			m_cabacCtx[i].uiMPS=1;
		}
	}
}

int32_t SCabacDecEngine::DecodeBinCabac(uint32_t uiBin,uint32_t& uiBinVal) {
	return DecodeBinCabac(m_cabacCtx+uiBin,uiBinVal);
}

int32_t SCabacDecEngine::DecodeBinCabac(SCabacCtx* pBinCtx,uint32_t& uiBinVal){
	uint32_t uiState=pBinCtx->uiState;
	uiBinVal=pBinCtx->uiMPS;
	uint64_t uiOffset=m_uiOffset;
	uint64_t uiRange=m_uiRange;
	int32_t iRenorm=1;
	uint32_t uiRangeLPS=g_kuiCabacRangeLps[uiState][(uiRange>>6)&0x03];
	uiRange-=uiRangeLPS;
	if(uiOffset>=(uiRange<<m_iBitsLeft)){		// LPS
		uiOffset-=(uiRange<<m_iBitsLeft);
		uiBinVal^=0x0001;
		if(!uiState)
			pBinCtx->uiMPS^=0x01;
		pBinCtx->uiState=g_kuiStateTransTable[uiState][0];
		iRenorm=g_kRenormTable256[uiRangeLPS];
		uiRange=uiRangeLPS<<iRenorm;
	}else{		// MPS
		pBinCtx->uiState=g_kuiStateTransTable[uiState][1];
		if(uiRange>=WELS_CABAC_QUARTER){
			m_uiRange=uiRange;
			return ERR_NONE;
		}else{
			uiRange<<=1;
		}
	}
// Renorm
	m_uiRange=uiRange;
	m_iBitsLeft-=iRenorm;
	if(m_iBitsLeft>0){
		m_uiOffset=uiOffset;
		return ERR_NONE;
	}
	uint32_t uiVal=0;
	int32_t iNumBitsRead=0;
	int32_t iErrorInfo=Read32BitsCabac(uiVal,iNumBitsRead);
	m_uiOffset=(uiOffset<<iNumBitsRead)|uiVal;
	m_iBitsLeft+=iNumBitsRead;
	if(iErrorInfo && m_iBitsLeft<0){
		return iErrorInfo;
	}
	return ERR_NONE;
}

};	// namespace NewDec

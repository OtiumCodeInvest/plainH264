
#include <assert.h>

#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/time.h"
#include "shared/output.h"
#include "encodeimage.h"

//H.264/AVC Advanced Video Coding
//SVC Scalable Video Coding 
//CABAC Context-based Adaptive Binary Arithmetic Coding
//CAVLC Context-adaptive variable-length coding 
//GOP Group of pictures. Pictures within two I s
//SAD Sum of Absolute Differences
//QP Quantization parameter
//VUI Video Usability Information
//MB macroblock
//MD Mode decision?

#ifdef _WIN32
#define ALIGNED_DECLARE( type, var, n ) __declspec(align(n)) type var
#else
#define ALIGNED_DECLARE( type, var, n ) type var __attribute__((aligned(n)))
#endif

#define ALIGN_WITH(x,n) (((x)+(n)-1)&~((n)-1))
#define CLIP3(iX,iY,iZ) ((iX)<(iY) ? (iY) : ((iX)>(iZ) ? (iZ) : (iX)))
#define DIV_ROUND(x,y) ((int32_t)((y)==0?((x)/((y)+1)):(((y)/2+(x))/(y))))
#define DIV_ROUND64(x,y) ((int64_t)((y)==0?((x)/((y)+1)):(((y)/2+(x))/(y))))
#define SIGN(a) ((int32_t)(a)>>31)
#define ABS(iX) ((iX)>0 ? (iX) : -(iX))
#define ROUND(x) ((int32_t)(0.5+(x)))

#define MB_WIDTH_LUMA 16
#define MB_WIDTH_CHROMA (MB_WIDTH_LUMA>>1)
#define MB_HEIGHT_LUMA 16
#define MB_HEIGHT_CHROMA (MB_HEIGHT_LUMA>>1)
#define MB_COEFF_LIST_SIZE (256+((MB_WIDTH_CHROMA*MB_HEIGHT_CHROMA)<<1))

#define CAMERA_STARTMV_RANGE (64)
#define ITERATIVE_TIMES (16)
#define CAMERA_MV_RANGE (CAMERA_STARTMV_RANGE+ITERATIVE_TIMES)
#define CAMERA_MVD_RANGE ((CAMERA_MV_RANGE+1)<<1)		//mvd=mv_range*2;

#define REF_NOT_AVAIL -2
#define REF_NOT_IN_LIST -1

#define CLIP3_QP_0_51(q) CLIP3(q,0,51)				// ((q)<(0) ? (0) : ((q)>(51) ? (51) : (q)))

enum ESliceType {
	P_SLICE=0,
	I_SLICE=2,
	UNKNOWN_SLICE=5
};
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

uint32_t g_time[8]={0};
uint32_t g_count[8]={0};
uint64_t g_start[8]={0};
const char* g_names[8]={0};

inline void StartTimer(int ix,const char* name=0) {
	g_names[ix]=name;
	g_start[ix]=GetTimerU64();
}
inline void StopTimer(int ix) {
	uint64_t t=GetTimerU64();
	g_time[ix]+=(uint32_t)(t-g_start[ix]);
	g_count[ix]++;
}
void PrintTimers() {
	for(int i=0;i!=countof(g_time);i++) {
		if(g_count[i]) {
			uint64_t tus=g_time[i];
			uprintf("Timer index=%d time% 9dus count% 8d ave time %.04fus name=%s\n",i,tus,g_count[i],(float)tus/(float)g_count[i],g_names[i]?g_names[i]:"NA");
		}
	}
	memset(g_time,0,sizeof(g_time));
	memset(g_count,0,sizeof(g_count));
	memset(g_start,0,sizeof(g_start));
}

struct SMVUnitXY { // each 4 Bytes
	int16_t iMvX;
	int16_t iMvY;
	inline bool IsZero()const{return *((int32_t*)&iMvX)==0;}
	SMVUnitXY& sDeltaMv(const SMVUnitXY& _v0,const SMVUnitXY& _v1) {
		iMvX=_v0.iMvX-_v1.iMvX;
		iMvY=_v0.iMvY-_v1.iMvY;
		return (*this);
	};
	SMVUnitXY& sAssignMv(const SMVUnitXY& _v0) {
		iMvX=_v0.iMvX;
		iMvY=_v0.iMvY;
		return (*this);
	};
};

enum {
	BLOCK_16x16=0,
	BLOCK_16x8=1,
	BLOCK_8x16=2,
	BLOCK_8x8=3,
	BLOCK_4x4=4,
	BLOCK_8x4=5,
	BLOCK_4x8=6,
	BLOCK_SIZE_ALL=7
};

// Reconstructed Picture definition It is used to express reference picture,also consequent reconstruction picture for output
struct SPicture {
// payload pData
	uint8_t* pBuffer;						// pointer to the first allocated byte,basical offset of pBuffer,dimension:
	uint8_t* pData[3];						// pointer to picture planes respectively
// picture information from pSps
	ESliceType m_type;						// got from sSliceHeader(): eSliceType
	uint32_t* m_uiRefMbType;				// for iMbWidth*iMbHeight
	uint8_t* pRefMbQp;						// for iMbWidth*iMbHeight
	int32_t* pMbSkipSad;					// for iMbWidth*iMbHeight
	SMVUnitXY* sMvList;
};

struct SVAAFrameInfo {
	int32_t* m_sad8x8;     		// sad of 8x8,every 4 in the same 16x16 get together
	uint8_t* m_mad8x8;
	int8_t* m_vaaBackgroundMbFlag;
};

enum {
//virtual gop size
	VGOP_SIZE=8,

//qp information
	GOM_MIN_QP_MODE=12,
	MAX_LOW_BR_QP=42,
	DELTA_QP_BGD_THD=3,
};

#define INT_MULTIPLY 100						// use to multiply in Double to Int Conversion,should be same as AQ_QSTEP_INT_MULTIPLY in WelsVP

// Enumerate the type of level id
typedef enum {
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

#define UNSPECIFIED_BIT_RATE 0				// to do: add detail comment

#define MB_BLOCK4x4_NUM 16
#define MB_BLOCK8x8_NUM 4
#define INTRA_4x4_MODE_NUM 8
#define MB_LUMA_CHROMA_BLOCK4x4_NUM 24

typedef uint32_t Mb_Type;

const uint8_t g_kuiChromaQpTable[52]={
  0,1,2,3,4,5,6,7,8,9,10,11,
  12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,
  28,29,29,30,31,32,32,33,34,34,35,35,36,36,37,37,
  37,38,38,38,39,39,39,39
};

struct MacroBlock {
	Mb_Type uiMbType;						// including MB detailed partition type,number and type of reference list
	uint8_t uiLumaQp;						// uiLumaQp: pPps->iInitialQp+sSliceHeader->delta_qp+mb->dquant.
	inline uint8_t GetChromaQp()const{return g_kuiChromaQpTable[CLIP3_QP_0_51(uiLumaQp)];}
	uint32_t uiChromPredMode;
	int32_t iLumaDQp;
	uint8_t uiCbp;
	int32_t iCbpDc;
	int32_t m_sadCost;
	SMVUnitXY sP16x16Mv;
	SMVUnitXY m_mvd[MB_BLOCK4x4_NUM];		//only for CABAC writing;storage structure the same as sMv,in 4x4 scan order.
	int8_t m_intra4x4PredMode[MB_BLOCK4x4_NUM];
	int8_t m_refIndex[MB_BLOCK8x8_NUM];
	SMVUnitXY m_mv[MB_BLOCK4x4_NUM];
	int8_t m_nonZeroCount[MB_LUMA_CHROMA_BLOCK4x4_NUM];
};

enum eNeighbor {
	N_TOPLEFT=0,
	N_TOP=1,
	N_TOPRIGHT=2,
	N_LEFT=3
};

enum EMbPosition {
	LEFT_MB_POS=0x01,// A
	TOP_MB_POS=0x02,// B
	TOPRIGHT_MB_POS=0x04,// C
};

struct MBWindow : public MacroBlock {
	MacroBlock* m_mb;
	const MacroBlock* m_neighbors[4]={0};
	int16_t iMbX;
	int16_t iMbY;
	int32_t iMbXY;
	inline bool HasNeightbor(eNeighbor n)const {
		return m_neighbors[(int)n] ? true:false;
	}
	inline const MacroBlock* GetNeighbor(eNeighbor n)const {
		return m_neighbors[(int)n];
	}
	bool SetNeighbor(int x,int y,int width,int height,eNeighbor n,const MacroBlock* macroBlocks) {
		if(x<0 || y<0 || x>=width || y>=height)
			return false;
		m_neighbors[(int)n]=macroBlocks+((y%3)*width+x);
		return true;
	}
	void InitTopLeft(int x,int y,int width,int height,MacroBlock* macroBlocks) {
		iMbX=x;
		iMbY=y;
		iMbXY=y*width+x;
		m_mb=macroBlocks+((y%3)*width+x);				// 3 line buffers due to deblocking one line delayed
		SetNeighbor(x+0,y-1,width,height,N_TOP,macroBlocks);
		SetNeighbor(x-1,y,width,height,N_LEFT,macroBlocks);
	}
	void InitFull(int x,int y,int width,int height,MacroBlock* macroBlocks) {
		InitTopLeft(x,y,width,height,macroBlocks);
		SetNeighbor(x-1,y-1,width,height,N_TOPLEFT,macroBlocks);
		SetNeighbor(x+1,y-1,width,height,N_TOPRIGHT,macroBlocks);
	}
};

struct SDCTCoeff {
	int16_t iLumaBlock[16][16];				//based on block4x4 luma DC/AC
	int16_t iLumaI16x16Dc[16];
	int16_t iChromaBlock[8][16];			//based on block4x4  chroma DC/AC
	int16_t iChromaDc[2][4];
};

// Bit-stream auxiliary reading/writing
struct SBitStringAux {
	uint8_t* pStartBuf;						// buffer to start position
	uint8_t* pEndBuf;						// buffer+length
	int32_t iBits;								// count bits of overall bitstreaming input
	uint8_t* pCurBuf;						// current reading position
	uint32_t uiCurBits;
	int32_t iLeftBits;						// count number of available bits left ([1,8]),
// need pointer to next byte start position in case 0 bit left then 8 instead
};

struct SMVComponentUnit {   				// each LIST_0/LIST_1
	SMVUnitXY sMotionVectorCache[5*6-1];	// Luma only: 5 x 6-1=29 D-Words
	int8_t iRefIndexCache[5*6];				// Luma only: 5 x 6=30 bytes
};
struct SMbCache {
	//the followed pData now is promised aligned to 16 bytes
	ALIGNED_DECLARE(SMVComponentUnit,sMvComponents,16);
	ALIGNED_DECLARE(int8_t,iNonZeroCoeffCount[48],16);	// Cache line size
	ALIGNED_DECLARE(int8_t,iIntraPredMode[48],16);
	int32_t iSadCost[4];								// avail 1;unavail 0
	SMVUnitXY  sMbMvp[MB_BLOCK4x4_NUM];					// for write bs
	//for residual decoding (recovery) at the side of Encoder
	int16_t m_coeffLevel[MB_COEFF_LIST_SIZE];
	//malloc memory for prediction
	uint8_t m_skipMb[384];
	uint8_t m_memPredMb[2*256];
	uint8_t* pMemPredLuma;								// inter && intra share same pointer;
	uint8_t* pMemPredChroma;							// inter && intra share same pointer;
	uint8_t* pBestPredIntraChroma;						//Cb:0~63;  Cr:64~127
	//uint8_t m_bufferInterPredMe[4*640];					// [4][400] is enough because only h&v or v&hv or h&hv. but if both h&v&hv is needed when 8 quart pixel,future we have to use [5][400].
	//no scan4[] order,just as memory order to store
	int32_t iSadCostSkip[4];							// avail 1;unavail 0
	bool m_mbTypeSkip[4];								// 1: skip;0: non-skip
	int32_t* pEncSad;
	//for residual encoding at the side of Encoder
	SDCTCoeff m_dct;
	uint8_t uiLumaI16x16Mode;
	bool bCollocatedPredFlag;							// denote if current MB is collocated predicted (MV==0).
	uint32_t m_uiRefMbType;
	uint8_t* m_encMb[3];								// pointer to macroblock encode picture
	uint8_t* m_decMb[3];								// pointer to macroblock this frame decode picture
	uint8_t* m_refMb[3];								// pointer to macroblock previous frame decoded picture
};

#define SPS_LOG2_MAX_FRAME_NUM 15

// NAL Unit Type (5 Bits)
enum ENalUnitType {
	NAL_UNIT_CODED_SLICE=1,
	NAL_UNIT_CODED_SLICE_IDR=5,
	NAL_UNIT_SPS=7,
	NAL_UNIT_PPS=8,
};


#define CABAC_CONTEXT_COUNT 460

enum {CABAC_LOW_WIDTH=sizeof(uint64_t)/sizeof(uint8_t)*8};

struct SStateCtx {
	uint8_t m_uiStateMps;
	uint8_t Mps()const{return m_uiStateMps&1;}
	uint8_t State()const{return m_uiStateMps>>1;}
	void Set(uint8_t uiState,uint8_t uiMps){m_uiStateMps=uiState*2+uiMps;}
};

enum ECtxBlockCat {
	LUMA_DC=0,
	LUMA_AC=1,
	LUMA_4x4=2,
	CHROMA_DC=3,
	CHROMA_AC=4
};

class SCabacCtxEnc {
	public:
		int32_t SpatialWriteMbSynCabac(ESliceType eSliceType,MBWindow* mbWindow,uint8_t* puiLastMbQp,const SMbCache& pMbCache,uint8_t uiChmaI8x8Mode);
		uint8_t* WriteSliceEndSyn();
		void InitSliceCabac(const SBitStringAux& pBs,int32_t iQp,ESliceType eSliceType);
	protected:
		uint8_t* m_pBufCur;
		uint8_t* m_pBufStart;
		uint64_t m_uiLow;
		int32_t m_iRenormCnt;
		int32_t m_iLowBitCnt;
		uint32_t  m_uiRange;
		SStateCtx m_sStateCtx[CABAC_CONTEXT_COUNT];
		void CabacEncodeTerminate(uint32_t uiBin);
		void WriteBlockResidualCabac(const SMbCache& pMbCache,const MBWindow& mbWindow,ECtxBlockCat eCtxBlockCat,int16_t  iIdx,int16_t iNonZeroCount,const int16_t* pBlock,int16_t iEndIdx);
		int16_t GetMbCtxCabac(const SMbCache& pMbCache,const MBWindow& mbWindow,ECtxBlockCat eCtxBlockCat,int16_t iIdx);
		void CabacMbDeltaQp(MBWindow* mbWindow);
		void CabacMbCbp(const MBWindow& mbWindow);
		void CabacSubMbMvd(MBWindow* mbWindow,const SMbCache& pMbCache);
		void CabacSubMbType(MBWindow* pCurMb);
		SMVUnitXY CabacMbMvd(const MBWindow& mbWindow,SMVUnitXY sCurMv,SMVUnitXY sPredMv,int16_t i4x4ScanIdx);
		void CabacMbMvdLx(int32_t sMvd,int32_t iCtx,int32_t iPredMvd);
		void CabacEncodeUeBypass(int32_t iExpBits,uint32_t uiVal);
		void CabacEncodeBypassOne(int32_t uiBin);
		void CabacMbRef(const SMbCache& pMbCache,int16_t iIdx);
		void CabacMbIntraChromaPredMode(const MBWindow& mbWindow,uint8_t uiChmaI8x8Mode);
		void CabacMbType(const MBWindow& mbWindow,const SMbCache& pMbCache,ESliceType eSliceType);
		void MbSkipCabac(MBWindow* mbWindow,ESliceType eSliceType,int16_t bSkipFlag);
		void WriteMbResidualCabac(const SMbCache& sMbCache,MBWindow* mbWindow,uint8_t* puiLastMbQp);
		void CabacEncodeDecision(int32_t iCtx,uint32_t uiBin);
		void CabacEncodeUpdateLow_();
		void CabacEncodeUpdateLowNontrivial_();
};

typedef void (*PCopyFunc)(uint8_t* pDst,int32_t iStrideD,const uint8_t* pSrc,int32_t iStrideS);
typedef void (*PSample4SadCostFunc)(uint8_t*,int32_t,uint8_t*,int32_t,int32_t*);

#define NAL_HEADER_SIZE 4

struct SStrideTables {
	void* m_allocBase;
	int32_t* pStrideDecBlockOffset[1][2];						// [iDid][tid==0][24 x 4]: luma+chroma=24 x 4
	int32_t* m_pStrideEncBlockOffset;							// [iDid][24 x 4]: luma+chroma=24 x 4
	int16_t* pMbIndexX;											// [iDid][iMbX]: map for iMbX in each spatial layer coding
	int16_t* pMbIndexY;											// [iDid][iMbY]: map for iMbY in each spatial layer coding
};

struct SvcRcPersist {
	int32_t m_iInitialQp;						// initial qp			//NC
	int64_t m_iIntraComplexity;					// 255*255(MaxMbSAD)*36864(MaxFS) make the highest bit of 32-bit integer 1
	int32_t m_iIntraMbCount;
	int64_t m_iIntraComplxMean;
	int32_t m_iLastCalculatedQScale;
	int64_t iLinearCmplx;						// *INT_MULTIPLY
	int64_t iFrameCmplxMean;
};

struct SRCSlicing {
	int32_t iComplexityIndexSlice;
	int32_t iCalculatedQpSlice;
	int32_t iTargetBitsSlice;
	int32_t iFrameBitsSlice;
	int32_t iGomBitsSlice;
	int32_t iGomTargetBits;
};

struct SBackgroundOU{
	int32_t iBackgroundFlag;
	int32_t iSAD;
	int32_t iSD;
	int32_t iMAD;
	int32_t iMinSubMad;
	int32_t iMaxDiffSubSd;
};

struct SDeblockingFilter {
	uint8_t* pCsData[3];					// pointer to reconstructed picture pData
	uint8_t uiLumaQP;
	uint8_t uiChromaQP;
};

struct SWelsME {
	uint16_t* pMvdCost;
	uint32_t uiSadPred;
	uint32_t uiSadCost;						// used by ME and RC //max SAD should be max_delta*size+lambda*mvdsize=255*256+91*33*2=65280+6006=71286>(2^16)-1=65535
	uint32_t uiSatdCost;					// satd+lm*nbits
	uint8_t uiBlockSize;					// BLOCK_WxH
	uint8_t* m_pEncMb;
	const uint8_t* m_pRefMb;
	SMVUnitXY sMvp;
	SMVUnitXY sMv;							// output
};

struct SWelsMD {
	int32_t m_iLambda;
	uint16_t* m_pMvdCost;
	int32_t iCostLuma;
	int32_t iSadPredMb;
	uint8_t m_uiRef;						// uiRefIndex appointed by Encoder,used for MC
	int32_t iCostSkipMb;
	struct {
		SWelsME sMe16x16;					// adjust each SWelsME for 8 D-word!
		SWelsME sMe8x8[4];
		SWelsME sMe16x8[2];
		SWelsME sMe8x16[2];
	} sMe;
};

struct SMeRefinePointer {
	uint8_t* pHalfPixH;
	uint8_t* pHalfPixV;
	uint8_t* pHalfPixHV;
	uint8_t* pQuarPixBest;
	uint8_t* pQuarPixTmp;
	PCopyFunc pfCopyBlockByMode;
};

class NewH264SVCEncoder : public ISVCEncoder {
	public:
		NewH264SVCEncoder();
		virtual ~NewH264SVCEncoder();
		virtual int Initialize(const SEncParamBase& param);
		virtual const uint8_t* EncodeFrame(uint32_t* byteSize,const SSourcePicture* pSrcPic);
		virtual void GetProfilerDisplay(std::vector<DisplayTimer>& profilerDisplay){}
	protected:
		void InitEncoderExt();
		void EncodePFrame(const SVAAFrameInfo& vaa,SPicture* pEncPic,SPicture* pDecPic,const SPicture* pRefPic,int32_t numberMbGom,uint32_t frameComplexity);
		static void ExpandReferencingPicture(uint8_t* pData[3],int32_t iWidth,int32_t iHeight,int32_t iStride[3]);
		void DeblockingMbAvcbase(const MBWindow& mbWindow,SDeblockingFilter* pFilter);
		void EncodeIFrame(SPicture* pEncPic,SPicture* pDecPic,const SPicture* pRefPic,int32_t numberMbGom);
		void DeblockingInterMb(SDeblockingFilter* pFilter,const MBWindow& mbWindow,uint8_t uiBS[2][4][4]);
		void FilteringEdgeChromaHV(SDeblockingFilter* pFilter,const MBWindow& mbWindow);
		static void FilteringEdgeChromaH(uint8_t* pPixCb,uint8_t* pPixCr,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS);
		void FilteringEdgeLumaHV(SDeblockingFilter* pFilter,const MBWindow& mbWindow);
		static void FilteringEdgeChromaIntraV(uint8_t* pPixCb,const SDeblockingFilter& filter,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS);
		static void FilteringEdgeChromaIntraH(uint8_t* pPixCb,const SDeblockingFilter& filter,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS);
		static void FilteringEdgeLumaH(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS);
		static void FilteringEdgeLumaV(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS);
		static void FilteringEdgeChromaV(uint8_t* pPixCb,uint8_t* pPixCr,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS);
		static void FilteringEdgeLumaIntraV(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS);
		static void FilteringEdgeLumaIntraH(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS);
		void MdIntraInit(SMbCache* pMbCache,const SPicture* pEncPic,const SPicture* pDecPic,const MBWindow& mbWindow);
		static void FillNeighborCacheIntra(SMbCache* pMbCache,const MBWindow& mbWindow);
		static uint32_t GetNeighborIntra(const MBWindow& mbWindow);
		void MdInterMbRefinement(SWelsMD* pWelsMd,const MBWindow* pCurMb,SMbCache* pMbCache);
		void InterMbEncode(SMbCache* pMbCache,MBWindow* pCurMb);
		void PMbChromaEncode(SMbCache* pMbCache,MBWindow* pCurMb);
		bool MdInterJudgeBGDPskip(MBWindow* windowMb,SMbCache* pMbCache,bool* bKeepSkip,const SPicture* pRefPic,const SVAAFrameInfo& vaa);
		bool CheckChromaCost(const SMbCache& pMbCache,const int32_t iCurMbXy,const SPicture* pRefPic)const;
		static void MdUpdateBGDInfo(SPicture* pDecPic,const MBWindow& windowMb,const bool bCollocatedPredFlag,const int32_t iRefPictureType,const SPicture* pRefPic);
		void RcMbInitGom(MBWindow* windowMb,SRCSlicing* pSOverRc,int32_t numberMbGom,int32_t iMinFrameQp,int32_t iMaxFrameQp);
		static void UpdateP16x8MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int32_t kiPartIdx,const int8_t kiRef,SMVUnitXY* pMv);
		static void UpdateP8x16MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int32_t kiPartIdx,const int8_t kiRef,SMVUnitXY* pMv);
		static void UpdateP16x16MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int8_t kiRef,SMVUnitXY* pMv);
		static void UpdateP8x8MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int32_t kiPartIdx,const int8_t kiRef,SMVUnitXY* pMv);
		void MeRefineFracPixel(uint8_t* pMemPredInterMb,SWelsME* pMe,SMeRefinePointer* pMeRefine,int32_t iWidth,int32_t iHeight);
		int32_t MdP16x16(SMbCache* pMbCache,SWelsMD* pWelsMd,const MBWindow& mbWindow,const SPicture* pRefPic,int32_t iMvRange);
		static int32_t PredictSad(const int8_t* pRefIndexCache,const int32_t* pSadCostCache);
		bool MdPSkipEnc(SWelsMD* pWelsMd,MBWindow* windowMb,SMbCache* pMbCache,const SPicture* pRefPic,int32_t iSadPredSkip);
		static void MdInterDoubleCheckPskip(MBWindow* pCurMb,SMbCache* pMbCache);
		static void EncInterY(MBWindow* pCurMb,SMbCache* pMbCache);
		static bool TryPUVskip(MBWindow* pCurMb,SMbCache* pMbCache,int32_t iUV);
		static bool TryPYskip(MBWindow* pCurMb,SMbCache* pMbCache);
		void MdInterFinePartitionVaa(SMbCache* pMbCache,const SVAAFrameInfo& vaa,SWelsMD* pWelsMd,MBWindow* windowMb,int32_t iBestCost,int32_t iMvRange);
		int32_t MdP8x16(SMbCache* pMbCache,SWelsMD* pWelsMd,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax);
		static void PredInter8x16Mv(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,SMVUnitXY* sMvp);
		int32_t MdP8x8(SMbCache* pMbCache,SWelsMD* pWelsMd,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax);
		static void UpdateP8x8Motion2Cache(SMbCache* pMbCache,int32_t iPartIdx,int8_t pRef,const SMVUnitXY& pMv);
		static void UpdateP8x16Motion2Cache(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,const SMVUnitXY& pMv);
		int32_t MdP16x8(SMbCache* pMbCache,SWelsMD* pWelsMd,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax);
		static void UpdateP16x8Motion2Cache(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,const SMVUnitXY& pMv);
		static void PredInter16x8Mv(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,SMVUnitXY* sMvp);
		void IMbChromaEncode(const MBWindow& pCurMb,SMbCache* pMbCache);
		static void EncRecUV(const MBWindow& pCurMb,SMbCache* pMbCache,int16_t* pRes,int32_t iUV);
		uint8_t MdIntraChroma(SMbCache* pMbCache,int32_t iLambda,const MBWindow& windowMb);
		void EncRecI16x16Y(const MBWindow& pCurMb,SMbCache* pMbCache);
		static void DctMb(int16_t* pRes,uint8_t* pEncMb,int32_t iEncStride,uint8_t* pBestPred);
		int32_t MdI16x16(SMbCache* pMbCache,int32_t iLambda,const MBWindow& windowMb);

		void SliceHeaderWrite(SBitStringAux* pBs,int32_t iGlobalQp,ESliceType eSliceType);
		static void MvdCostInit(uint16_t* pMvdCostInter,const int32_t kiMvdSz);
		void WritePpsNal(SBitStringAux* pBitStringAux);
		int32_t WriteNal(SBitStringAux* pBitStringAux,uint8_t* pBsBuffer,int32_t kiStartPos,const int32_t kiType);
		void WriteSpsNal(SBitStringAux* pBitStringAux,int32_t pSpsIdDelta);
		static int32_t WriteVUI(SBitStringAux* pBitStringAux);
		int32_t InitIFrame(int32_t* iMinFrameQp,int32_t* iMaxFrameQp,int32_t* iTargetBits);
		int32_t InitPFrame(int32_t* iMinFrameQp,int32_t* iMaxFrameQp,int32_t* iTargetBits,uint32_t frameComplexity);
		void EndIFrame(int32_t iAverageFrameQp,int32_t iLayerSize);
		void EndPFrame(int32_t iAverageFrameQp,uint32_t iFrameComplexity,int32_t iLayerSize);
		static void GetEncBlockStrideOffset(int32_t* pBlock,const int32_t kiStrideY,const int32_t kiStrideUV);
		static uint32_t DeblockingBSMarginalMBAvcbase(const MBWindow& pCurMb,const MacroBlock* pNeighMb,int32_t iEdge);
		static void FillNeighborCacheInterWithBGD(SMbCache* pMbCache,const MBWindow& mbWindow,int32_t iMbWidth,const int8_t* pVaaBgMbFlag);
		void DeblockingBSCalc_c(SDeblockingFilter* pFilter,Mb_Type uiCurMbType,const MBWindow& mbWindow);
		void MdInterInit(SMbCache* pMbCache,const SVAAFrameInfo& vaa,SPicture* pDecPic,const MBWindow& mbWindow,const SPicture* pRefPic,int32_t iMvRange);
		static void UpdateNonZeroCountCache(const MBWindow& pMb,SMbCache* pMbCache);
		void OutputPMbWithoutConstructCsRsNoCopy(SMbCache* pMbCache,const MBWindow& mb);
		static int32_t PredictSadSkip(const int8_t* pRefIndexCache,const bool* pMbSkipCache,const int32_t* pSadCostCache);
		static void InitMe(SWelsME* sWelsMe,const SWelsMD& sWelsMd,const int32_t iBlockSize,uint8_t* pEnc,uint8_t* pRef);
		int32_t AllocStrideTables();
		SPicture* AllocPicture(bool bNeedMbInfo);
		void ClearPicture(SPicture* picture,bool clearData,bool clearInfo);
		void FreePicture(SPicture* picture);
		uint32_t AnalyzePictureComplexity(SVAAFrameInfo* vaa,SPicture* pCurPicture,const SPicture* pRefPicture,int32_t iMbNumInGom);
		static void VAACalcSadBgd_c(const uint8_t* pCurData,const uint8_t* pRefData,int32_t iPicWidth,int32_t iPicHeight,int32_t iPicStride,int32_t* pSad8x8,int32_t* pSd8x8,uint8_t* pMad8x8);
		void CopyPicture(SPicture* pDstPic,const SSourcePicture* kpSrc);
		void BackgroundDetection(SVAAFrameInfo* vaa,SPicture* pCurPicture,SPicture* pRefPicture,int* sumOfDiff8x8);
		static inline int32_t CalculateAsdChromaEdge(uint8_t* pOriRef,uint8_t* pOriCur,int32_t iStride);
		static inline bool ForegroundDilation23Luma(SBackgroundOU* pBackgroundOU,SBackgroundOU* pOUNeighbours[]);							//Foreground_Dilation_2_3_Luma
		static inline bool ForegroundDilation23Chroma(int8_t iNeighbourForegroundFlags,int32_t iStartSamplePos,int32_t iPicStrideUV,uint8_t* pCurU,uint8_t* pCurV,uint8_t* pRefU,uint8_t* pRefV);		//Foreground_Dilation_2_3_Chroma
		inline void ForegroundDilation(SBackgroundOU* pBackgroundOU,SBackgroundOU* pOUNeighbours[],int32_t iChromaSampleStartPos,uint8_t* pCurU,uint8_t* pCurV,uint8_t* pRefU,uint8_t* pRefV);
		static inline void BackgroundErosion(SBackgroundOU* pBackgroundOU,SBackgroundOU* pOUNeighbours[]);
		static inline void SetBackgroundMbFlag(int8_t* pBackgroundMbFlag,int32_t iPicWidthInMb,int32_t iBackgroundMbFlag);
		static inline void UpperOUForegroundCheck(SBackgroundOU* pCurOU,int8_t* pBackgroundMbFlag,int32_t iPicWidthInOU,int32_t iPicWidthInMb);
		void ForegroundDilationAndBackgroundErosion(SVAAFrameInfo* vaa,SBackgroundOU* pBackgroundOU,uint8_t* pCurU,uint8_t* pCurV,uint8_t* pRefU,uint8_t* pRefV);
		bool m_firstPFrame=true;
		int m_iFrameNumber=-1;
		int m_frameNumber=0;
		int m_intraFrameIndex=0;											// Count since last I frame
		SPicture* m_refPics[2];
		SPicture* m_cur;
		SPicture* m_prev;
		int m_videoWidth;
		int m_videoHeight;
		int32_t m_stride[3];
		int m_blockWidth;													// m_videoWidth/16
		int m_blockHeight;													// m_videoHeight/16
	private:
		int32_t m_iTargetBitrate;
		SvcRcPersist m_svcRcPersist;
		SStrideTables m_strideTable;										// Const table created in Initialize
		uint8_t* m_frameBs=0;												// restoring bitstream pBuffer of all NALs in a frame
		int32_t m_frameBsSize=0;											// count size of frame bs in bytes allocated
		int32_t m_posBsBuffer=0;											// current writing position of frame bs pBuffer
		Profiler m_profiler;
		std::vector<DisplayTimer> m_profilerDisplay;
		void CopyMb(SPicture* pDstPic,const SPicture* pSrcPic,const MBWindow& mb);
		void CopyMb(SPicture* pDstPic,const uint8_t* pY,const uint8_t* pU,const uint8_t* pV,int32_t sY,int32_t sUV,const MBWindow& mb);
	public:
		virtual void GetProfileDisplay(std::vector<DisplayTimer>* profilerDisplay) {
			*profilerDisplay=m_profilerDisplay;
		}
	private:
		void CreateMalloc();
		void DestroyMalloc();
		void* Malloc(uint32_t byteSize);
		void* Mallocz(uint32_t byteSize);
		void Free(void* p);
		void* m_memoryBlock=0;
		uint32_t m_memoryBlockByteSize;
		uint32_t m_memoryBlockPos;
};
void NewH264SVCEncoder::CreateMalloc() {
	m_memoryBlockByteSize=1024*1024*20;
	m_memoryBlock=MALLOC64(m_memoryBlockByteSize);
	m_memoryBlockPos=0;
}
void NewH264SVCEncoder::DestroyMalloc() {
	FREE64(m_memoryBlock);
}

void* NewH264SVCEncoder::Malloc(uint32_t byteSize) {
	uint32_t byteSizeRoundup=(byteSize+15)&-16;
	if(m_memoryBlockPos+byteSizeRoundup>m_memoryBlockByteSize)
		FATAL("Out of memory");
	void* p=(uint8_t*)m_memoryBlock+m_memoryBlockPos;
	m_memoryBlockPos+=byteSizeRoundup;
	return p;
	//return _aligned_malloc(byteSize,0x10);
}
void* NewH264SVCEncoder::Mallocz(uint32_t byteSize) {
	void* p=Malloc(byteSize);
	memset(p,0,byteSize);
	return p;
}
void NewH264SVCEncoder::Free(void* p){
	//_aligned_free(p);
}

#define WRITE_BE_32(ptr,val) do{ \
	(ptr)[0]=(val)>>24;\
	(ptr)[1]=(val)>>16;\
	(ptr)[2]=(val)>> 8;\
	(ptr)[3]=(val)>> 0;\
}while(0)

const uint32_t g_kuiGolombUELength[256]={
	1,3,3,5,5,5,5,7,7,7,7,7,7,7,7,//14
	9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,//30
	11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,//46
	11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,//62
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,//
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	17
};

#define MAX_MACROBLOCK_SIZE_IN_BYTE     400 //3200/8,3200 is from Annex A.3.1.(n)
#define MAX_MACROBLOCK_SIZE_IN_BYTE_x2 (MAX_MACROBLOCK_SIZE_IN_BYTE<<1)

// initialize bitstream writing
static inline void InitBits(SBitStringAux* pBs,const uint8_t* kpBuf,const int32_t kiSize) {
	uint8_t* ptr=(uint8_t*)kpBuf;
	pBs->pStartBuf=ptr;
	pBs->pCurBuf=ptr;
	pBs->pEndBuf=ptr+kiSize;
	pBs->iLeftBits=32;
	pBs->uiCurBits=0;
}

static inline int32_t BsWriteBits(SBitStringAux* pBitString,int32_t iLen,const uint32_t kuiValue) {
	if(iLen<pBitString->iLeftBits) {
		pBitString->uiCurBits=(pBitString->uiCurBits<<iLen) | kuiValue;
		pBitString->iLeftBits-=iLen;
	}else{
		iLen-=pBitString->iLeftBits;
		pBitString->uiCurBits=(pBitString->uiCurBits<<pBitString->iLeftBits) | (kuiValue>>iLen);
		WRITE_BE_32(pBitString->pCurBuf,pBitString->uiCurBits);
		pBitString->pCurBuf+=4;
		pBitString->uiCurBits=kuiValue&((1<<iLen)-1);
		pBitString->iLeftBits=32-iLen;
	}
	return 0;
}

//Write 1 bit
static inline int32_t BsWriteOneBit(SBitStringAux* pBitString,const uint32_t kuiValue) {
	BsWriteBits(pBitString,1,kuiValue);
	return 0;
}

static inline int32_t BsFlush (SBitStringAux* pBitString) {
	WRITE_BE_32(pBitString->pCurBuf,pBitString->uiCurBits<<pBitString->iLeftBits);
	pBitString->pCurBuf+=4-pBitString->iLeftBits/8;
	pBitString->iLeftBits=32;
	pBitString->uiCurBits=0;
	return 0;
}

//Write unsigned exp golomb codes
static inline int32_t BsWriteUE(SBitStringAux* pBitString,const uint32_t kuiValue) {
	uint32_t iTmpValue=kuiValue+1;
	if(256>kuiValue) {
		BsWriteBits(pBitString,g_kuiGolombUELength[kuiValue],kuiValue+1);
	}else{
		uint32_t n=0;
		if(iTmpValue&0xffff0000) {
			iTmpValue >>=16;
			n+=16;
		}
		if(iTmpValue&0xff00) {
			iTmpValue >>=8;
			n+=8;
		}
		n+=(g_kuiGolombUELength[iTmpValue-1]>>1);
		BsWriteBits(pBitString,(n<<1)+1,kuiValue+1);
	}
	return 0;
}

//Write signed exp golomb codes
static inline int32_t BsWriteSE(SBitStringAux* pBitString,const int32_t kiValue) {
	uint32_t iTmpValue;
	if(kiValue==0) {
		BsWriteOneBit(pBitString,1);
	}else
	if(0<kiValue) {
		iTmpValue=(kiValue<<1)-1;
		BsWriteUE(pBitString,iTmpValue);
	}else{
		iTmpValue=((-kiValue)<<1);
		BsWriteUE(pBitString,iTmpValue);
	}
	return 0;
}
// Write RBSP trailing bits
static inline int32_t BsRbspTrailingBits(SBitStringAux* pBitString) {
	BsWriteOneBit(pBitString,1);
	BsFlush (pBitString);
	return 0;
}

// Write truncated exp golomb codes
static inline void BsWriteTE(SBitStringAux* pBs,const int32_t kiX,const uint32_t kuiValue) {
	if(1==kiX) {
		BsWriteOneBit(pBs,!kuiValue);
	}else{
		BsWriteUE(pBs,kuiValue);
	}
}

// Get size of unsigned exp golomb codes
static inline uint32_t BsSizeUE (const uint32_t kiValue) {
	if(256>kiValue) {
		return g_kuiGolombUELength[kiValue];
	}else{
		uint32_t n=0;
		uint32_t iTmpValue=kiValue+1;
		if(iTmpValue&0xffff0000) {
			iTmpValue >>=16;
			n+=16;
		}
		if(iTmpValue&0xff00) {
			iTmpValue >>=8;
			n+=8;
		}
		n+=(g_kuiGolombUELength[iTmpValue-1]>>1);
		return ((n<<1)+1);
	}
}

// Get size of signed exp golomb codes
static inline uint32_t BsSizeSE(const int32_t kiValue) {
	uint32_t iTmpValue;
	if(kiValue==0) {
		 return 1;
	}else
	if(0<kiValue) {
		iTmpValue=(kiValue<<1)-1;
		return BsSizeUE (iTmpValue);
	}else{
		iTmpValue=((-kiValue)<<1);
		return BsSizeUE (iTmpValue);
	}
}

static inline void BsAlign(SBitStringAux* pBs) {
	if(pBs->iLeftBits&7) {
		pBs->uiCurBits<<=pBs->iLeftBits&7;
		pBs->uiCurBits|=(1<<(pBs->iLeftBits&7))-1;
		pBs->iLeftBits &=~7;
	}
	BsFlush(pBs);
}

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

void WelsI16x16LumaPredV_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	uint8_t i=15;
	const int8_t* kpSrc=(int8_t*)&pRef[-kiStride];
	const uint64_t kuiT1=LD64 (kpSrc);
	const uint64_t kuiT2=LD64 (kpSrc+8);
	uint8_t* pDst=pPred;
	do{
		ST64(pDst,kuiT1);
		ST64(pDst+8,kuiT2);
		pDst+=16;
	}while(i-->0);
}

void WelsI16x16LumaPredH_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	int32_t iStridex15=(kiStride<<4)-kiStride;
	int32_t iPredStride=16;
	int32_t iPredStridex15=240;//(iPredStride<<4)-iPredStride;
	uint8_t i=15;
	do{
		const uint8_t kuiSrc8=pRef[iStridex15-1];
		const uint64_t kuiV64=(uint64_t) (0x0101010101010101ULL*kuiSrc8);
		ST64(&pPred[iPredStridex15],kuiV64);
		ST64(&pPred[iPredStridex15+8],kuiV64);
		iStridex15-=kiStride;
		iPredStridex15-=iPredStride;
	}while(i-->0);
}

#define I4x4_COUNT 4
#define I8x8_COUNT 8
#define I16x16_COUNT 16

static inline void WelsFillingPred8to16_c(uint8_t* pPred,uint8_t* pSrc) {
	ST64(pPred,LD64(pSrc));
	ST64(pPred+8,LD64(pSrc));
}
static inline void WelsFillingPred8x2to16_c(uint8_t* pPred,uint8_t* pSrc) {
	ST64(pPred,LD64(pSrc));
	ST64(pPred+8,LD64(pSrc+8));
}
static inline void WelsFillingPred1to16_c(uint8_t* pPred,const uint8_t kuiSrc) {
	const uint8_t kuiSrc8[8]={kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc};
	ST64(pPred,LD64(kuiSrc8));
	ST64(pPred+8,LD64(kuiSrc8));
}

//ENFORCE_STACK_ALIGN_1D: force 1 dimension local data aligned in stack
#define ENFORCE_STACK_ALIGN_1D(_tp,_nm,_sz,_al) _tp _nm ## _tEmP[(_sz)+(_al)-1];_tp *_nm=_nm ## _tEmP+((_al)-1)-(((uintptr_t)(_nm ## _tEmP+((_al)-1))&((_al)-1))/sizeof(_tp));

#define ENFORCE_STACK_ALIGN_2D(_tp,_nm,_cx,_cy,_al) \
    assert( ((_al) && !((_al)&((_al)-1))) && ((_al) >=sizeof(_tp)) );/*_al should be power-of-2 and >=sizeof(_tp)*/\
    _tp _nm ## _tEmP[(_cx)*(_cy)+(_al)/sizeof(_tp)-1];\
    _tp *_nm ## _tEmP_al=_nm ## _tEmP+((_al)/sizeof(_tp)-1);\
    _nm ## _tEmP_al-=(((uintptr_t)_nm ## _tEmP_al&((_al)-1))/sizeof(_tp));\
    _tp (*_nm)[(_cy)]=(_tp (*)[(_cy)])_nm ## _tEmP_al;


#define I8x8_PRED_STRIDE 8

void WelsIChromaPredV_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	const uint64_t kuiSrc64=LD64(&pRef[-kiStride]);
	ST64(pPred,kuiSrc64);
	ST64(pPred+8 ,kuiSrc64);
	ST64(pPred+16,kuiSrc64);
	ST64(pPred+24,kuiSrc64);
	ST64(pPred+32,kuiSrc64);
	ST64(pPred+40,kuiSrc64);
	ST64(pPred+48,kuiSrc64);
	ST64(pPred+56,kuiSrc64);
}

void WelsIChromaPredH_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	int32_t iStridex7=(kiStride<<3)-kiStride;
	int32_t iI8x8Stridex7=(I8x8_PRED_STRIDE<<3)-I8x8_PRED_STRIDE;
	uint8_t i=7;
	do{
		const uint8_t kuiLeft=pRef[iStridex7-1];// pLeft value
		uint64_t kuiSrc64=(uint64_t) (0x0101010101010101ULL*kuiLeft);
		ST64(pPred+iI8x8Stridex7,kuiSrc64);
		iStridex7-=kiStride;
		iI8x8Stridex7-=I8x8_PRED_STRIDE;
	}while(i-->0);
}

static inline uint8_t WelsClip1(int32_t iX) {
	uint8_t uiTmp=(uint8_t) (((iX)&~255) ? (- (iX)>>31) : (iX));
	return uiTmp;
}

void WelsIChromaPredPlane_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	int32_t iLTshift=0,iTopshift=0,iLeftshift=0,iTopSum=0,iLeftSum=0;
	int32_t i,j;
	const uint8_t* pTop=&pRef[-kiStride];
	const uint8_t* pLeft=&pRef[-1];
	for(i=0;i<4;i++) {
		iTopSum+=(i+1)*(pTop[4+i]-pTop[2-i]);
		iLeftSum+=(i+1)*(pLeft[ (4+i)*kiStride]-pLeft[ (2-i)*kiStride]);
	}
	iLTshift=(pLeft[7*kiStride]+pTop[7])<<4;
	iTopshift=(17*iTopSum+16)>>5;
	iLeftshift=(17*iLeftSum+16)>>5;
	for(i=0;i<8;i++) {
		for(j=0;j<8;j++) {
			pPred[j]=WelsClip1 ((iLTshift+iTopshift*(j-3)+iLeftshift*(i-3)+16)>>5);
		}
		pPred+=I8x8_PRED_STRIDE;
	}
}

void WelsIChromaPredDc_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	const int32_t kuiL1=kiStride-1;
	const int32_t kuiL2=kuiL1+kiStride;
	const int32_t kuiL3=kuiL2+kiStride;
	const int32_t kuiL4=kuiL3+kiStride;
	const int32_t kuiL5=kuiL4+kiStride;
	const int32_t kuiL6=kuiL5+kiStride;
	const int32_t kuiL7=kuiL6+kiStride;
	const uint8_t kuiMean1=(pRef[-kiStride]+pRef[1-kiStride]+pRef[2-kiStride]+pRef[3-kiStride]+pRef[-1]+pRef[kuiL1]+pRef[kuiL2]+pRef[kuiL3]+4)>>3;
	const uint32_t kuiSum2=pRef[4-kiStride]+pRef[5-kiStride]+pRef[6-kiStride]+pRef[7-kiStride];
	const uint32_t kuiSum3=pRef[kuiL4]+pRef[kuiL5]+pRef[kuiL6]+pRef[kuiL7];
	const uint8_t kuiMean2=(kuiSum2+2)>>2;
	const uint8_t kuiMean3=(kuiSum3+2)>>2;
	const uint8_t kuiMean4=(kuiSum2+kuiSum3+4)>>3;
	const uint8_t kuiTopMean[8]={kuiMean1,kuiMean1,kuiMean1,kuiMean1,kuiMean2,kuiMean2,kuiMean2,kuiMean2};
	const uint8_t kuiBottomMean[8]={kuiMean3,kuiMean3,kuiMean3,kuiMean3,kuiMean4,kuiMean4,kuiMean4,kuiMean4};
	const uint64_t kuiTopMean64=LD64 (kuiTopMean);
	const uint64_t kuiBottomMean64=LD64 (kuiBottomMean);
	ST64(pPred,kuiTopMean64);
	ST64(pPred+8 ,kuiTopMean64);
	ST64(pPred+16,kuiTopMean64);
	ST64(pPred+24,kuiTopMean64);
	ST64(pPred+32,kuiBottomMean64);
	ST64(pPred+40,kuiBottomMean64);
	ST64(pPred+48,kuiBottomMean64);
	ST64(pPred+56,kuiBottomMean64);
}

void WelsIChromaPredDcLeft_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	const int32_t kuiL1=kiStride-1;
	const int32_t kuiL2=kuiL1+kiStride;
	const int32_t kuiL3=kuiL2+kiStride;
	const int32_t kuiL4=kuiL3+kiStride;
	const int32_t kuiL5=kuiL4+kiStride;
	const int32_t kuiL6=kuiL5+kiStride;
	const int32_t kuiL7=kuiL6+kiStride;
	const uint8_t kuiTopMean=(pRef[-1]+pRef[kuiL1]+pRef[kuiL2]+pRef[kuiL3]+2)>>2;
	const uint8_t kuiBottomMean=(pRef[kuiL4]+pRef[kuiL5]+pRef[kuiL6]+pRef[kuiL7]+2)>>2;
	const uint64_t kuiTopMean64=(uint64_t) (0x0101010101010101ULL*kuiTopMean);
	const uint64_t kuiBottomMean64=(uint64_t) (0x0101010101010101ULL*kuiBottomMean);
	ST64(pPred,kuiTopMean64);
	ST64(pPred+8 ,kuiTopMean64);
	ST64(pPred+16,kuiTopMean64);
	ST64(pPred+24,kuiTopMean64);
	ST64(pPred+32,kuiBottomMean64);
	ST64(pPred+40,kuiBottomMean64);
	ST64(pPred+48,kuiBottomMean64);
	ST64(pPred+56,kuiBottomMean64);
}

void WelsIChromaPredDcTop_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	const uint8_t kuiMean1=(pRef[-kiStride]+pRef[1-kiStride]+pRef[2-kiStride]+pRef[3-kiStride]+2)>>2;
	const uint8_t kuiMean2=(pRef[4-kiStride]+pRef[5-kiStride]+pRef[6-kiStride]+pRef[7-kiStride]+2)>>2;
	const uint8_t kuiMean[8]={kuiMean1,kuiMean1,kuiMean1,kuiMean1,kuiMean2,kuiMean2,kuiMean2,kuiMean2};
	const uint64_t kuiMean64=LD64 (kuiMean);
	ST64(pPred,kuiMean64);
	ST64(pPred+8,kuiMean64);
	ST64(pPred+16,kuiMean64);
	ST64(pPred+24,kuiMean64);
	ST64(pPred+32,kuiMean64);
	ST64(pPred+40,kuiMean64);
	ST64(pPred+48,kuiMean64);
	ST64(pPred+56,kuiMean64);
}

void WelsIChromaPredDcNA_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	const uint64_t kuiDcValue64=(uint64_t)0x8080808080808080ULL;
	ST64(pPred,kuiDcValue64);
	ST64(pPred+8 ,kuiDcValue64);
	ST64(pPred+16,kuiDcValue64);
	ST64(pPred+24,kuiDcValue64);
	ST64(pPred+32,kuiDcValue64);
	ST64(pPred+40,kuiDcValue64);
	ST64(pPred+48,kuiDcValue64);
	ST64(pPred+56,kuiDcValue64);
}

void WelsI16x16LumaPredPlane_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	int32_t iLTshift=0,iTopshift=0,iLeftshift=0,iTopSum=0,iLeftSum=0;
	int32_t i,j;
	const uint8_t* pTop=&pRef[-kiStride];
	const uint8_t* pLeft=&pRef[-1];
	int32_t iPredStride=16;
	for(i=0;i<8;i++) {
		iTopSum+=(i+1)*(pTop[8+i]-pTop[6-i]);
		iLeftSum+=(i+1)*(pLeft[ (8+i)*kiStride]-pLeft[ (6-i)*kiStride]);
	}
	iLTshift=(pLeft[15*kiStride]+pTop[15])<<4;
	iTopshift=(5*iTopSum+32)>>6;
	iLeftshift=(5*iLeftSum+32)>>6;
	for(i=0;i<16;i++) {
		for(j=0;j<16;j++) {
			pPred[j]=WelsClip1 ((iLTshift+iTopshift*(j-7)+iLeftshift*(i-7)+16)>>5);
		}
		pPred+=iPredStride;
	}
}

void WelsI16x16LumaPredDc_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	int32_t iStridex15=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	do{							//caculate the iMean value
		iSum+=pRef[-1+iStridex15]+pRef[-kiStride+i];
		iStridex15-=kiStride;
	}while(i-->0);
	iMean=(16+iSum)>>5;
	memset(pPred,iMean,256);
}

void WelsI16x16LumaPredDcTop_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	do{							//caculate the iMean value
		iSum+=pRef[-kiStride+i];
	}while(i-->0);
	iMean=(8+iSum)>>4;
	memset(pPred,iMean,256);
}

void WelsI16x16LumaPredDcLeft_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	int32_t iStridex15=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	do{							//caculate the iMean value
		iSum+=pRef[-1+iStridex15];
		iStridex15-=kiStride;
	}while(i-->0);
	iMean=(8+iSum)>>4;
	memset(pPred,iMean,256);
}

void WelsI16x16LumaPredDcNA_c(uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	memset(pPred,0x80,256);
}

/////////intra16x16  Luma
#define I16_PRED_INVALID   -1
#define I16_PRED_V 0
#define I16_PRED_H 1
#define I16_PRED_DC 2
#define I16_PRED_P 3

#define I16_PRED_DC_L 4
#define I16_PRED_DC_T 5
#define I16_PRED_DC_128 6
#define I16_PRED_DC_A 7

#define I4_PRED_INVALID    0
#define I4_PRED_V 0
#define I4_PRED_H 1
#define I4_PRED_DC 2
#define I4_PRED_DDL 3		//diagonal_down_left
#define I4_PRED_DDR 4		//diagonal_down_right
#define I4_PRED_VR 5		//vertical_right
#define I4_PRED_HD 6		//horizon_down
#define I4_PRED_VL 7		//vertical_left
#define I4_PRED_HU 8		//horizon_up

#define I4_PRED_DC_L 9
#define I4_PRED_DC_T 10
#define I4_PRED_DC_128 11

#define I4_PRED_DDL_TOP 12	//right-top replacing by padding rightmost pixel of top
#define I4_PRED_VL_TOP 13	//right-top replacing by padding rightmost pixel of top
#define I4_PRED_A 14

#define C_PRED_INVALID -1
#define C_PRED_DC 0
#define C_PRED_H 1
#define C_PRED_V 2
#define C_PRED_P 3

#define C_PRED_DC_L 4
#define C_PRED_DC_T 5
#define C_PRED_DC_128 6
#define C_PRED_A 7

void WelsILuma16x16Pred_c(uint8_t mode,uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	switch(mode) {
		case I16_PRED_V:
			WelsI16x16LumaPredV_c(pPred,pRef,kiStride);
			break;
		case I16_PRED_H:
			WelsI16x16LumaPredH_c(pPred,pRef,kiStride);
			break;
		case I16_PRED_DC:
			WelsI16x16LumaPredDc_c(pPred,pRef,kiStride);
			break;
		case I16_PRED_P:
			WelsI16x16LumaPredPlane_c(pPred,pRef,kiStride);
			break;
		case I16_PRED_DC_L:
			WelsI16x16LumaPredDcLeft_c(pPred,pRef,kiStride);
			break;
		case I16_PRED_DC_T:
			WelsI16x16LumaPredDcTop_c(pPred,pRef,kiStride);
			break;
		case I16_PRED_DC_128:
			WelsI16x16LumaPredDcNA_c(pPred,pRef,kiStride);
			break;
		default:
			FATAL("WTF");
	}
}

void WelsIChromaPred_c(uint8_t mode,uint8_t* pPred,const uint8_t* pRef,const int32_t kiStride) {
	switch(mode) {
		case C_PRED_DC:
			WelsIChromaPredDc_c(pPred,pRef,kiStride);
			break;
		case C_PRED_H:
			WelsIChromaPredH_c(pPred,pRef,kiStride);
			break;
		case C_PRED_V:
			WelsIChromaPredV_c(pPred,pRef,kiStride);
			break;
		case C_PRED_P:
			WelsIChromaPredPlane_c(pPred,pRef,kiStride);
			break;
		case C_PRED_DC_L:
			WelsIChromaPredDcLeft_c(pPred,pRef,kiStride);
			break;
		case C_PRED_DC_T:
			WelsIChromaPredDcTop_c(pPred,pRef,kiStride);
			break;
		case C_PRED_DC_128:
			WelsIChromaPredDcNA_c(pPred,pRef,kiStride);
			break;
		default:
			FATAL("WTF");
	}
}

void NewH264SVCEncoder::InitMe(SWelsME* sWelsMe,const SWelsMD& sWelsMd,const int32_t iBlockSize,uint8_t* pEnc,uint8_t* pRef) {
	sWelsMe->uiBlockSize=iBlockSize;
	sWelsMe->pMvdCost=sWelsMd.m_pMvdCost;
	sWelsMe->m_pEncMb=pEnc;
	sWelsMe->m_pRefMb=pRef;
}
int32_t SampleSad4x4_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	int32_t iSadSum=0;
	int32_t i=0;
	const uint8_t* pSrc1=pSample1;
	const uint8_t* pSrc2=pSample2;
	for(i=0;i<4;i++) {
		iSadSum+=ABS((pSrc1[0]-pSrc2[0]));
		iSadSum+=ABS((pSrc1[1]-pSrc2[1]));
		iSadSum+=ABS((pSrc1[2]-pSrc2[2]));
		iSadSum+=ABS((pSrc1[3]-pSrc2[3]));
		pSrc1+=iStride1;
		pSrc2+=iStride2;
	}
	return iSadSum;
}
int32_t SampleSad8x4_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	int32_t iSadSum=0;
	iSadSum+=SampleSad4x4_c(pSample1,iStride1,pSample2,iStride2);
	iSadSum+=SampleSad4x4_c(pSample1+4,iStride1,pSample2+4,iStride2);
	return iSadSum;
}
int32_t SampleSad4x8_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	int32_t iSadSum=0;
	iSadSum+=SampleSad4x4_c(pSample1,iStride1,pSample2,iStride2);
	iSadSum+=SampleSad4x4_c(pSample1+(iStride1<<2),iStride1,pSample2+(iStride2<<2),iStride2);
	return iSadSum;
}

int32_t SampleSad8x8_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	int32_t iSadSum=0;
	int32_t i=0;
	const uint8_t* pSrc1=pSample1;
	const uint8_t* pSrc2=pSample2;
	for(i=0;i<8;i++) {
		iSadSum+=ABS((pSrc1[0]-pSrc2[0]));
		iSadSum+=ABS((pSrc1[1]-pSrc2[1]));
		iSadSum+=ABS((pSrc1[2]-pSrc2[2]));
		iSadSum+=ABS((pSrc1[3]-pSrc2[3]));
		iSadSum+=ABS((pSrc1[4]-pSrc2[4]));
		iSadSum+=ABS((pSrc1[5]-pSrc2[5]));
		iSadSum+=ABS((pSrc1[6]-pSrc2[6]));
		iSadSum+=ABS((pSrc1[7]-pSrc2[7]));
		pSrc1+=iStride1;
		pSrc2+=iStride2;
	}
	return iSadSum;
}
int32_t SampleSad16x8_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	int32_t iSadSum=0;
	iSadSum+=SampleSad8x8_c(pSample1,iStride1,pSample2,iStride2);
	iSadSum+=SampleSad8x8_c(pSample1+8,iStride1,pSample2+8,iStride2);
	return iSadSum;
}
int32_t SampleSad8x16_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	int32_t iSadSum=0;
	iSadSum+=SampleSad8x8_c(pSample1,iStride1,pSample2,iStride2);
	iSadSum+=SampleSad8x8_c(pSample1+(iStride1<<3),iStride1,pSample2+(iStride2<<3),iStride2);
	return iSadSum;
}
int32_t SampleSad16x16_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
  int32_t iSadSum=0;
  iSadSum+=SampleSad8x8_c(pSample1,iStride1,pSample2,iStride2);
  iSadSum+=SampleSad8x8_c(pSample1+8,iStride1,pSample2+8, iStride2);
  iSadSum+=SampleSad8x8_c(pSample1+(iStride1<<3),iStride1,pSample2+(iStride2<<3),iStride2);
  iSadSum+=SampleSad8x8_c(pSample1+(iStride1<<3)+8,iStride1,pSample2+(iStride2<<3)+8,iStride2);
  return iSadSum;
}

void SampleSadFour16x16_c(const uint8_t* iSample1,int32_t iStride1,const uint8_t* iSample2,int32_t iStride2,int32_t* pSad) {
	*(pSad)=SampleSad16x16_c(iSample1,iStride1,(iSample2-iStride2),iStride2);
	*(pSad+1)=SampleSad16x16_c(iSample1,iStride1,(iSample2+iStride2),iStride2);
	*(pSad+2)=SampleSad16x16_c(iSample1,iStride1,(iSample2-1),iStride2);
	*(pSad+3)=SampleSad16x16_c(iSample1,iStride1,(iSample2+1),iStride2);
}
void SampleSadFour16x8_c(const uint8_t* iSample1,int32_t iStride1,const uint8_t* iSample2,int32_t iStride2,int32_t* pSad) {
	*(pSad)=SampleSad16x8_c(iSample1,iStride1,(iSample2-iStride2),iStride2);
	*(pSad+1)=SampleSad16x8_c(iSample1,iStride1,(iSample2+iStride2),iStride2);
	*(pSad+2)=SampleSad16x8_c(iSample1,iStride1,(iSample2-1),iStride2);
	*(pSad+3)=SampleSad16x8_c(iSample1,iStride1,(iSample2+1),iStride2);
}
void SampleSadFour8x16_c(const uint8_t* iSample1,int32_t iStride1,const uint8_t* iSample2,int32_t iStride2,int32_t* pSad) {
	*(pSad)=SampleSad8x16_c(iSample1,iStride1,(iSample2-iStride2),iStride2);
	*(pSad+1)=SampleSad8x16_c(iSample1,iStride1,(iSample2+iStride2),iStride2);
	*(pSad+2)=SampleSad8x16_c(iSample1,iStride1,(iSample2-1),iStride2);
	*(pSad+3)=SampleSad8x16_c(iSample1,iStride1,(iSample2+1),iStride2);
}
void SampleSadFour8x8_c(const uint8_t* iSample1,int32_t iStride1,const uint8_t* iSample2,int32_t iStride2,int32_t* pSad) {
	*(pSad)=SampleSad8x8_c(iSample1,iStride1,(iSample2-iStride2),iStride2);
	*(pSad+1)=SampleSad8x8_c(iSample1,iStride1,(iSample2+iStride2),iStride2);
	*(pSad+2)=SampleSad8x8_c(iSample1,iStride1,(iSample2-1),iStride2);
	*(pSad+3)=SampleSad8x8_c(iSample1,iStride1,(iSample2+1),iStride2);
}
int32_t SampleSatd4x4_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	int32_t iSatdSum=0;
	int32_t pSampleMix[4][4]={{ 0 }};
	int32_t iSample0,iSample1,iSample2,iSample3;
	int32_t i=0;
	const uint8_t* pSrc1=pSample1;
	const uint8_t* pSrc2=pSample2;
//step 1: get the difference
	for(i=0;i<4;i++) {
		pSampleMix[i][0]=pSrc1[0]-pSrc2[0];
		pSampleMix[i][1]=pSrc1[1]-pSrc2[1];
		pSampleMix[i][2]=pSrc1[2]-pSrc2[2];
		pSampleMix[i][3]=pSrc1[3]-pSrc2[3];
		pSrc1+=iStride1;
		pSrc2+=iStride2;
	}
//step 2: horizontal transform
	for(i=0;i<4;i++) {
		iSample0=pSampleMix[i][0]+pSampleMix[i][2];
		iSample1=pSampleMix[i][1]+pSampleMix[i][3];
		iSample2=pSampleMix[i][0]-pSampleMix[i][2];
		iSample3=pSampleMix[i][1]-pSampleMix[i][3];
		pSampleMix[i][0]=iSample0+iSample1;
		pSampleMix[i][1]=iSample2+iSample3;
		pSampleMix[i][2]=iSample2-iSample3;
		pSampleMix[i][3]=iSample0-iSample1;
	}
//step 3: vertical transform and get the sum of SATD
	for(i=0;i<4;i++) {
		iSample0=pSampleMix[0][i]+pSampleMix[2][i];
		iSample1=pSampleMix[1][i]+pSampleMix[3][i];
		iSample2=pSampleMix[0][i]-pSampleMix[2][i];
		iSample3=pSampleMix[1][i]-pSampleMix[3][i];
		pSampleMix[0][i]=iSample0+iSample1;
		pSampleMix[1][i]=iSample2+iSample3;
		pSampleMix[2][i]=iSample2-iSample3;
		pSampleMix[3][i]=iSample0-iSample1;
		iSatdSum+=(ABS(pSampleMix[0][i])+ABS(pSampleMix[1][i])+ABS(pSampleMix[2][i])+ABS(pSampleMix[3][i]));
	}
	return ((iSatdSum+1)>>1);
}
int32_t SampleSatd8x8_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	StartTimer(4,"SAT");
	int32_t iSatdSum=0;
	iSatdSum+=SampleSatd4x4_c(pSample1,iStride1,pSample2,iStride2);
	iSatdSum+=SampleSatd4x4_c(pSample1+4,iStride1,pSample2+4,iStride2);
	iSatdSum+=SampleSatd4x4_c(pSample1+(iStride1<<2),iStride1,pSample2+(iStride2<<2),iStride2);
	iSatdSum+=SampleSatd4x4_c(pSample1+(iStride1<<2)+4,iStride1,pSample2+(iStride2<<2)+4,iStride2);
	StopTimer(4);
	return iSatdSum;
}
int32_t SampleSatd16x8_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	StartTimer(4);
	int32_t iSatdSum=0;
	iSatdSum+=SampleSatd8x8_c(pSample1,iStride1,pSample2,iStride2);
	iSatdSum+=SampleSatd8x8_c(pSample1+8,iStride1,pSample2+8,iStride2);
	StopTimer(4);
	return iSatdSum;
}
int32_t SampleSatd8x16_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	StartTimer(4);
	int32_t iSatdSum=0;
	iSatdSum+=SampleSatd8x8_c(pSample1,iStride1,pSample2,iStride2);
	iSatdSum+=SampleSatd8x8_c(pSample1+(iStride1<<3),iStride1,pSample2+(iStride2<<3),iStride2);
	StopTimer(4);
	return iSatdSum;
}
int32_t SampleSatd16x16_c(const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	StartTimer(4);
	int32_t iSatdSum=0;
	iSatdSum+=SampleSatd8x8_c(pSample1,iStride1,pSample2,iStride2);
	iSatdSum+=SampleSatd8x8_c(pSample1+8,iStride1,pSample2+8,iStride2);
	iSatdSum+=SampleSatd8x8_c(pSample1+(iStride1<<3),iStride1,pSample2+(iStride2<<3),iStride2);
	iSatdSum+=SampleSatd8x8_c(pSample1+(iStride1<<3)+8,iStride1,pSample2+(iStride2<<3)+8,iStride2);
	StopTimer(4);
	return iSatdSum;
}

int32_t SampleSatd_c(uint8_t uiBlockSize,const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	switch(uiBlockSize) {
		case BLOCK_16x16:
			return SampleSatd16x16_c(pSample1,iStride1,pSample2,iStride2);
		case BLOCK_8x8:
			return SampleSatd8x8_c(pSample1,iStride1,pSample2,iStride2);
		case BLOCK_16x8:
			return SampleSatd16x8_c(pSample1,iStride1,pSample2,iStride2);
		case BLOCK_8x16:
			return SampleSatd8x16_c(pSample1,iStride1,pSample2,iStride2);
	}
	FATAL("WTF");
	return 0;
}

int32_t SampleSad_c(uint8_t uiBlockSize,const uint8_t* pSample1,int32_t iStride1,const uint8_t* pSample2,int32_t iStride2) {
	switch(uiBlockSize) {
		case BLOCK_16x16:
			return SampleSad16x16_c(pSample1,iStride1,pSample2,iStride2);
		case BLOCK_8x8:
			return SampleSad8x8_c(pSample1,iStride1,pSample2,iStride2);
		case BLOCK_16x8:
			return SampleSad16x8_c(pSample1,iStride1,pSample2,iStride2);
		case BLOCK_8x16:
			return SampleSad8x16_c(pSample1,iStride1,pSample2,iStride2);
	}
	FATAL("WTF");
	return 0;
}

// AVC MB types
#define MB_TYPE_INTRA16x16  0x00000002
#define MB_TYPE_16x16       0x00000008
#define MB_TYPE_16x8        0x00000010
#define MB_TYPE_8x16        0x00000020
#define MB_TYPE_8x8         0x00000040
#define MB_TYPE_SKIP        0x00000100

#define MB_TYPE_INTRA     (MB_TYPE_INTRA16x16)
#define MB_TYPE_INTER     (MB_TYPE_16x16 | MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8 | MB_TYPE_SKIP)
#define IS_INTRA16x16(type) ( MB_TYPE_INTRA16x16==(type) )
#define IS_INTRA(type) ( (type)&MB_TYPE_INTRA )
#define IS_INTER(type) ( (type)&MB_TYPE_INTER )
#define IS_SKIP(type) ( ((type)&MB_TYPE_SKIP)!=0 )

void NewH264SVCEncoder::MdUpdateBGDInfo(SPicture* pDecPic,const MBWindow& windowMb,const bool bCollocatedPredFlag,const int32_t iRefPicType,const SPicture* pRefPic) {
	const int32_t kiMbXY=windowMb.iMbXY;
	if(windowMb.m_mb->uiCbp || I_SLICE==iRefPicType || !bCollocatedPredFlag) {
		pDecPic->pRefMbQp[kiMbXY]=windowMb.m_mb->uiLumaQp;
	}else{
		pDecPic->pRefMbQp[kiMbXY]=pRefPic->pRefMbQp[kiMbXY];
	}
}

//cache element equal to 30
const uint8_t g_kuiCache30ScanIdx[16]={ //mv or uiRefIndex cache scan index,4*4 block as basic unit
	7,8,13,14,
	9,10,15,16,
	19,20,25,26,
	21,22,27,28
};

#define MB_LEFT_BIT             0// add to use in intra up-sample
#define MB_TOP_BIT              1
#define MB_TOPRIGHT_BIT         2

//basic pMv prediction unit for pMv width (4,2,1)
void PredMv(SMVUnitXY* sMvp,const SMVComponentUnit* kpMvComp,int8_t iPartIdx,int8_t iPartW,int32_t iRef) {
	const uint8_t kuiLeftIdx=g_kuiCache30ScanIdx[iPartIdx]-1;
	const uint8_t kuiTopIdx=g_kuiCache30ScanIdx[iPartIdx]-6;
	int32_t iLeftRef=kpMvComp->iRefIndexCache[kuiLeftIdx];
	int32_t iTopRef=kpMvComp->iRefIndexCache[kuiTopIdx];
	int32_t iRightTopRef=kpMvComp->iRefIndexCache[kuiTopIdx+iPartW];
	int32_t iDiagonalRef;
	SMVUnitXY sMvA(kpMvComp->sMotionVectorCache[kuiLeftIdx]);
	SMVUnitXY sMvB(kpMvComp->sMotionVectorCache[kuiTopIdx]);
	SMVUnitXY sMvC;
	if(REF_NOT_AVAIL==iRightTopRef) {
		iDiagonalRef=kpMvComp->iRefIndexCache[ kuiTopIdx-1];	// left_top;
		sMvC=kpMvComp->sMotionVectorCache[kuiTopIdx-1];
	}else{
		iDiagonalRef=iRightTopRef;								// right_top;
		sMvC=kpMvComp->sMotionVectorCache[kuiTopIdx+iPartW];
	}
	if((REF_NOT_AVAIL==iTopRef) && (REF_NOT_AVAIL==iDiagonalRef) && iLeftRef!=REF_NOT_AVAIL) {
		*sMvp=sMvA;
		return;
	}
	// b2[diag] b1[top] b0[left] is available!
	int32_t iMatchRef=(iRef==iLeftRef)<<MB_LEFT_BIT;
	iMatchRef|=(iRef==iTopRef)<<MB_TOP_BIT;
	iMatchRef|=(iRef==iDiagonalRef)<<MB_TOPRIGHT_BIT;
	switch(iMatchRef) {
		case LEFT_MB_POS:		// A
			*sMvp=sMvA;
			break;
		case TOP_MB_POS:		// B
			*sMvp=sMvB;
			break;
		case TOPRIGHT_MB_POS:	// C or D
			*sMvp=sMvC;
			break;
		default:
			sMvp->iMvX=Median(sMvA.iMvX,sMvB.iMvX,sMvC.iMvX);
			sMvp->iMvY=Median(sMvA.iMvY,sMvB.iMvY,sMvC.iMvY);
			break;
	}
}

void PredSkipMv(const SMbCache& pMbCache,SMVUnitXY* sMvp) {
	const SMVComponentUnit* kpMvComp=&pMbCache.sMvComponents;
	const int8_t kiLeftRef=kpMvComp->iRefIndexCache[6];	// A
	const int8_t kiTopRef=kpMvComp->iRefIndexCache[1];	// B
	if(kiLeftRef==REF_NOT_AVAIL || kiTopRef==REF_NOT_AVAIL || (!kiLeftRef && kpMvComp->sMotionVectorCache[6].IsZero()) || (!kiTopRef  && kpMvComp->sMotionVectorCache[1].IsZero())) {
		ST32(sMvp,0);
		return;
	}
	PredMv(sMvp,kpMvComp,0,4,0);

}

#define ABS_LC(a) ((iSign^(int32_t)(a))-iSign)
#define NEW_QUANT(pDct,iFF,iMF) (((iFF)+ABS_LC(pDct))*(iMF)) >>16
#define WELS_NEW_QUANT(pDct,iFF,iMF) ABS_LC(NEW_QUANT(pDct,iFF,iMF))

//Copy functions
void Copy8x8_c(uint8_t* pDst,int32_t iStrideD,const uint8_t* pSrc,int32_t iStrideS) {
	int32_t i;
	for(i=0;i<4;i++) {
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+iStrideD,LD32(pSrc+iStrideS));
		ST32(pDst+iStrideD+4,LD32(pSrc+iStrideS+4));
		pDst+=iStrideD<<1;
		pSrc+=iStrideS<<1;
	}
}
void Copy8x16_c(uint8_t* pDst,int32_t iStrideD,const uint8_t* pSrc,int32_t iStrideS) {
	int32_t i;
	for(i=0;i<8;++i) {
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+iStrideD,LD32(pSrc+iStrideS));
		ST32(pDst+iStrideD+4 ,LD32(pSrc+iStrideS+4));
		pDst+=iStrideD<<1;
		pSrc+=iStrideS<<1;
	}
}
void Copy16x8_c(uint8_t* pDst,int32_t iStrideD,const uint8_t* pSrc,int32_t iStrideS) {
	int32_t i;
	for(i=0;i<8;i++) {
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+8,LD32(pSrc+8));
		ST32(pDst+12,LD32(pSrc+12));
		pDst+=iStrideD;
		pSrc+=iStrideS;
	}
}
void Copy16x16_c(uint8_t* pDst,int32_t iStrideD,const uint8_t* pSrc,int32_t iStrideS) {
	int32_t i;
	for(i=0;i<16;i++) {
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+8,LD32(pSrc+8));
		ST32(pDst+12,LD32(pSrc+12));
		pDst+=iStrideD;
		pSrc+=iStrideS;
	}
}
int32_t WelsHadamardQuant2x2_c(int16_t* pRs,const int16_t iFF,int16_t iMF,int16_t* pDct,int16_t* pBlock) {
	int16_t s[4];
	int32_t iSign,i,iDcNzc=0;
	s[0]=pRs[0]+pRs[32];
	s[1]=pRs[0] -pRs[32];
	s[2]=pRs[16]+pRs[48];
	s[3]=pRs[16]-pRs[48];
	pRs[0]=0;
	pRs[16]=0;
	pRs[32]=0;
	pRs[48]=0;
	pDct[0]=s[0]+s[2];
	pDct[1]=s[0]-s[2];
	pDct[2]=s[1]+s[3];
	pDct[3]=s[1]-s[3];
	iSign=SIGN(pDct[0]);
	pDct[0]=WELS_NEW_QUANT(pDct[0],iFF,iMF);
	iSign=SIGN(pDct[1]);
	pDct[1]=WELS_NEW_QUANT(pDct[1],iFF,iMF);
	iSign=SIGN(pDct[2]);
	pDct[2]=WELS_NEW_QUANT(pDct[2],iFF,iMF);
	iSign=SIGN(pDct[3]);
	pDct[3]=WELS_NEW_QUANT(pDct[3],iFF,iMF);
	ST64(pBlock,LD64 (pDct));
	for(i=0;i<4;i++)
		iDcNzc+=(pBlock[i]!=0);
	return iDcNzc;
}

typedef void (*PWelsMcWidthHeightFunc) (const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight);

static inline void McCopyWidthEq2_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight) {
	int32_t i;
	for(i=0;i<iHeight;i++) { // iWidth==2 only for chroma
		ST16A2(pDst,LD16(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq4_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight) {
	int32_t i;
	for(i=0;i<iHeight;i++) {
		ST32A4 (pDst,LD32(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq8_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight) {
	int32_t i;
	for(i=0;i<iHeight;i++) {
		ST64A8(pDst,LD64 (pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq16_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight) {
	int32_t i;
	for(i=0;i<iHeight;i++) {
		ST64A8 (pDst  ,LD64 (pSrc));
		ST64A8 (pDst+8,LD64 (pSrc+8));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

// Luma sample MC
static inline int32_t HorFilterInput16bit_c(const int16_t* pSrc) {
	int32_t iPix05=pSrc[0]+pSrc[5];
	int32_t iPix14=pSrc[1]+pSrc[4];
	int32_t iPix23=pSrc[2]+pSrc[3];
	return (iPix05-(iPix14*5)+(iPix23*20));
}
// h: iOffset=1/v: iOffset=iSrcStride
static inline int32_t FilterInput8bitWithStride_c(const uint8_t* pSrc,const int32_t kiOffset) {
	const int32_t kiOffset1=kiOffset;
	const int32_t kiOffset2=(kiOffset<<1);
	const int32_t kiOffset3=kiOffset+kiOffset2;
	const uint32_t kuiPix05=*(pSrc-kiOffset2)+*(pSrc+kiOffset3);
	const uint32_t kuiPix14=*(pSrc-kiOffset1)+*(pSrc+kiOffset2);
	const uint32_t kuiPix23=*(pSrc)+* (pSrc+kiOffset1);
	return (kuiPix05-((kuiPix14<<2)+kuiPix14)+(kuiPix23<<4)+(kuiPix23<<2));
}

static inline void PixelAvg_c(uint8_t* pDst,int32_t iDstStride,const uint8_t* pSrcA,int32_t iSrcAStride,const uint8_t* pSrcB,int32_t iSrcBStride,int32_t iWidth,int32_t iHeight) {
	int32_t i,j;
	for(i=0;i<iHeight;i++) {
		for(j=0;j<iWidth;j++) {
			pDst[j]=(pSrcA[j]+pSrcB[j]+1)>>1;
		}
		pDst+=iDstStride;
		pSrcA+=iSrcAStride;
		pSrcB+=iSrcBStride;
	}
}
static inline void McCopy_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	if(iWidth==16)
		McCopyWidthEq16_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else
	if(iWidth==8)
		McCopyWidthEq8_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else
	if(iWidth==4)
		McCopyWidthEq4_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else //here iWidth==2
		McCopyWidthEq2_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
}

//horizontal filter to gain half sample,that is (2,0) location in quarter sample
static inline void McHorVer20_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	int32_t i,j;
	for(i=0;i<iHeight;i++) {
		for(j=0;j<iWidth;j++) {
			pDst[j]=WelsClip1 ((FilterInput8bitWithStride_c(pSrc+j,1)+16)>>5);
		}
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

//vertical filter to gain half sample,that is (0,2) location in quarter sample
static inline void McHorVer02_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	int32_t i,j;
	for(i=0;i<iHeight;i++) {
		for(j=0;j<iWidth;j++) {
			pDst[j]=WelsClip1 ((FilterInput8bitWithStride_c(pSrc+j,iSrcStride)+16)>>5);
		}
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

//horizontal and vertical filter to gain half sample,that is (2,2) location in quarter sample
static inline void McHorVer22_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	int16_t iTmp[17+5];
	int32_t i,j,k;
	for(i=0;i<iHeight;i++) {
		for(j=0;j<iWidth+5;j++) {
			iTmp[j]=FilterInput8bitWithStride_c(pSrc-2+j,iSrcStride);
		}
		for(k=0;k<iWidth;k++) {
			pDst[k]=WelsClip1 ((HorFilterInput16bit_c(&iTmp[k])+512)>>10);
		}
		pSrc+=iSrcStride;
		pDst+=iDstStride;
	}
}

/////////////////////luma MC//////////////////////////
static inline void McHorVer01_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer03_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc+iSrcStride,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer10_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer11_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer12_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiVerTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiVerTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer13_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer21_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiHorTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer23_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiHorTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer30_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiHorTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc+1,iSrcStride,uiHorTmp,16,iWidth,iHeight);
}
static inline void McHorVer31_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer32_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiVerTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiVerTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer33_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight) {
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}

void McLuma_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight)
//pSrc has been added the offset of mv
{
	static const PWelsMcWidthHeightFunc pWelsMcFunc[4][4]={ //[x][y]
		{McCopy_c,McHorVer01_c,McHorVer02_c,McHorVer03_c},
		{McHorVer10_c,McHorVer11_c,McHorVer12_c,McHorVer13_c},
		{McHorVer20_c,McHorVer21_c,McHorVer22_c,McHorVer23_c},
		{McHorVer30_c,McHorVer31_c,McHorVer32_c,McHorVer33_c},
	};
	pWelsMcFunc[iMvX&3][iMvY&3](pSrc,iSrcStride,pDst,iDstStride,iWidth,iHeight);
}

// weight for chroma fraction pixel interpolation
//iA=(8-dx)*(8-dy);
//iB=dx*(8-dy);
//iC=(8-dx)*dy;
//iD=dx*dy
static const uint8_t g_kuiABCD[8][8][4]={ //g_kA[dy][dx],g_kB[dy][dx],g_kC[dy][dx],g_kD[dy][dx]
	{
		{64,0,0,0},{56,8,0,0},{48,16,0,0},{40,24,0,0},
		{32,32,0,0},{24,40,0,0},{16,48,0,0},{8,56,0,0}
	},{
		{56,0,8,0},{49,7,7,1},{42,14,6,2},{35,21,5,3},
		{28,28,4,4},{21,35,3,5},{14,42,2,6},{7,49,1,7}
	},{
		{48,0,16,0},{42,6,14,2},{36,12,12,4},{30,18,10,6},
		{24,24,8,8},{18,30,6,10},{12,36,4,12},{6,42,2,14}
	},{
		{40,0,24,0},{35,5,21,3},{30,10,18,6},{25,15,15,9},
		{20,20,12,12},{15,25,9,15},{10,30,6,18},{5,35,3,21}
	},{
		{32,0,32,0},{28,4,28,4},{24,8,24,8},{20,12,20,12},
		{16,16,16,16},{12,20,12,20},{8,24,8,24},{4,28,4,28}
	},{
		{24,0,40,0},{21,3,35,5},{18,6,30,10},{15,9,25,15},
		{12,12,20,20},{9,15,15,25},{6,18,10,30},{3,21,5,35}
	},{
		{16,0,48,0},{14,2,42,6},{12,4,36,12},{10,6,30,18},
		{8,8,24,24},{6,10,18,30},{4,12,12,36},{2,14,6,42}
	},{
		{8,0,56,0},{7,1,49,7},{6,2,42,14},{5,3,35,21},
		{4,4,28,28},{3,5,21,35},{2,6,14,42},{1,7,7,49}
	}
};

static inline void McChromaWithFragMv_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight) {
	int32_t i,j;
	int32_t iA,iB,iC,iD;
	const uint8_t* pSrcNext=pSrc+iSrcStride;
	const uint8_t* pABCD=g_kuiABCD[iMvY&0x07][iMvX&0x07];
	iA=pABCD[0];
	iB=pABCD[1];
	iC=pABCD[2];
	iD=pABCD[3];
	for(i=0;i<iHeight;i++) {
		for(j=0;j<iWidth;j++) {
			pDst[j]=(iA*pSrc[j]+iB*pSrc[j+1]+iC*pSrcNext[j]+iD*pSrcNext[j+1]+32)>>6;
		}
		pDst+=iDstStride;
		pSrc=pSrcNext;
		pSrcNext+=iSrcStride;
	}
}

void McChroma_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight) {//pSrc has been added the offset of mv
	const int32_t kiD8x=iMvX&0x07;
	const int32_t kiD8y=iMvY&0x07;
	if(0==kiD8x && 0==kiD8y)
		McCopy_c(pSrc,iSrcStride,pDst,iDstStride,iWidth,iHeight);
	else
		McChromaWithFragMv_c(pSrc,iSrcStride,pDst,iDstStride,iMvX,iMvY,iWidth,iHeight);
}

#define INTER_VARIANCE_SAD_THRESHOLD 20

uint8_t MdInterAnalysisVaaInfo_c(const int32_t* pSad8x8) {
	int32_t iSadBlock[4],iAverageSadBlock[4];
	int32_t iAverageSad,iVarianceSad;
	iSadBlock[0]=pSad8x8[0];
	iAverageSad=iSadBlock[0];
	iSadBlock[1]=pSad8x8[1];
	iAverageSad+=iSadBlock[1];
	iSadBlock[2]=pSad8x8[2];
	iAverageSad+=iSadBlock[2];
	iSadBlock[3]=pSad8x8[3];
	iAverageSad+=iSadBlock[3];
	iAverageSad=iAverageSad>>2;
	iAverageSadBlock[0]=(iSadBlock[0]>>6)-(iAverageSad>>6);
	iVarianceSad=iAverageSadBlock[0]*iAverageSadBlock[0];
	iAverageSadBlock[1]=(iSadBlock[1]>>6)-(iAverageSad>>6);
	iVarianceSad+=iAverageSadBlock[1]*iAverageSadBlock[1];
	iAverageSadBlock[2]=(iSadBlock[2]>>6)-(iAverageSad>>6);
	iVarianceSad+=iAverageSadBlock[2]*iAverageSadBlock[2];
	iAverageSadBlock[3]=(iSadBlock[3]>>6)-(iAverageSad>>6);
	iVarianceSad+=iAverageSadBlock[3]*iAverageSadBlock[3];
	if(iVarianceSad<INTER_VARIANCE_SAD_THRESHOLD) {
		return 15;
	}
	uint8_t uiMbSign=0;
	if(iSadBlock[0]>iAverageSad)
		uiMbSign|=0x08;
	if(iSadBlock[1]>iAverageSad)
		uiMbSign|=0x04;
	if(iSadBlock[2]>iAverageSad)
		uiMbSign|=0x02;
	if(iSadBlock[3]>iAverageSad)
		uiMbSign|=0x01;
	return (uiMbSign);
}
void UpdateMbMv_c(SMVUnitXY* pMvBuffer,const SMVUnitXY ksMv) {
	int32_t k=0;
	for(;k<MB_BLOCK4x4_NUM;k+=4) {
		pMvBuffer[k]=pMvBuffer[k+1]=pMvBuffer[k+2]=pMvBuffer[k+3]=ksMv;
	}
}

inline bool IsCostLessEqualSkipCost(int32_t iCurCost,const int32_t iPredPskipSad,const int32_t iRefMbType,const SPicture* pRef,const int32_t iMbXy,const int32_t iSmallestInvisibleTh) {
	return ((iPredPskipSad>iSmallestInvisibleTh && iCurCost>=iPredPskipSad) || (pRef->m_type==P_SLICE && iRefMbType==MB_TYPE_SKIP && pRef->pMbSkipSad[iMbXy]>iSmallestInvisibleTh && iCurCost>=(pRef->pMbSkipSad[iMbXy])));
}

bool NewH264SVCEncoder::CheckChromaCost(const SMbCache& pMbCache,const int32_t iCurMbXy,const SPicture* pRefPic)const {
#define KNOWN_CHROMA_TOO_LARGE 640
#define SMALLEST_INVISIBLE 128 //2*64,2 in pixel maybe the smallest not visible for luma
	const uint8_t* pCbEnc=pMbCache.m_encMb[1];
	const uint8_t* pCrEnc=pMbCache.m_encMb[2];
	const uint8_t* pCbRef=pMbCache.m_refMb[1];
	const uint8_t* pCrRef=pMbCache.m_refMb[2];
	const int32_t iCbEncStride=m_stride[1];
	const int32_t iCrEncStride=m_stride[2];
	const int32_t iChromaRefStride=m_stride[1];
	const int32_t iCbSad=SampleSad8x8_c(pCbEnc,iCbEncStride,pCbRef,iChromaRefStride);
	const int32_t iCrSad=SampleSad8x8_c(pCrEnc,iCrEncStride,pCrRef,iChromaRefStride);
	const bool bChromaTooLarge=(iCbSad>KNOWN_CHROMA_TOO_LARGE || iCrSad>KNOWN_CHROMA_TOO_LARGE);
	const int32_t iChromaSad=iCbSad+iCrSad;
	int32_t iSadPredSkip=PredictSadSkip(pMbCache.sMvComponents.iRefIndexCache,pMbCache.m_mbTypeSkip,pMbCache.iSadCostSkip);
	const bool bChromaCostCannotSkip=IsCostLessEqualSkipCost(iChromaSad,iSadPredSkip,pMbCache.m_uiRefMbType,pRefPic,iCurMbXy,SMALLEST_INVISIBLE);
	return (!bChromaCostCannotSkip && !bChromaTooLarge);
}

bool NewH264SVCEncoder::MdInterJudgeBGDPskip(MBWindow* windowMb,SMbCache* pMbCache,bool* bKeepSkip,const SPicture* pRefPic,const SVAAFrameInfo& vaa) {
	const int32_t kiRefMbQp=pRefPic->pRefMbQp[windowMb->iMbXY];
	int8_t* pVaaBgMbFlag=vaa.m_vaaBackgroundMbFlag+windowMb->iMbXY;
	*bKeepSkip=*bKeepSkip && ((!pVaaBgMbFlag[-1]) && (!pVaaBgMbFlag[-m_blockWidth]) && (!pVaaBgMbFlag[-m_blockWidth+1]));
	if(*pVaaBgMbFlag && !IS_INTRA(pMbCache->m_uiRefMbType) && (kiRefMbQp-windowMb->m_mb->uiLumaQp<=DELTA_QP_BGD_THD || kiRefMbQp<=26)) {
		//the current BGD method uses luma SAD in first step judging of Background blocks and uses chroma edges to confirm the Background blocks
		//HOWEVER,there is such case in SCC, that the luma of two collocated blocks (block in reference frame and in current frame) is very similar but the chroma are very different,at the same time the chroma are plain and without edge
		//IN SUCH A CASE, it will be not proper to just use Pskip TODO: consider reusing this result of ChromaCheck when SCDSkip needs this as well
		if(CheckChromaCost(*pMbCache,windowMb->iMbXY,pRefPic)) {
			return true;
		}
	}
	return false;
}

static inline int32_t BsGetBitsPos(SBitStringAux* pBs) {
	return (int32_t)(((pBs->pCurBuf-pBs->pStartBuf)<<3)+32-pBs->iLeftBits);
}

const int8_t g_kiMapModeIntraChroma[7]={0,1,2,3,0,0,0};
const int8_t g_kiMapModeI16x16[7]={0,1,2,3,2,2,2};

const int32_t g_kiQpCostTable[52]={
	1,1,1,1,1,1,1,1,//  0-7
	1,1,1,1,    //  8-11
	1,1,1,1,2,2,2,2,// 12-19
	3,3,3,4,4,4,5,6,// 20-27
	6,7,8,9,10,11,13,14,// 28-35
	16,18,20,23,25,29,32,36,// 36-43
	40,45,51,57,64,72,81,91 // 44-51
};

const uint8_t g_kuiCache48CountScan4Idx[24]={
	// Luma
	9,10,17,18,		// 1+1*8,2+1*8,1+2*8,2+2*8,
	11,12,19,20,	// 3+1*8,4+1*8,3+2*8,4+2*8,
	25,26,33,34,	// 1+3*8,2+3*8,1+4*8,2+4*8,
	27,28,35,36,	// 3+3*8,4+3*8,3+4*8,4+4*8,
	// Cb
	14,15,			// 6+1*8,7+1*8,
	22,23,			// 6+2*8,7+2*8,
	// Cr
	38,39,			// 6+4*8,7+4*8,
	46,47,			// 6+5*8,7+5*8,
};

//////pNonZeroCount[16+8] mapping scan index
const uint8_t g_kuiMbCountScan4Idx[24]={
         		//  0   1 | 4  5      luma 8*8 block           pNonZeroCount[16+8]
  0,1,4,5,//  2   3 | 6  7        0  |  1                  0   1   2   3
  2,3,6,7,//---------------      ---------                 4   5   6   7
  8,9,12,13,//  8   9 | 12 13       2  |  3                  8   9  10  11
  10,11,14,15,// 10  11 | 14 15-----------------------------> 12  13  14  15
  16,17,20,21,//----------------    chroma 8*8 block          16  17  18  19
  18,19,22,23   // 16  17 | 20 21        0    1                 20  21  22  23
};


#define  CTX_NA 0

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


const int8_t g_kiClz5Table[32]={6,5,4,4,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

//Table 9-44 – Specification of rangeTabLPS depending on pStateIdx and qCodIRangeIdx
const uint8_t g_kuiCabacRangeLps[64][4]={
	{ 128,176,208,240},{ 128,167,197,227},{ 128,158,187,216},{ 123,150,178,205},{ 116,142,169,195},{ 111,135,160,185},{ 105,128,152,175},{ 100,122,144,166},
	{  95,116,137,158},{  90,110,130,150},{  85,104,123,142},{  81,99,117,135},{  77,94,111,128},{  73,89,105,122},{  69,85,100,116},{  66,80,95,110},
	{  62,76,90,104},{  59,72,86,99},{  56,69,81,94},{  53,65,77,89},{  51,62,73,85},{  48,59,69,80},{  46,56,66,76},{  43,53,63,72},
	{  41,50,59,69},{  39,48,56,65},{  37,45,54,62},{  35,43,51,59},{  33,41,48,56},{  32,39,46,53},{  30,37,43,50},{  29,35,41,48},
	{  27,33,39,45},{  26,31,37,43},{  24,30,35,41},{  23,28,33,39},{  22,27,32,37},{  21,26,30,35},{  20,24,29,33},{  19,23,27,31},
	{  18,22,26,30},{  17,21,25,28},{  16,20,23,27},{  15,19,22,25},{  14,18,21,24},{  14,17,20,23},{  13,16,19,22},{  12,15,18,21},
	{  12,14,17,20},{  11,14,16,19},{  11,13,15,18},{  10,12,15,17},{  10,12,14,16},{   9,11,13,15},{   9,11,12,14},{   8,10,12,14},
	{   8,9,11,13},{   7,9,11,12},{   7,9,10,12},{   7,8,10,11},{   6,8,9,11},{   6,7,9,10},{   6,7,8,9},{   2,2,2,2}
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


int32_t WelsCalNonZeroCount2x2Block(const int16_t* pBlock) {
	return (pBlock[0]!=0)+(pBlock[1]!=0)+(pBlock[2]!=0)+(pBlock[3]!=0);
}

// dc value pick up and hdm_4x4
void WelsHadamardT4Dc_c(int16_t* pLumaDc,int16_t* pDct) {
	int32_t p[16],s[4];
	int32_t i,iIdx;
	for(i=0;i<16;i+=4) {
		iIdx=((i&0x08)<<4)+((i&0x04)<<3);
		s[0]=pDct[iIdx]+pDct[iIdx+80];
		s[3]=pDct[iIdx]-pDct[iIdx+80];
		s[1]=pDct[iIdx+16]+pDct[iIdx+64];
		s[2]=pDct[iIdx+16]-pDct[iIdx+64];
		p[i]=s[0]+s[1];
		p[i+2]=s[0]-s[1];
		p[i+1]=s[3]+s[2];
		p[i+3]=s[3]-s[2];
	}
	for(i=0;i<4;i++) {
		s[0]=p[i]+p[i+12];
		s[3]=p[i]-p[i+12];
		s[1]=p[i+4]+p[i+8];
		s[2]=p[i+4]-p[i+8];
		pLumaDc[i]=CLIP3((s[0]+s[1]+1)>>1,-32768,32767);
		pLumaDc[i+8]=CLIP3((s[0]-s[1]+1)>>1,-32768,32767);
		pLumaDc[i+4]=CLIP3((s[3]+s[2]+1)>>1,-32768,32767);
		pLumaDc[i+12]=CLIP3((s[3]-s[2]+1)>>1,-32768,32767);
	}
}
int32_t WelsHadamardQuant2x2Skip_c(int16_t* pRs,int16_t iFF,int16_t iMF) {
	int16_t pDct[4],s[4];
	int16_t iThreshold=((1<<16)-1)/iMF-iFF;
	s[0]=pRs[0]+pRs[32];
	s[1]=pRs[0] -pRs[32];
	s[2]=pRs[16]+pRs[48];
	s[3]=pRs[16]-pRs[48];
	pDct[0]=s[0]+s[2];
	pDct[1]=s[0]-s[2];
	pDct[2]=s[1]+s[3];
	pDct[3]=s[1]-s[3];
	return ((ABS(pDct[0])>iThreshold) || (ABS(pDct[1])>iThreshold) || (ABS(pDct[2])>iThreshold) || (ABS(pDct[3])>iThreshold));
}

//DCT functions
void WelsDctT4_c(int16_t* pDct,uint8_t* pPixel1,int32_t iStride1,uint8_t* pPixel2,int32_t iStride2) {
	int16_t i,pData[16],s[4];
	for(i=0;i<16;i+=4) {
		const int32_t kiI1=1+i;
		const int32_t kiI2=2+i;
		const int32_t kiI3=3+i;
		pData[i]=pPixel1[0]-pPixel2[0];
		pData[kiI1]=pPixel1[1]-pPixel2[1];
		pData[kiI2]=pPixel1[2]-pPixel2[2];
		pData[kiI3]=pPixel1[3]-pPixel2[3];
		pPixel1+=iStride1;
		pPixel2+=iStride2;
// horizontal transform
		s[0]=pData[i]+pData[kiI3];
		s[3]=pData[i]-pData[kiI3];
		s[1]=pData[kiI1]+pData[kiI2];
		s[2]=pData[kiI1]-pData[kiI2];
		pDct[i]=s[0]+s[1];
		pDct[kiI2]=s[0]-s[1];
		pDct[kiI1]=(s[3]*(1<<1))+s[2];
		pDct[kiI3]=s[3]-(s[2]*(1<<1));
	}
// vertical transform
	for(i=0;i<4;i++) {
		const int32_t kiI4=4+i;
		const int32_t kiI8=8+i;
		const int32_t kiI12=12+i;
		s[0]=pDct[i]+pDct[kiI12];
		s[3]=pDct[i]-pDct[kiI12];
		s[1]=pDct[kiI4]+pDct[kiI8];
		s[2]=pDct[kiI4]-pDct[kiI8];
		pDct[i]=s[0]+s[1];
		pDct[kiI8]=s[0]-s[1];
		pDct[kiI4]=(s[3]*(1<<1))+s[2];
		pDct[kiI12]=s[3]-(s[2]*(1<<1));
	}
}

void WelsDctFourT4_c(int16_t* pDct,uint8_t* pPixel1,int32_t iStride1,uint8_t* pPixel2,int32_t iStride2) {
  int32_t stride_1=iStride1<<2;
  int32_t stride_2=iStride2<<2;
  WelsDctT4_c(pDct,&pPixel1[0],iStride1,&pPixel2[0],iStride2);
  WelsDctT4_c(pDct+16,&pPixel1[4],iStride1,&pPixel2[4],iStride2);
  WelsDctT4_c(pDct+32,&pPixel1[stride_1],iStride1,&pPixel2[stride_2],iStride2);
  WelsDctT4_c(pDct+48,&pPixel1[stride_1+4],iStride1,&pPixel2[stride_2+4],iStride2);
}

// Scan and Score functions
void WelsScan4x4DcAc_c(int16_t* pLevel,int16_t* pDct) {
	ST32(pLevel,LD32(pDct));
	pLevel[2]=pDct[4];
	pLevel[3]=pDct[8];
	pLevel[4]=pDct[5];
	ST32(pLevel+5,LD32(pDct+2));
	pLevel[7]=pDct[6];
	pLevel[8]=pDct[9];
	ST32(pLevel+9,LD32(pDct+12));
	pLevel[11]=pDct[10];
	pLevel[12]=pDct[7];
	pLevel[13]=pDct[11];
	ST32(pLevel+14,LD32(pDct+14));
}

void WelsScan4x4Ac_c(int16_t* pLevel,int16_t* pDct) {
	pLevel[0]=pDct[1];
	pLevel[1]=pDct[4];
	pLevel[2]=pDct[8];
	pLevel[3]=pDct[5];
	ST32(&pLevel[4],LD32(&pDct[2]));
	pLevel[6]=pDct[6];
	pLevel[7]=pDct[9];
	ST32(&pLevel[8],LD32(&pDct[12]));
	pLevel[10]=pDct[10];
	pLevel[11]=pDct[7];
	pLevel[12]=pDct[11];
	ST32(&pLevel[13],LD32(&pDct[14]));
	pLevel[15]=0;
}

//refer to JVT-O079
int32_t WelsCalculateSingleCtr4x4_c(int16_t* pDct) {
	static const int32_t kiTRunTable[16]={ 3,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0 };
	int32_t iSingleCtr=0;
	int32_t iIdx=15;
	int32_t iRun;
	while(iIdx >=0 && pDct[iIdx]==0)
		--iIdx;
	while(iIdx>=0) {
		--iIdx;
		iRun=iIdx;
		while(iIdx >=0 && pDct[iIdx]==0)
			--iIdx;
		iRun-=iIdx;
		iSingleCtr+=kiTRunTable[iRun];
	}
	return iSingleCtr;
}

int32_t WelsGetNoneZeroCount_c(const int16_t* pLevel) {
	int32_t iCnt=0;
	int32_t iIdx=0;
	while(iIdx<16) {
		iCnt+=(pLevel[iIdx]==0);
		iCnt+=(pLevel[1+iIdx]==0);
		iCnt+=(pLevel[2+iIdx]==0);
		iCnt+=(pLevel[3+iIdx]==0);
		iIdx+=4;
	}
	return (16-iCnt);
}

// HDM and Quant functions
void WelsQuant4x4_c(int16_t* pDct,const int16_t* pFF,const int16_t* pMF) {
	int32_t i,j,iSign;
	for(i=0;i<16;i+=4) {
		j=i&0x07;
		iSign=SIGN(pDct[i]);
		pDct[i]=WELS_NEW_QUANT(pDct[i],pFF[j],pMF[j]);
		iSign=SIGN(pDct[i+1]);
		pDct[i+1]=WELS_NEW_QUANT(pDct[i+1],pFF[j+1],pMF[j+1]);
		iSign=SIGN(pDct[i+2]);
		pDct[i+2]=WELS_NEW_QUANT(pDct[i+2],pFF[j+2],pMF[j+2]);
		iSign=SIGN(pDct[i+3]);
		pDct[i+3]=WELS_NEW_QUANT(pDct[i+3],pFF[j+3],pMF[j+3]);
	}
}

void WelsQuant4x4Dc_c(int16_t* pDct,int16_t iFF,int16_t iMF) {
	int32_t i,iSign;
	for(i=0;i<16;i+=4) {
		iSign=SIGN(pDct[i]);
		pDct[i]=WELS_NEW_QUANT(pDct[i],iFF,iMF);
		iSign=SIGN(pDct[i+1]);
		pDct[i+1]=WELS_NEW_QUANT(pDct[i+1],iFF,iMF);
		iSign=SIGN(pDct[i+2]);
		pDct[i+2]=WELS_NEW_QUANT(pDct[i+2],iFF,iMF);
		iSign=SIGN(pDct[i+3]);
		pDct[i+3]=WELS_NEW_QUANT(pDct[i+3],iFF,iMF);
	}
}

void WelsQuantFour4x4_c(int16_t* pDct,const int16_t* pFF,const int16_t* pMF) {
	int32_t i,j,iSign;
	for(i=0;i<64;i+=4) {
		j=i&0x07;
		iSign=SIGN(pDct[i]);
		pDct[i]=WELS_NEW_QUANT(pDct[i],pFF[j],pMF[j]);
		iSign=SIGN(pDct[i+1]);
		pDct[i+1]=WELS_NEW_QUANT(pDct[i+1],pFF[j+1],pMF[j+1]);
		iSign=SIGN(pDct[i+2]);
		pDct[i+2]=WELS_NEW_QUANT(pDct[i+2],pFF[j+2],pMF[j+2]);
		iSign=SIGN(pDct[i+3]);
		pDct[i+3]=WELS_NEW_QUANT(pDct[i+3],pFF[j+3],pMF[j+3]);
	}
}
void WelsQuantFour4x4Max_c(int16_t* pDct,const int16_t* pFF,const int16_t* pMF,int16_t* pMax) {
	int32_t i,j,k,iSign;
	int16_t iMaxAbs;
	for(k=0;k<4;k++) {
		iMaxAbs=0;
		for(i=0;i<16;i++) {
			j=i&0x07;
			iSign=SIGN(pDct[i]);
			pDct[i]=NEW_QUANT(pDct[i],pFF[j],pMF[j]);
			if(iMaxAbs<pDct[i]) iMaxAbs=pDct[i];
			pDct[i]=ABS_LC(pDct[i]);
		}
		pDct+=16;
		pMax[k]=iMaxAbs;
	}
}


// Dequant and Ihdm functions
void WelsIHadamard4x4Dc (int16_t* pRes) { //pBuffer size : 4x4
	int16_t iTemp[4];
	int32_t i=4;
	while(--i >=0) {
		const int32_t kiIdx=i<<2;
		const int32_t kiIdx1=1+kiIdx;
		const int32_t kiIdx2=1+kiIdx1;
		const int32_t kiIdx3=1+kiIdx2;
		iTemp[0]=pRes[kiIdx]+pRes[kiIdx2];
		iTemp[1]=pRes[kiIdx]-pRes[kiIdx2];
		iTemp[2]=pRes[kiIdx1]-pRes[kiIdx3];
		iTemp[3]=pRes[kiIdx1]+pRes[kiIdx3];
		pRes[kiIdx]=iTemp[0]+iTemp[3];
		pRes[kiIdx1]=iTemp[1]+iTemp[2];
		pRes[kiIdx2]=iTemp[1]-iTemp[2];
		pRes[kiIdx3]=iTemp[0]-iTemp[3];
	}
	i=4;
	while(--i>=0) {
		const int32_t kiI4=4+i;
		const int32_t kiI8=4+kiI4;
		const int32_t kiI12=4+kiI8;
		iTemp[0]=pRes[i]+pRes[kiI8];
		iTemp[1]=pRes[i]-pRes[kiI8];
		iTemp[2]=pRes[kiI4]-pRes[kiI12];
		iTemp[3]=pRes[kiI4]+pRes[kiI12];
		pRes[i]=iTemp[0]+iTemp[3];
		pRes[kiI4]=iTemp[1]+iTemp[2];
		pRes[kiI8]=iTemp[1]-iTemp[2];
		pRes[kiI12]=iTemp[0]-iTemp[3];
	}
}

ALIGNED_DECLARE (const uint16_t,g_kuiDequantCoeff[52][8],16)={
	/* 0*/{   10,13,10,13,13,16,13,16 },/* 1*/{   11,14,11,14,14,18,14,18 },
	/* 2*/{   13,16,13,16,16,20,16,20 },/* 3*/{   14,18,14,18,18,23,18,23 },
	/* 4*/{   16,20,16,20,20,25,20,25 },/* 5*/{   18,23,18,23,23,29,23,29 },
	/* 6*/{   20,26,20,26,26,32,26,32 },/* 7*/{   22,28,22,28,28,36,28,36 },
	/* 8*/{   26,32,26,32,32,40,32,40 },/* 9*/{   28,36,28,36,36,46,36,46 },
	/*10*/{   32,40,32,40,40,50,40,50 },/*11*/{   36,46,36,46,46,58,46,58 },
	/*12*/{   40,52,40,52,52,64,52,64 },/*13*/{   44,56,44,56,56,72,56,72 },
	/*14*/{   52,64,52,64,64,80,64,80 },/*15*/{   56,72,56,72,72,92,72,92 },
	/*16*/{   64,80,64,80,80,100,80,100 },/*17*/{   72,92,72,92,92,116,92,116 },
	/*18*/{   80,104,80,104,104,128,104,128 },/*19*/{   88,112,88,112,112,144,112,144 },
	/*20*/{  104,128,104,128,128,160,128,160 },/*21*/{  112,144,112,144,144,184,144,184 },
	/*22*/{  128,160,128,160,160,200,160,200 },/*23*/{  144,184,144,184,184,232,184,232 },
	/*24*/{  160,208,160,208,208,256,208,256 },/*25*/{  176,224,176,224,224,288,224,288 },
	/*26*/{  208,256,208,256,256,320,256,320 },/*27*/{  224,288,224,288,288,368,288,368 },
	/*28*/{  256,320,256,320,320,400,320,400 },/*29*/{  288,368,288,368,368,464,368,464 },
	/*30*/{  320,416,320,416,416,512,416,512 },/*31*/{  352,448,352,448,448,576,448,576 },
	/*32*/{  416,512,416,512,512,640,512,640 },/*33*/{  448,576,448,576,576,736,576,736 },
	/*34*/{  512,640,512,640,640,800,640,800 },/*35*/{  576,736,576,736,736,928,736,928 },
	/*36*/{  640,832,640,832,832,1024,832,1024 },/*37*/{  704,896,704,896,896,1152,896,1152 },
	/*38*/{  832,1024,832,1024,1024,1280,1024,1280 },/*39*/{  896,1152,896,1152,1152,1472,1152,1472 },
	/*40*/{ 1024,1280,1024,1280,1280,1600,1280,1600 },/*41*/{ 1152,1472,1152,1472,1472,1856,1472,1856 },
	/*42*/{ 1280,1664,1280,1664,1664,2048,1664,2048 },/*43*/{ 1408,1792,1408,1792,1792,2304,1792,2304 },
	/*44*/{ 1664,2048,1664,2048,2048,2560,2048,2560 },/*45*/{ 1792,2304,1792,2304,2304,2944,2304,2944 },
	/*46*/{ 2048,2560,2048,2560,2560,3200,2560,3200 },/*47*/{ 2304,2944,2304,2944,2944,3712,2944,3712 },
	/*48*/{ 2560,3328,2560,3328,3328,4096,3328,4096 },/*49*/{ 2816,3584,2816,3584,3584,4608,3584,4608 },
	/*50*/{ 3328,4096,3328,4096,4096,5120,4096,5120 },/*51*/{ 3584,4608,3584,4608,4608,5888,4608,5888 },
};

ALIGNED_DECLARE (const uint16_t,g_kuiDequantCoeff8x8[52][64],16)={
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==0
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==1
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==2
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==3
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==4
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==5
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==6
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==7
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==8
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==9
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==10
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==11
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==12
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==13
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==14
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==15
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==16
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==17
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==18
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==19
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==20
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==21
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==22
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==23
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==24
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==25
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==26
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==27
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==28
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==29
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==30
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==31
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==32
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==33
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==34
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==35
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==36
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==37
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==38
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==39
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==40
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==41
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==42
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==43
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==44
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==45
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		//QP==46
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		//QP==47
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		//QP==48
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		//QP==49
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		//QP==50
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		//QP==51
};

// for qp<12
void WelsDequantLumaDc4x4(int16_t* pRes,const int32_t kiQp) {
	int32_t i=15;
	const uint16_t kuiDequantValue=g_kuiDequantCoeff[kiQp%6][0];
	const int16_t kiQF0=kiQp/6;
	const int16_t kiQF1=2-kiQF0;
	const int16_t kiQF0S=1<<(1-kiQF0);
	while(i >=0) {
		pRes[i]=(pRes[i]  *kuiDequantValue+kiQF0S)>>kiQF1;
		pRes[i-1]=(pRes[i-1]*kuiDequantValue+kiQF0S)>>kiQF1;
		pRes[i-2]=(pRes[i-2]*kuiDequantValue+kiQF0S)>>kiQF1;
		pRes[i-3]=(pRes[i-3]*kuiDequantValue+kiQF0S)>>kiQF1;
		i-=4;
	}
}

// for qp >=12
void WelsDequantIHadamard4x4_c(int16_t* pRes,const uint16_t kuiMF) {
	int16_t iTemp[4];
	int32_t i;
	for(i=0;i<16;i+=4) {
		iTemp[0]=pRes[i] +pRes[i+2];
		iTemp[1]=pRes[i]  -pRes[i+2];
		iTemp[2]=pRes[i+1]-pRes[i+3];
		iTemp[3]=pRes[i+1]+pRes[i+3];
		pRes[i]=iTemp[0]+iTemp[3];
		pRes[i+1]=iTemp[1]+iTemp[2];
		pRes[i+2]=iTemp[1]-iTemp[2];
		pRes[i+3]=iTemp[0]-iTemp[3];
	}
	for(i=0;i<4;i++) {
		iTemp[0]=pRes[i] +pRes[i+8];
		iTemp[1]=pRes[i]  -pRes[i+8];
		iTemp[2]=pRes[i+4]-pRes[i+12];
		iTemp[3]=pRes[i+4]+pRes[i+12];
		pRes[i]=(iTemp[0]+iTemp[3])*kuiMF;
		pRes[i+4]=(iTemp[1]+iTemp[2])*kuiMF;
		pRes[i+8]=(iTemp[1]-iTemp[2])*kuiMF;
		pRes[i+12]=(iTemp[0]-iTemp[3])*kuiMF;
	}
}

void WelsDequantIHadamard2x2Dc(int16_t* pDct,const uint16_t kuiMF) {
  const int16_t kiSumU=pDct[0]+pDct[2];
  const int16_t kiDelU=pDct[0]-pDct[2];
  const int16_t kiSumD=pDct[1]+pDct[3];
  const int16_t kiDelD=pDct[1]-pDct[3];
  pDct[0]=((kiSumU+kiSumD)*kuiMF)>>1;
  pDct[1]=((kiSumU-kiSumD)*kuiMF)>>1;
  pDct[2]=((kiDelU+kiDelD)*kuiMF)>>1;
  pDct[3]=((kiDelU-kiDelD)*kuiMF)>>1;
}

void WelsDequantFour4x4_c(int16_t* pRes,const uint16_t* kpMF) {
	int32_t i;
	for(i=0;i<8;i++) {
		pRes[i]*=kpMF[i];
		pRes[i+8]*=kpMF[i];
		pRes[i+16]*=kpMF[i];
		pRes[i+24]*=kpMF[i];
		pRes[i+32]*=kpMF[i];
		pRes[i+40]*=kpMF[i];
		pRes[i+48]*=kpMF[i];
		pRes[i+56]*=kpMF[i];
	}
}

// IDCT functions,final output=prediction(CS)+IDCT(scaled_coeff)
void WelsIDctT4Rec_c(uint8_t* pRec,int32_t iStride,uint8_t* pPred,int32_t iPredStride,int16_t* pDct) {
	int32_t i;
	int16_t iTemp[16];
	int32_t iDstStridex2=iStride<<1;
	int32_t iDstStridex3=iStride+iDstStridex2;
	int32_t iPredStridex2=iPredStride<<1;
	int32_t iPredStridex3=iPredStride+iPredStridex2;
	for(i=0;i<4;i++) { //horizon
		int32_t iIdx=i<<2;
		const int32_t kiHorSumU=pDct[iIdx]+pDct[iIdx+2];	// add 0-2
		const int32_t kiHorDelU=pDct[iIdx]-pDct[iIdx+2];	// sub 0-2
		const int32_t kiHorSumD=pDct[iIdx+1]+(pDct[iIdx+3]>>1);
		const int32_t kiHorDelD=(pDct[iIdx+1]>>1)-pDct[iIdx+3];
		iTemp[iIdx]=kiHorSumU+kiHorSumD;
		iTemp[iIdx+1]=kiHorDelU+kiHorDelD;
		iTemp[iIdx+2]=kiHorDelU-kiHorDelD;
		iTemp[iIdx+3]=kiHorSumU-kiHorSumD;
	}
	for(i=0;i<4;i++) { //vertical
		const int32_t kiVerSumL=iTemp[i]+iTemp[8+i];
		const int32_t kiVerDelL=iTemp[i]-iTemp[8+i];
		const int32_t kiVerDelR=(iTemp[4+i]>>1)-iTemp[12+i];
		const int32_t kiVerSumR=iTemp[4+i]+(iTemp[12+i]>>1);
		pRec[i]=WelsClip1 (pPred[i]+((kiVerSumL+kiVerSumR+32)>>6));
		pRec[iStride+i]=WelsClip1 (pPred[iPredStride+i]+((kiVerDelL+kiVerDelR+32)>>6));
		pRec[iDstStridex2+i]=WelsClip1(pPred[iPredStridex2+i]+((kiVerDelL-kiVerDelR+32)>>6));
		pRec[iDstStridex3+i]=WelsClip1(pPred[iPredStridex3+i]+((kiVerSumL-kiVerSumR+32)>>6));
	}
}

void WelsIDctFourT4Rec_c(uint8_t* pRec,int32_t iStride,uint8_t* pPred,int32_t iPredStride,int16_t* pDct) {
	int32_t iDstStridex4=iStride<<2;
	int32_t iPredStridex4=iPredStride<<2;
	WelsIDctT4Rec_c(pRec,iStride,pPred,iPredStride,pDct);
	WelsIDctT4Rec_c(&pRec[4],iStride,&pPred[4],iPredStride,pDct+16);
	WelsIDctT4Rec_c(&pRec[iDstStridex4],iStride,&pPred[iPredStridex4],iPredStride,pDct+32);
	WelsIDctT4Rec_c(&pRec[iDstStridex4+4],iStride,&pPred[iPredStridex4+4],iPredStride,pDct+48);
}

// pfIDctI16x16Dc: do luma idct of an MB for I16x16 mode,when only dc value are non-zero
void WelsIDctRecI16x16Dc_c(uint8_t* pRec,int32_t iStride,uint8_t* pPred,int32_t iPredStride,int16_t* pDctDc) {
	int32_t i,j;
	for(i=0;i<16;i++) {
		for(j=0;j<16;j++) {
			pRec[j]=WelsClip1 (pPred[j]+((pDctDc[ (i&0x0C)+(j>>2)]+32)>>6));
		}
		pRec+=iStride;
		pPred+=iPredStride;
	}
}

void WelsIDctT4RecOnMb(uint8_t* pDst,int32_t iDstStride,uint8_t* pPred,int32_t iPredStride,int16_t* pDct) {
	int32_t iDstStridex8=iDstStride<<3;
	int32_t iPredStridex8=iPredStride<<3;
	WelsIDctFourT4Rec_c(&pDst[0],iDstStride,&pPred[0],iPredStride,pDct);
	WelsIDctFourT4Rec_c(&pDst[8],iDstStride,&pPred[8],iPredStride,pDct+64);
	WelsIDctFourT4Rec_c(&pDst[iDstStridex8],iDstStride,&pPred[iPredStridex8],iPredStride,pDct+128);
	WelsIDctFourT4Rec_c(&pDst[iDstStridex8+8],iDstStride,&pPred[iPredStridex8+8],iPredStride,pDct+192);
}

//  C code only
void DeblockLumaLt4_c(uint8_t* pPix,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc) {
	for(int32_t i=0;i<16;i++) {
		int32_t iTc0=pTc[i>>2];
		if(iTc0 >=0) {
			int32_t p0=pPix[-iStrideX];
			int32_t p1=pPix[-2*iStrideX];
			int32_t p2=pPix[-3*iStrideX];
			int32_t q0=pPix[0];
			int32_t q1=pPix[iStrideX];
			int32_t q2=pPix[2*iStrideX];
			bool bDetaP0Q0=ABS(p0-q0)<iAlpha;
			bool bDetaP1P0=ABS(p1-p0)<iBeta;
			bool bDetaQ1Q0=ABS(q1-q0)<iBeta;
			int32_t iTc=iTc0;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0) {
				bool bDetaP2P0=ABS(p2-p0)<iBeta;
				bool bDetaQ2Q0=ABS(q2-q0)<iBeta;
				if(bDetaP2P0) {
					pPix[-2*iStrideX]=p1+CLIP3((p2+((p0+q0+1)>>1)-(p1*(1<<1)))>>1,-iTc0,iTc0);
					iTc++;
				}
				if(bDetaQ2Q0) {
					pPix[iStrideX]=q1+CLIP3((q2+((p0+q0+1)>>1)-(q1*(1<<1)))>>1,-iTc0,iTc0);
					iTc++;
				}
				int32_t iDeta=CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc,iTc);
				pPix[-iStrideX]=WelsClip1 (p0+iDeta);    /* p0' */
				pPix[0]=WelsClip1 (q0-iDeta);    /* q0' */
			}
		}
		pPix+=iStrideY;
	}
}
void DeblockLumaEq4_c(uint8_t* pPix,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta) {
	int32_t p0,p1,p2,q0,q1,q2;
	int32_t iDetaP0Q0;
	bool bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0;i<16;i++) {
		p0=pPix[-iStrideX];
		p1=pPix[-2*iStrideX];
		p2=pPix[-3*iStrideX];
		q0=pPix[0];
		q1=pPix[iStrideX];
		q2=pPix[2*iStrideX];
		iDetaP0Q0=ABS(p0-q0);
		bDetaP1P0=ABS(p1-p0)<iBeta;
		bDetaQ1Q0=ABS(q1-q0)<iBeta;
		if((iDetaP0Q0<iAlpha) && bDetaP1P0 && bDetaQ1Q0) {
			if(iDetaP0Q0<((iAlpha>>2)+2)) {
			bool bDetaP2P0=ABS(p2-p0)<iBeta;
			bool bDetaQ2Q0=ABS(q2-q0)<iBeta;
			if(bDetaP2P0) {
				const int32_t p3=pPix[-4*iStrideX];
				pPix[-iStrideX]=(p2+(p1*(1<<1))+(p0*(1<<1))+(q0*(1<<1))+q1+4)>>3;		//p0
				pPix[-2*iStrideX]=(p2+p1+p0+q0+2)>>2;									//p1
				pPix[-3*iStrideX]=((p3*(1<<1))+p2+(p2*(1<<1))+p1+p0+q0+4)>>3;			//p2
			}else{
				pPix[-1*iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;								//p0
			}
			if(bDetaQ2Q0) {
				const int32_t q3=pPix[3*iStrideX];
				pPix[0]=(p1+(p0*(1<<1))+(q0*(1<<1))+(q1*(1<<1))+q2+4)>>3;				//q0
				pPix[iStrideX]=(p0+q0+q1+q2+2)>>2;										//q1
				pPix[2*iStrideX]=((q3*(1<<1))+q2+(q2*(1<<1))+q1+q0+p0+4)>>3;			//q2
			}else{
				pPix[0]=((q1*(1<<1))+q0+p1+2)>>2;										//q0
			}
			}else{
			pPix[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;									//p0
			pPix[ 0]=((q1*(1<<1))+q0+p1+2)>>2;											//q0
			}
		}
		pPix+=iStrideY;
	}
}
void DeblockLumaLt4V_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc) {
	DeblockLumaLt4_c(pPix,iStride,1,iAlpha,iBeta,tc);
}
void DeblockLumaLt4H_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc) {
	DeblockLumaLt4_c(pPix,1,iStride,iAlpha,iBeta,tc);
}
void DeblockLumaEq4V_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta) {
	DeblockLumaEq4_c(pPix,iStride,1,iAlpha,iBeta);
}
void DeblockLumaEq4H_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta) {
	DeblockLumaEq4_c(pPix,1,iStride,iAlpha,iBeta);
}
void DeblockChromaLt4_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc) {
	int32_t p0,p1,q0,q1,iDeta;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0;i<8;i++) {
		int32_t iTc0=pTc[i>>1];
		if(iTc0>0) {
			p0=pPixCb[-iStrideX];
			p1=pPixCb[-2*iStrideX];
			q0=pPixCb[0];
			q1=pPixCb[iStrideX];
			bDetaP0Q0=ABS(p0-q0)<iAlpha;
			bDetaP1P0=ABS(p1-p0)<iBeta;
			bDetaQ1Q0=ABS(q1-q0)<iBeta;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0) {
				iDeta=CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCb[-iStrideX]=WelsClip1 (p0+iDeta);				// p0
				pPixCb[0]=WelsClip1 (q0-iDeta);						// q0
			}
			p0=pPixCr[-iStrideX];
			p1=pPixCr[-2*iStrideX];
			q0=pPixCr[0];
			q1=pPixCr[iStrideX];
			bDetaP0Q0=ABS(p0-q0)<iAlpha;
			bDetaP1P0=ABS(p1-p0)<iBeta;
			bDetaQ1Q0=ABS(q1-q0)<iBeta;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0) {
				iDeta=CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCr[-iStrideX]=WelsClip1 (p0+iDeta);				// p0
				pPixCr[0]=WelsClip1 (q0-iDeta);						// q0
			}
		}
		pPixCb+=iStrideY;
		pPixCr+=iStrideY;
	}
}
void DeblockChromaEq4_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta) {
	int32_t p0,p1,q0,q1;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0;i<8;i++) {
		//cb
		p0=pPixCb[-iStrideX];
		p1=pPixCb[-2*iStrideX];
		q0=pPixCb[0];
		q1=pPixCb[iStrideX];
		bDetaP0Q0=ABS(p0-q0)<iAlpha;
		bDetaP1P0=ABS(p1-p0)<iBeta;
		bDetaQ1Q0=ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0) {
			pPixCb[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;		// p0
			pPixCb[0]=((q1*(1<<1))+q0+p1+2)>>2;				// q0
		}
		//cr
		p0=pPixCr[-iStrideX];
		p1=pPixCr[-2*iStrideX];
		q0=pPixCr[0];
		q1=pPixCr[iStrideX];
		bDetaP0Q0=ABS(p0-q0)<iAlpha;
		bDetaP1P0=ABS(p1-p0)<iBeta;
		bDetaQ1Q0=ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0) {
			pPixCr[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;		// p0
			pPixCr[0]=((q1*(1<<1))+q0+p1+2)>>2;				// q0
		}
		pPixCr+=iStrideY;
		pPixCb+=iStrideY;
	}
}
void DeblockChromaLt4V_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc) {
	DeblockChromaLt4_c(pPixCb,pPixCr,iStride,1,iAlpha,iBeta,tc);
}
void DeblockChromaLt4H_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc) {
	DeblockChromaLt4_c(pPixCb,pPixCr,1,iStride,iAlpha,iBeta,tc);
}
void DeblockChromaEq4V_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta) {
	DeblockChromaEq4_c(pPixCb,pPixCr,iStride,1,iAlpha,iBeta);
}
void DeblockChromaEq4H_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta) {
	DeblockChromaEq4_c(pPixCb,pPixCr,1,iStride,iAlpha,iBeta);
}

void DeblockChromaLt42_c(uint8_t* pPixCbCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc) {
	int32_t p0,p1,q0,q1,iDeta;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0;i<8;i++) {
		int32_t iTc0=pTc[i>>1];
		if(iTc0>0) {
			p0=pPixCbCr[-iStrideX];
			p1=pPixCbCr[-2*iStrideX];
			q0=pPixCbCr[0];
			q1=pPixCbCr[iStrideX];
			bDetaP0Q0=ABS(p0-q0)<iAlpha;
			bDetaP1P0=ABS(p1-p0)<iBeta;
			bDetaQ1Q0=ABS(q1-q0)<iBeta;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0) {
			iDeta=CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCbCr[-iStrideX]=WelsClip1 (p0+iDeta);		// p0
				pPixCbCr[0]=WelsClip1 (q0-iDeta);				// q0
			}
		}
		pPixCbCr+=iStrideY;
	}
}
void DeblockChromaEq42_c(uint8_t* pPixCbCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta) {
	int32_t p0,p1,q0,q1;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0;i<8;i++) {
		p0=pPixCbCr[-iStrideX];
		p1=pPixCbCr[-2*iStrideX];
		q0=pPixCbCr[0];
		q1=pPixCbCr[iStrideX];
		bDetaP0Q0=ABS(p0-q0)<iAlpha;
		bDetaP1P0=ABS(p1-p0)<iBeta;
		bDetaQ1Q0=ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0) {
			pPixCbCr[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;		// p0
			pPixCbCr[0]=((q1*(1<<1))+q0+p1+2)>>2;				// q0
		}
		pPixCbCr+=iStrideY;
	}
}

#define MB_BS_MV(sCurMv,sNeighMv,uiBIdx,uiBnIdx) (( ABS( sCurMv[uiBIdx].iMvX-sNeighMv[uiBnIdx].iMvX ) >=4 ) || ( ABS( sCurMv[uiBIdx].iMvY-sNeighMv[uiBnIdx].iMvY ) >=4 ))

static const uint8_t g_kuiTableBIdx[2][8]={{0,4,8,12,3,7,11,15},{0,1,2,3,12,13,14,15},};

uint32_t NewH264SVCEncoder::DeblockingBSMarginalMBAvcbase(const MBWindow& pCurMb,const MacroBlock* pNeighMb,int32_t iEdge) {
	int32_t i;
	uint32_t uiBSx4;
	uint8_t* pBS=(uint8_t*) (&uiBSx4);
	const uint8_t* pBIdx=&g_kuiTableBIdx[iEdge][0];
	const uint8_t* pBnIdx=&g_kuiTableBIdx[iEdge][4];
	for(i=0;i<4;i++) {
		if(pCurMb.m_mb->m_nonZeroCount[*pBIdx] | pNeighMb->m_nonZeroCount[*pBnIdx]) {
			pBS[i]=2;
		}else{
			pBS[i]=MB_BS_MV(pCurMb.m_mb->m_mv,pNeighMb->m_mv,*pBIdx,*pBnIdx);
		}
		pBIdx++;
		pBnIdx++;
	}
	return uiBSx4;
}

#define SMB_EDGE_MV(uiRefIndex,sMotionVector,uiBIdx,uiBnIdx) (!!((ABS(sMotionVector[uiBIdx].iMvX-sMotionVector[uiBnIdx].iMvX) &(~3)) | (ABS(sMotionVector[uiBIdx].iMvY-sMotionVector[uiBnIdx].iMvY) &(~3))))
#define BS_EDGE(bsx1,uiRefIndex,sMotionVector,uiBIdx,uiBnIdx) ( (bsx1|SMB_EDGE_MV(uiRefIndex,sMotionVector,uiBIdx,uiBnIdx))<<(bsx1?1:0))

void inline DeblockingBSInsideMBNormal(const MBWindow& pCurMb,uint8_t uiBS[2][4][4],int8_t* pNnzTab) {
	uint32_t uiNnz32b0,uiNnz32b1,uiNnz32b2,uiNnz32b3;
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBsx4,4,4);
	uiNnz32b0=*(uint32_t*)(pNnzTab+0);
	uiNnz32b1=*(uint32_t*)(pNnzTab+4);
	uiNnz32b2=*(uint32_t*)(pNnzTab+8);
	uiNnz32b3=*(uint32_t*)(pNnzTab+12);

	for(int i=0;i<3;i++)
		uiBsx4[i]=pNnzTab[i] | pNnzTab[i+1];
	uiBS[0][1][0]=BS_EDGE(uiBsx4[0],iRefIdx,pCurMb.m_mb->m_mv,1,0);
	uiBS[0][2][0]=BS_EDGE(uiBsx4[1],iRefIdx,pCurMb.m_mb->m_mv,2,1);
	uiBS[0][3][0]=BS_EDGE(uiBsx4[2],iRefIdx,pCurMb.m_mb->m_mv,3,2);

	for(int i=0;i<3;i++)
		uiBsx4[i]=pNnzTab[4+i] | pNnzTab[4+i+1];
	uiBS[0][1][1]=BS_EDGE(uiBsx4[0],iRefIdx,pCurMb.m_mb->m_mv,5,4);
	uiBS[0][2][1]=BS_EDGE(uiBsx4[1],iRefIdx,pCurMb.m_mb->m_mv,6,5);
	uiBS[0][3][1]=BS_EDGE(uiBsx4[2],iRefIdx,pCurMb.m_mb->m_mv,7,6);

	for(int i=0;i<3;i++)
		uiBsx4[i]=pNnzTab[8+i] | pNnzTab[8+i+1];
	uiBS[0][1][2]=BS_EDGE(uiBsx4[0],iRefIdx,pCurMb.m_mb->m_mv,9,8);
	uiBS[0][2][2]=BS_EDGE(uiBsx4[1],iRefIdx,pCurMb.m_mb->m_mv,10,9);
	uiBS[0][3][2]=BS_EDGE(uiBsx4[2],iRefIdx,pCurMb.m_mb->m_mv,11,10);

	for(int i=0;i<3;i++)
		uiBsx4[i]=pNnzTab[12+i] | pNnzTab[12+i+1];
	uiBS[0][1][3]=BS_EDGE(uiBsx4[0],iRefIdx,pCurMb.m_mb->m_mv,13,12);
	uiBS[0][2][3]=BS_EDGE(uiBsx4[1],iRefIdx,pCurMb.m_mb->m_mv,14,13);
	uiBS[0][3][3]=BS_EDGE(uiBsx4[2],iRefIdx,pCurMb.m_mb->m_mv,15,14);

	//horizontal
	*(uint32_t*)uiBsx4=(uiNnz32b0 | uiNnz32b1);
	uiBS[1][1][0]=BS_EDGE(uiBsx4[0],iRefIdx,pCurMb.m_mb->m_mv,4,0);
	uiBS[1][1][1]=BS_EDGE(uiBsx4[1],iRefIdx,pCurMb.m_mb->m_mv,5,1);
	uiBS[1][1][2]=BS_EDGE(uiBsx4[2],iRefIdx,pCurMb.m_mb->m_mv,6,2);
	uiBS[1][1][3]=BS_EDGE(uiBsx4[3],iRefIdx,pCurMb.m_mb->m_mv,7,3);

	*(uint32_t*)uiBsx4=(uiNnz32b1 | uiNnz32b2);
	uiBS[1][2][0]=BS_EDGE(uiBsx4[0],iRefIdx,pCurMb.m_mb->m_mv,8,4);
	uiBS[1][2][1]=BS_EDGE(uiBsx4[1],iRefIdx,pCurMb.m_mb->m_mv,9,5);
	uiBS[1][2][2]=BS_EDGE(uiBsx4[2],iRefIdx,pCurMb.m_mb->m_mv,10,6);
	uiBS[1][2][3]=BS_EDGE(uiBsx4[3],iRefIdx,pCurMb.m_mb->m_mv,11,7);

	*(uint32_t*)uiBsx4=(uiNnz32b2 | uiNnz32b3);
	uiBS[1][3][0]=BS_EDGE(uiBsx4[0],iRefIdx,pCurMb.m_mb->m_mv,12,8);
	uiBS[1][3][1]=BS_EDGE(uiBsx4[1],iRefIdx,pCurMb.m_mb->m_mv,13,9);
	uiBS[1][3][2]=BS_EDGE(uiBsx4[2],iRefIdx,pCurMb.m_mb->m_mv,14,10);
	uiBS[1][3][3]=BS_EDGE(uiBsx4[3],iRefIdx,pCurMb.m_mb->m_mv,15,11);
}

void inline DeblockingBSInsideMBAvsbase(int8_t* pNnzTab,uint8_t nBS[2][4][4],int32_t iLShiftFactor) {
	uint32_t uiNnz32b0,uiNnz32b1,uiNnz32b2,uiNnz32b3;

	uiNnz32b0=*(uint32_t*)(pNnzTab+0);
	uiNnz32b1=*(uint32_t*)(pNnzTab+4);
	uiNnz32b2=*(uint32_t*)(pNnzTab+8);
	uiNnz32b3=*(uint32_t*)(pNnzTab+12);

	nBS[0][1][0]=(pNnzTab[0] | pNnzTab[1])<<iLShiftFactor;
	nBS[0][2][0]=(pNnzTab[1] | pNnzTab[2])<<iLShiftFactor;
	nBS[0][3][0]=(pNnzTab[2] | pNnzTab[3])<<iLShiftFactor;

	nBS[0][1][1]=(pNnzTab[4] | pNnzTab[5])<<iLShiftFactor;
	nBS[0][2][1]=(pNnzTab[5] | pNnzTab[6])<<iLShiftFactor;
	nBS[0][3][1]=(pNnzTab[6] | pNnzTab[7])<<iLShiftFactor;
	*(uint32_t*)nBS[1][1]=(uiNnz32b0 | uiNnz32b1)<<iLShiftFactor;

	nBS[0][1][2]=(pNnzTab[8]  | pNnzTab[9]) <<iLShiftFactor;
	nBS[0][2][2]=(pNnzTab[9]  | pNnzTab[10])<<iLShiftFactor;
	nBS[0][3][2]=(pNnzTab[10] | pNnzTab[11])<<iLShiftFactor;
	*(uint32_t*)nBS[1][2]=(uiNnz32b1 | uiNnz32b2)<<iLShiftFactor;

	nBS[0][1][3]=(pNnzTab[12] | pNnzTab[13])<<iLShiftFactor;
	nBS[0][2][3]=(pNnzTab[13] | pNnzTab[14])<<iLShiftFactor;
	nBS[0][3][3]=(pNnzTab[14] | pNnzTab[15])<<iLShiftFactor;
	*(uint32_t*)nBS[1][3]=(uiNnz32b2 | uiNnz32b3)<<iLShiftFactor;
}

void WelsNonZeroCount_c(int8_t* pNonZeroCount) {
	int32_t i;
	for(i=0;i<24;i++) {
		pNonZeroCount[i]=!!pNonZeroCount[i];
	}
}

void NewH264SVCEncoder::DeblockingBSCalc_c(SDeblockingFilter* pFilter,Mb_Type uiCurMbType,const MBWindow& mbWindow) {
	uint8_t uiBS[2][4][4]={{{0}}};
	if(mbWindow.iMbX>0) {
		*(uint32_t*)uiBS[0][0]=IS_INTRA(mbWindow.GetNeighbor(N_LEFT)->uiMbType) ? 0x04040404 : DeblockingBSMarginalMBAvcbase(mbWindow,mbWindow.GetNeighbor(N_LEFT),0);
	}else{
		*(uint32_t*)uiBS[0][0]=0;
	}
	if(mbWindow.iMbY>0) {
		*(uint32_t*)uiBS[1][0]=IS_INTRA(mbWindow.GetNeighbor(N_TOP)->uiMbType) ? 0x04040404 : DeblockingBSMarginalMBAvcbase(mbWindow,mbWindow.GetNeighbor(N_TOP),1);
	}else{
		*(uint32_t*)uiBS[1][0]=0;
	}
	//SKIP MB_16x16 or others
	if(uiCurMbType!=MB_TYPE_SKIP) {
		WelsNonZeroCount_c(mbWindow.m_mb->m_nonZeroCount);// set all none-zero nzc to 1;dbk can be opti!
		if(uiCurMbType==MB_TYPE_16x16) {
			DeblockingBSInsideMBAvsbase(mbWindow.m_mb->m_nonZeroCount,uiBS,1);
		}else{
			DeblockingBSInsideMBNormal(mbWindow,uiBS,mbWindow.m_mb->m_nonZeroCount);
		}
	}else{
		*(uint32_t*)uiBS[0][1]=*(uint32_t*)uiBS[0][2]=* (uint32_t*)uiBS[0][3]=* (uint32_t*)uiBS[1][1]=* (uint32_t*)uiBS[1][2]=* (uint32_t*)uiBS[1][3]=0;
	}
	DeblockingInterMb(pFilter,mbWindow,uiBS);
}

void NewH264SVCEncoder::FillNeighborCacheInterWithBGD(SMbCache* pMbCache,const MBWindow& mbWindow,int32_t iMbWidth,const int8_t* pVaaBgMbFlag) {
	const MacroBlock* pLeftMb=mbWindow.GetNeighbor(N_LEFT);
	const MacroBlock* pTopMb=mbWindow.GetNeighbor(N_TOP);
	const MacroBlock* pLeftTopMb=mbWindow.GetNeighbor(N_TOPLEFT);
	const MacroBlock* iRightTopMb=mbWindow.GetNeighbor(N_TOPRIGHT);
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	if(mbWindow.HasNeightbor(N_LEFT) && IS_INTER(pLeftMb->uiMbType)) {
		pMvComp->sMotionVectorCache[6]=pLeftMb->m_mv[3];
		pMvComp->sMotionVectorCache[12]=pLeftMb->m_mv[7];
		pMvComp->sMotionVectorCache[18]=pLeftMb->m_mv[11];
		pMvComp->sMotionVectorCache[24]=pLeftMb->m_mv[15];
		pMvComp->iRefIndexCache[6]=pLeftMb->m_refIndex[1];
		pMvComp->iRefIndexCache[12]=pLeftMb->m_refIndex[1];
		pMvComp->iRefIndexCache[18]=pLeftMb->m_refIndex[3];
		pMvComp->iRefIndexCache[24]=pLeftMb->m_refIndex[3];
		pMbCache->iSadCost[3]=pLeftMb->m_sadCost;
		if(pLeftMb->uiMbType==MB_TYPE_SKIP && pVaaBgMbFlag[-1]==0) {
			pMbCache->m_mbTypeSkip[3]=1;
			pMbCache->iSadCostSkip[3]=pMbCache->pEncSad[-1];
		}else{
			pMbCache->m_mbTypeSkip[3]=0;
			pMbCache->iSadCostSkip[3]=0;
		}
	}else{ //avail or non-inter
		ST32(&pMvComp->sMotionVectorCache[6],0);
		ST32(&pMvComp->sMotionVectorCache[12],0);
		ST32(&pMvComp->sMotionVectorCache[18],0);
		ST32(&pMvComp->sMotionVectorCache[24],0);
		pMvComp->iRefIndexCache[6]=pMvComp->iRefIndexCache[12]=pMvComp->iRefIndexCache[18]=pMvComp->iRefIndexCache[24]=mbWindow.HasNeightbor(N_LEFT) ? REF_NOT_IN_LIST : REF_NOT_AVAIL;
		pMbCache->iSadCost[3]=0;
		pMbCache->m_mbTypeSkip[3]=0;
		pMbCache->iSadCostSkip[3]=0;
	}
	if((mbWindow.HasNeightbor(N_TOP)) && IS_INTER(pTopMb->uiMbType)) { //TOP MB
		ST64(&pMvComp->sMotionVectorCache[1],LD64 (&pTopMb->m_mv[12]));
		ST64(&pMvComp->sMotionVectorCache[3],LD64 (&pTopMb->m_mv[14]));
		pMvComp->iRefIndexCache[1]=pTopMb->m_refIndex[2];
		pMvComp->iRefIndexCache[2]=pTopMb->m_refIndex[2];
		pMvComp->iRefIndexCache[3]=pTopMb->m_refIndex[3];
		pMvComp->iRefIndexCache[4]=pTopMb->m_refIndex[3];
		pMbCache->iSadCost[1]=pTopMb->m_sadCost;
		if(pTopMb->uiMbType==MB_TYPE_SKIP  && pVaaBgMbFlag[-iMbWidth]==0) {
			pMbCache->m_mbTypeSkip[1]=1;
			pMbCache->iSadCostSkip[1]=pMbCache->pEncSad[-iMbWidth];
		}else{
			pMbCache->m_mbTypeSkip[1]=0;
			pMbCache->iSadCostSkip[1]=0;
		}
	}else{ //unavail
		ST64(&pMvComp->sMotionVectorCache[1],0);
		ST64(&pMvComp->sMotionVectorCache[3],0);
		pMvComp->iRefIndexCache[1]=pMvComp->iRefIndexCache[2]=pMvComp->iRefIndexCache[3]=pMvComp->iRefIndexCache[4]=mbWindow.HasNeightbor(N_TOP) ? REF_NOT_IN_LIST : REF_NOT_AVAIL;
		pMbCache->iSadCost[1]=0;
		pMbCache->m_mbTypeSkip[1]=0;
		pMbCache->iSadCostSkip[1]=0;
	}
	if((mbWindow.HasNeightbor(N_TOPLEFT)) && IS_INTER(pLeftTopMb->uiMbType)) { //LEFT_TOP MB
		pMvComp->sMotionVectorCache[0]=pLeftTopMb->m_mv[15];
		pMvComp->iRefIndexCache[0]=pLeftTopMb->m_refIndex[3];
		pMbCache->iSadCost[0]=pLeftTopMb->m_sadCost;
		if(pLeftTopMb->uiMbType==MB_TYPE_SKIP  && pVaaBgMbFlag[-iMbWidth-1]==0) {
			pMbCache->m_mbTypeSkip[0]=1;
			pMbCache->iSadCostSkip[0]=pMbCache->pEncSad[-iMbWidth-1];
		}else{
			pMbCache->m_mbTypeSkip[0]=0;
			pMbCache->iSadCostSkip[0]=0;
		}
	}else{ //unavail
		ST32(&pMvComp->sMotionVectorCache[0],0);
		pMvComp->iRefIndexCache[0]=mbWindow.HasNeightbor(N_TOPLEFT) ? REF_NOT_IN_LIST : REF_NOT_AVAIL;
		pMbCache->iSadCost[0]=0;
		pMbCache->m_mbTypeSkip[0]=0;
		pMbCache->iSadCostSkip[0]=0;
	}
	if(mbWindow.HasNeightbor(N_TOPRIGHT) && IS_INTER(iRightTopMb->uiMbType)) { //RIGHT_TOP MB
		pMvComp->sMotionVectorCache[5]=iRightTopMb->m_mv[12];
		pMvComp->iRefIndexCache[5]=iRightTopMb->m_refIndex[2];
		pMbCache->iSadCost[2]=iRightTopMb->m_sadCost;
		if(iRightTopMb->uiMbType==MB_TYPE_SKIP && pVaaBgMbFlag[-iMbWidth+1]==0) {
			pMbCache->m_mbTypeSkip[2]=1;
			pMbCache->iSadCostSkip[2]=pMbCache->pEncSad[-iMbWidth+1];
		}else{
			pMbCache->m_mbTypeSkip[2]=0;
			pMbCache->iSadCostSkip[2]=0;
		}
	}else{ //unavail
		ST32(&pMvComp->sMotionVectorCache[5],0);
		pMvComp->iRefIndexCache[5]=mbWindow.HasNeightbor(N_TOPRIGHT) ? REF_NOT_IN_LIST : REF_NOT_AVAIL;
		pMbCache->iSadCost[2]=0;
		pMbCache->m_mbTypeSkip[2]=0;
		pMbCache->iSadCostSkip[2]=0;
	}
	//right-top 4*4 pBlock unavailable
	ST32(&pMvComp->sMotionVectorCache[9],0);
	ST32(&pMvComp->sMotionVectorCache[21],0);
	ST32(&pMvComp->sMotionVectorCache[11],0);
	ST32(&pMvComp->sMotionVectorCache[17],0);
	ST32(&pMvComp->sMotionVectorCache[23],0);
	pMvComp->iRefIndexCache[ 9]=pMvComp->iRefIndexCache[11]=pMvComp->iRefIndexCache[17]=pMvComp->iRefIndexCache[21]=pMvComp->iRefIndexCache[23]=REF_NOT_AVAIL;
}

#define PADDING_LENGTH 32 // reference extension

// rewrite it (split into luma&chroma) that is helpful for mmx/sse2 optimization perform,9/27/2009
static inline void ExpandPictureLuma_c(uint8_t* pDst,const int32_t kiStride,const int32_t kiPicW,const int32_t kiPicH) {
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
		memcpy (pTop,pTmp,kiPicW);
		memcpy (pBottom,pDstLastLine,kiPicW);
		// pad corners
		memset(pTop-kiPaddingLen,kuiTL,kiPaddingLen);		// pTop left
		memset(pTop+kiPicW,kuiTR,kiPaddingLen);				// pTop right
		memset(pBottom-kiPaddingLen,kuiBL,kiPaddingLen);	// pBottom left
		memset(pBottom+kiPicW,kuiBR,kiPaddingLen);			// pBottom right
		++i;
	}while(i<kiPaddingLen);
	// pad left and right
	i=0;
	do{
		memset(pTmp-kiPaddingLen,pTmp[0],kiPaddingLen);
		memset(pTmp+kiPicW,pTmp[kiPicW-1],kiPaddingLen);
		pTmp+=kiStride;
		++i;
	}while(i<kiPicH);
}

static inline void ExpandPictureChroma_c(uint8_t* pDst,const int32_t kiStride,const int32_t kiPicW,const int32_t kiPicH) {
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
		memcpy (pTop,pTmp,kiPicW);
		memcpy (pBottom,pDstLastLine,kiPicW);

		// pad corners
		memset(pTop-kiPaddingLen,kuiTL,kiPaddingLen);		// pTop left
		memset(pTop+kiPicW,kuiTR,kiPaddingLen);				// pTop right
		memset(pBottom-kiPaddingLen,kuiBL,kiPaddingLen);	// pBottom left
		memset(pBottom+kiPicW,kuiBR,kiPaddingLen);			// pBottom right

		++i;
	}while(i<kiPaddingLen);
	// pad left and right
	i=0;
	do{
		memset(pTmp-kiPaddingLen,pTmp[0],kiPaddingLen);
		memset(pTmp+kiPicW,pTmp[kiPicW-1],kiPaddingLen);
		pTmp+=kiStride;
		++i;
	}while(i<kiPicH);
}

void NewH264SVCEncoder::GetEncBlockStrideOffset(int32_t* pBlock,const int32_t kiStrideY,const int32_t kiStrideUV) {
	int32_t i,j,k,r;
	for(j=0;j<4;j++) {
		i=j<<2;
		k=(j&0x01)<<1;
		r=j&0x02;
		pBlock[i]=(0+k+(0+r)*kiStrideY)<<2;
		pBlock[i+1]=(1+k+(0+r)*kiStrideY)<<2;
		pBlock[i+2]=(0+k+(1+r)*kiStrideY)<<2;
		pBlock[i+3]=(1+k+(1+r)*kiStrideY)<<2;
		pBlock[16+j]=pBlock[20+j]=((j&0x01)+r*kiStrideUV)<<2;
	}
}

static inline void WelsSetMemUint32_c(uint32_t* pDst,uint32_t iValue,int32_t iSizeOfData) {
	for(int i=0;i<iSizeOfData;i++) {
		pDst[i]=iValue;
	}
}

static inline void WelsSetMemUint16_c(uint16_t* pDst,uint16_t iValue,int32_t iSizeOfData) {
	for(int i=0;i<iSizeOfData;i++) {
		pDst[i]=iValue;
	}
}

// alloc picture pData with borders for each plane based width and height of picture

void NewH264SVCEncoder::FreePicture(SPicture* picture) {
	Free(picture->pBuffer);
	if(picture->m_uiRefMbType) {
		Free(picture->m_uiRefMbType);
		Free(picture->pRefMbQp);
		Free(picture->sMvList);
		Free(picture->pMbSkipSad);
	}
}

void NewH264SVCEncoder::ClearPicture(SPicture* picture,bool clearData,bool clearInfo) {
	int32_t iPicHeight=ALIGN_WITH(m_videoHeight,MB_HEIGHT_LUMA)+(PADDING_LENGTH<<1); 	// with height of vertical
	int32_t iPicChromaWidth=m_stride[0]>>1;
	int32_t iPicChromaHeight=iPicHeight>>1;
	int32_t iLumaSize=m_stride[0]*iPicHeight;
	int32_t iChromaSize=iPicChromaWidth*iPicChromaHeight;
	if(clearData) {
		memset(picture->pBuffer,0,iLumaSize+(iChromaSize<<1));
	}
	if(clearInfo) {
		const uint32_t kuiCountMbNum=m_blockWidth*m_blockHeight;
		memset(picture->m_uiRefMbType,0,kuiCountMbNum*sizeof(uint32_t));
		memset(picture->pRefMbQp,0,kuiCountMbNum*sizeof(uint8_t));
		memset(picture->sMvList,0,kuiCountMbNum*sizeof(SMVUnitXY));
		memset(picture->pMbSkipSad,0,kuiCountMbNum*sizeof(int32_t));
	}
}

SPicture* NewH264SVCEncoder::AllocPicture(bool bNeedMbInfo) {
	SPicture* picture=new SPicture();
	int32_t iPicHeight=ALIGN_WITH(m_videoHeight,MB_HEIGHT_LUMA)+(PADDING_LENGTH<<1); 	// with height of vertical
	int32_t iPicChromaWidth=m_stride[0]>>1;
	int32_t iPicChromaHeight=iPicHeight>>1;
	int32_t iLumaSize=m_stride[0]*iPicHeight;
	int32_t iChromaSize=iPicChromaWidth*iPicChromaHeight;
	picture->pBuffer=(uint8_t*)Malloc(iLumaSize+(iChromaSize<<1));
	picture->pData[0]=picture->pBuffer+(1+m_stride[0])*PADDING_LENGTH;
	picture->pData[1]=picture->pBuffer+iLumaSize+(((1+m_stride[1])*PADDING_LENGTH)>>1);
	picture->pData[2]=picture->pBuffer+iLumaSize+iChromaSize+(((1+m_stride[2])*PADDING_LENGTH)>>1);
	if(bNeedMbInfo) {
		const uint32_t kuiCountMbNum=m_blockWidth*m_blockHeight;
		picture->m_uiRefMbType=(uint32_t*)Mallocz(kuiCountMbNum*sizeof(uint32_t));
		picture->pRefMbQp=(uint8_t*)Mallocz(kuiCountMbNum*sizeof(uint8_t));
		picture->sMvList=static_cast<SMVUnitXY*>(Mallocz(kuiCountMbNum*sizeof(SMVUnitXY)));
		picture->pMbSkipSad=(int32_t*)Mallocz(kuiCountMbNum*sizeof(int32_t));
	}
	return picture;
}

int32_t NewH264SVCEncoder::AllocStrideTables() {
	int16_t* pTmpRow=NULL,*pRowX=NULL,*pRowY=NULL,*p=NULL;
	uint8_t* pBaseDec=NULL,*pBaseEnc=NULL,*pBaseMbX=NULL,*pBaseMbY=NULL;
	struct {
		int32_t iMbWidth;
		int32_t iCountMbNum;	 			// count number of SMB in each spatial
		int32_t iSizeAllMbAlignCache;		// cache line size aligned in each spatial
	} sMbSizeMap[1]={{0}};
	int32_t iLineSizeY[1][2]={{0}};
	int32_t iLineSizeUV[1][2]={{0}};
	int32_t iMapSpatialIdx[1][2]={{0}};
	int32_t iSizeDec=0;
	int32_t iSizeEnc=0;
	int32_t iCountLayersNeedCs[2]={0};
	const int32_t kiUnit1Size=24*sizeof(int32_t);
	int32_t iUnit2Size=0;
	int32_t iNeedAllocSize=0;
	int32_t iRowSize=0;
	int16_t iMaxMbWidth=0;
	int16_t iMaxMbHeight=0;
	int32_t i=0;
	const int32_t kiTmpWidth=(m_videoWidth+15)>>4;
	const int32_t kiTmpHeight=(m_videoHeight+15)>>4;
	int32_t iNumMb=kiTmpWidth*kiTmpHeight;
	sMbSizeMap[0].iMbWidth=kiTmpWidth;
	sMbSizeMap[0].iCountMbNum=iNumMb;
	iNumMb*=sizeof(int16_t);
	sMbSizeMap[0].iSizeAllMbAlignCache=iNumMb;
	iUnit2Size+=iNumMb;
	const bool kbBaseTemporalFlag=true;
	const int32_t kiWidthPad=ALIGN_WITH(m_videoWidth,16)+(PADDING_LENGTH<<1);
	iLineSizeY[0][kbBaseTemporalFlag]=ALIGN_WITH(kiWidthPad,32);
	iLineSizeUV[0][kbBaseTemporalFlag]=ALIGN_WITH(kiWidthPad>>1,16);
	iMapSpatialIdx[iCountLayersNeedCs[kbBaseTemporalFlag]][kbBaseTemporalFlag]=0;
	++iCountLayersNeedCs[kbBaseTemporalFlag];
	iSizeDec=kiUnit1Size*(iCountLayersNeedCs[0]+iCountLayersNeedCs[1]);
	iSizeEnc=kiUnit1Size*1;
	iNeedAllocSize=iSizeDec+iSizeEnc+(iUnit2Size<<1);
	uint8_t* pBase=(uint8_t*)Mallocz(iNeedAllocSize);
	m_strideTable.m_allocBase=pBase;

	pBaseDec=pBase;							// iCountLayersNeedCs
	pBaseEnc=pBaseDec+iSizeDec;				// iNumSpatialLayers
	pBaseMbX=pBaseEnc+iSizeEnc;				// iNumSpatialLayers
	pBaseMbY=pBaseMbX+iUnit2Size;			// iNumSpatialLayers
	const int32_t kiActualSpatialIdx=iMapSpatialIdx[0][kbBaseTemporalFlag];
	if(kiActualSpatialIdx)
		FATAL("TF");
	const int32_t kiLumaWidth=iLineSizeY[kiActualSpatialIdx][kbBaseTemporalFlag];
	const int32_t kiChromaWidth=iLineSizeUV[kiActualSpatialIdx][kbBaseTemporalFlag];
	GetEncBlockStrideOffset((int32_t*)pBaseDec,kiLumaWidth,kiChromaWidth);
	m_strideTable.pStrideDecBlockOffset[kiActualSpatialIdx][kbBaseTemporalFlag]=(int32_t*)pBaseDec;
	pBaseDec+=kiUnit1Size;
	int32_t iMatchIndex=0;
	bool bMatchFlag=false;
	i=0;
	while(i<iCountLayersNeedCs[kbBaseTemporalFlag]) {
		const int32_t kiActualIdx=iMapSpatialIdx[i][kbBaseTemporalFlag];
		if(!bMatchFlag) {
			iMatchIndex=kiActualIdx;
			bMatchFlag=true;
		}
		++i;
	}
	// not in spatial map and assign match one to it
	m_strideTable.pStrideDecBlockOffset[1][kbBaseTemporalFlag]=m_strideTable.pStrideDecBlockOffset[iMatchIndex][kbBaseTemporalFlag];
	const int32_t kiAllocMbSize=sMbSizeMap[0].iSizeAllMbAlignCache;
	m_strideTable.m_pStrideEncBlockOffset=(int32_t*)pBaseEnc;

	m_strideTable.pMbIndexX=(int16_t*)pBaseMbX;
	m_strideTable.pMbIndexY=(int16_t*)pBaseMbY;
	pBaseEnc+=kiUnit1Size;
	pBaseMbX+=kiAllocMbSize;
	pBaseMbY+=kiAllocMbSize;
	// initialize pMbIndexX and pMbIndexY tables as below
	iMaxMbWidth=sMbSizeMap[1-1].iMbWidth;
	iMaxMbWidth=ALIGN_WITH(iMaxMbWidth,4); // 4 loops for int16_t required introduced as below
	iRowSize=iMaxMbWidth*sizeof(int16_t);
	pTmpRow=(int16_t*)alloca(iRowSize);
	memset(pTmpRow,0,iRowSize);
	pRowX=pTmpRow;
	pRowY=pRowX;
	// initialize pRowX&pRowY
	i=0;
	p=pRowX;
	while(i<iMaxMbWidth) {
		p[0]=i;
		p[1]=1+i;
		p[2]=2+i;
		p[3]=3+i;
		p+=4;
		i+=4;
	}
	int16_t* pMbIndexX=m_strideTable.pMbIndexX;
	const int32_t kiMbWidth=sMbSizeMap[0].iMbWidth;
	const int32_t kiMbHeight=sMbSizeMap[0].iCountMbNum/kiMbWidth;
	const int32_t kiLineSize=kiMbWidth*sizeof(int16_t);
	i=0;
	while(i<kiMbHeight) {
		memcpy(pMbIndexX,pRowX,kiLineSize);
		pMbIndexX+=kiMbWidth;
		++i;
	}
	memset(pRowY,0,iRowSize);
	iMaxMbHeight=sMbSizeMap[1-1].iCountMbNum/sMbSizeMap[1-1].iMbWidth;
	i=0;
	for(;;) {
		ENFORCE_STACK_ALIGN_1D(int16_t,t,4,16)
		int32_t t32=0;
		int16_t j=0;
		const int32_t kiMbWidth=sMbSizeMap[0].iMbWidth;
		const int32_t kiMbHeight=sMbSizeMap[0].iCountMbNum/kiMbWidth;
		const int32_t kiLineSize=kiMbWidth*sizeof(int16_t);
		int16_t* pMbIndexY=m_strideTable.pMbIndexY+i*kiMbWidth;
		if(i<kiMbHeight) {
			memcpy (pMbIndexY,pRowY,kiLineSize);
		}
		++i;
		if(i>=iMaxMbHeight)
			break;
		t32=i | (i<<16);
		ST32(t,t32);
		ST32(t+2,t32);
		p=pRowY;
		while(j<iMaxMbWidth) {
			ST64(p,LD64 (t));
			p+=4;
			j+=4;
		}
	}
	return 0;
}

//iMvdSz=(648*2+1) or (972*2+1);
void NewH264SVCEncoder::MvdCostInit(uint16_t* pMvdCostInter,const int32_t kiMvdSz) {
	const int32_t kiSz=kiMvdSz>>1;
	uint16_t* pNegMvd=pMvdCostInter;
	uint16_t* pPosMvd=pMvdCostInter+kiSz+1;
	const int32_t* kpQpLambda=&g_kiQpCostTable[0];
	int32_t i,j;
	for(i=0;i<52;++i) {
		const uint16_t kiLambda=kpQpLambda[i];
		int32_t iNegSe=-kiSz;
		int32_t iPosSe=1;
		for(j=0;j<kiSz;j+=4) {
			*pNegMvd++=kiLambda*BsSizeSE(iNegSe++);
			*pNegMvd++=kiLambda*BsSizeSE(iNegSe++);
			*pNegMvd++=kiLambda*BsSizeSE(iNegSe++);
			*pNegMvd++=kiLambda*BsSizeSE(iNegSe++);
			*pPosMvd++=kiLambda*BsSizeSE(iPosSe++);
			*pPosMvd++=kiLambda*BsSizeSE(iPosSe++);
			*pPosMvd++=kiLambda*BsSizeSE(iPosSe++);
			*pPosMvd++=kiLambda*BsSizeSE(iPosSe++);
		}
		*pNegMvd=kiLambda;
		pNegMvd+=kiSz+1;
		pPosMvd+=kiSz+1;
	}
}

#define SPS_BUFFER_SIZE         32
#define PPS_BUFFER_SIZE         16
#define SSEI_BUFFER_SIZE        128

#define COMPRESS_RATIO_THR (1.0f) 	//set to size of the original data,which will be large enough considering MinCR

struct SLevelLimits {
	ELevelIdc uiLevelIdc;										// level idc
	uint32_t uiMaxMBPS;											// Max macroblock processing rate(MB/s)
	uint32_t uiMaxFS;											// Max frame sizea(MBs)
	uint32_t uiMaxDPBMbs;										// Max decoded picture buffer size(MBs)
	uint32_t uiMaxBR;											// Max video bit rate
	uint32_t uiMaxCPB;											// Max CPB size
	int16_t iMinVmv;											// Vertical MV component range upper bound
	int16_t iMaxVmv;											// Vertical MV component range lower bound
	uint16_t uiMinCR;											// Min compression ration
	int16_t iMaxMvsPer2Mb;										// Max number of motion vectors per two consecutive MBs
};

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
int32_t CheckLevelLimitation(const SLevelLimits* kpLevelLimit,int32_t iTargetBitRate,uint32_t uiPicWidthInMBs,uint32_t uiPicHeightInMBs) {
	uint32_t uiPicInMBs=uiPicWidthInMBs*uiPicHeightInMBs;
	if(kpLevelLimit->uiMaxFS<uiPicInMBs)
		return 0;
	if((kpLevelLimit->uiMaxFS<<3)<(uiPicWidthInMBs*uiPicWidthInMBs))
		return 0;
	if((kpLevelLimit->uiMaxFS<<3)<(uiPicHeightInMBs*uiPicHeightInMBs))
		return 0;
	if(kpLevelLimit->uiMaxDPBMbs<uiPicInMBs)
		return 0;
	if((iTargetBitRate!=UNSPECIFIED_BIT_RATE) && ((int32_t) kpLevelLimit->uiMaxBR *1200)<iTargetBitRate)    //RC enabled,considering bitrate constraint
		return 0;
	return 1;
}

#define WEIGHT_MULTIPLY 2000
#define EPSN (0.000001f) // (1e-6) // desired float precision

#define MAX_BITS_VARY_PERCENTAGE 100			//bits vary range in percentage
#define MAX_BITS_VARY_PERCENTAGE_x3d2 150		//bits vary range in percentage*3/2

#define IDR_BITRATE_RATIO  4

#define FRAME_CMPLX_RATIO_RANGE 20 // *INT_MULTIPLY

const int32_t g_kiQpToQstepTable[52]={
	63,71,79,89,100,112,126,141,159,178,
	200,224,252,283,317,356,400,449,504,566,
	635,713,800,898,1008,1131,1270,1425,1600,1796,
	2016,2263,2540,2851,3200,3592,4032,4525,5080,5702,
	6400,7184,8063,9051,10159,11404,12800,14368,16127,18102,
	20319,22807
};//ROUND(INT_MULTIPLY*pow (2.0,(iQP-4.0)/6.0))

static inline int32_t RcConvertQp2QStep(int32_t iQP) {
	return g_kiQpToQstepTable[iQP];
}

static inline int32_t RcConvertQStep2Qp(int32_t iQpStep) {
	if(iQpStep <=g_kiQpToQstepTable[0]) //Qp step too small,return qp=0
		return 0;
	return ROUND((6*log (iQpStep*1.0f/INT_MULTIPLY)/log (2.0)+4.0));
}

#define LINEAR_MODEL_DECAY_FACTOR 80

void NewH264SVCEncoder::EndIFrame(int32_t iAverageFrameQp,int32_t iLayerSize) {
	//int64_t iFrameComplexity=0;
	int32_t iFrameDqBits=iLayerSize<<3;
	m_svcRcPersist.m_iLastCalculatedQScale=iAverageFrameQp;
	int32_t iQStep=RcConvertQp2QStep(iAverageFrameQp);
	int64_t iIntraCmplx=iQStep*(int64_t)iFrameDqBits;
	if(!m_iFrameNumber) {
		m_svcRcPersist.m_iIntraComplexity=iIntraCmplx;
		m_svcRcPersist.m_iIntraComplxMean=0;
	}else{
		m_svcRcPersist.m_iIntraComplexity=DIV_ROUND64(((LINEAR_MODEL_DECAY_FACTOR)*m_svcRcPersist.m_iIntraComplexity+(INT_MULTIPLY-LINEAR_MODEL_DECAY_FACTOR)*iIntraCmplx),INT_MULTIPLY);
		m_svcRcPersist.m_iIntraComplxMean=DIV_ROUND64(((LINEAR_MODEL_DECAY_FACTOR)*(int64_t)m_svcRcPersist.m_iIntraComplxMean),INT_MULTIPLY);
	}
	if(m_svcRcPersist.m_iIntraComplxMean)
		FATAL("JUST TESTING");
	m_svcRcPersist.m_iIntraMbCount=m_blockWidth*m_blockHeight;
}
void NewH264SVCEncoder::EndPFrame(int32_t iAverageFrameQp,uint32_t frameComplexity,int32_t iLayerSize) {
	int32_t iFrameDqBits=iLayerSize<<3;
	m_svcRcPersist.m_iLastCalculatedQScale=iAverageFrameQp;
	int32_t iQStep=RcConvertQp2QStep(iAverageFrameQp);
	if(m_firstPFrame) {
		m_svcRcPersist.iLinearCmplx=((int64_t)iFrameDqBits)*iQStep;
		m_svcRcPersist.iFrameCmplxMean=(int32_t)frameComplexity;
	}else{
		m_svcRcPersist.iLinearCmplx=DIV_ROUND64(((LINEAR_MODEL_DECAY_FACTOR)*(int64_t)m_svcRcPersist.iLinearCmplx+(INT_MULTIPLY-LINEAR_MODEL_DECAY_FACTOR)*((int64_t)iFrameDqBits*iQStep)),INT_MULTIPLY);
		m_svcRcPersist.iFrameCmplxMean=DIV_ROUND64(((LINEAR_MODEL_DECAY_FACTOR)*static_cast<int64_t>(m_svcRcPersist.iFrameCmplxMean)+(INT_MULTIPLY-LINEAR_MODEL_DECAY_FACTOR)*(int64_t)frameComplexity),INT_MULTIPLY);
	}
}

void NewH264SVCEncoder::RcMbInitGom(MBWindow* windowMb,SRCSlicing* pSOverRc,int32_t numberMbGom,int32_t iMinFrameQp,int32_t iMaxFrameQp) {
	if((windowMb->iMbXY%numberMbGom)==0) {
		if(windowMb->iMbXY) {
			pSOverRc->iComplexityIndexSlice++;
			int64_t iLeftBits=pSOverRc->iTargetBitsSlice-pSOverRc->iFrameBitsSlice;
			int64_t iTargetLeftBits=iLeftBits+pSOverRc->iGomBitsSlice-pSOverRc->iGomTargetBits;
			if((iLeftBits<=0) || (iTargetLeftBits<=0)) {
				pSOverRc->iCalculatedQpSlice+=2;
			}else{
				int64_t iBitsRatio=10000*iLeftBits/(iTargetLeftBits+1);
				if(iBitsRatio<8409)    		//2^(-1.5/6)*10000
					pSOverRc->iCalculatedQpSlice+=2;
				else
				if(iBitsRatio<9439)   	//2^(-0.5/6)*10000
					pSOverRc->iCalculatedQpSlice+=1;
				else
				if(iBitsRatio>10600)  	//2^(0.5/6)*10000
					pSOverRc->iCalculatedQpSlice-=1;
				else
				if(iBitsRatio>11900)  	//2^(1.5/6)*10000
					pSOverRc->iCalculatedQpSlice-=2;
			}
			pSOverRc->iCalculatedQpSlice=CLIP3(pSOverRc->iCalculatedQpSlice,iMinFrameQp,iMaxFrameQp);
			pSOverRc->iGomBitsSlice=0;
		}
		int32_t iLeftBits=pSOverRc->iTargetBitsSlice-pSOverRc->iFrameBitsSlice;
		if(iLeftBits<=0) {
			pSOverRc->iGomTargetBits=0;
		}else{
			int32_t d=(((m_blockWidth*m_blockHeight)-1)/numberMbGom)-pSOverRc->iComplexityIndexSlice;
			if(d<=0) {
				pSOverRc->iGomTargetBits=iLeftBits;
			}else{
				pSOverRc->iGomTargetBits=DIV_ROUND(iLeftBits,d);
			}
		}
	}
	int32_t iLumaQp=pSOverRc->iCalculatedQpSlice;
	windowMb->m_mb->uiLumaQp=iLumaQp;
}

int32_t NewH264SVCEncoder::InitPFrame(int32_t* iMinFrameQp,int32_t* iMaxFrameQp,int32_t* iTargetBits,uint32_t frameComplexity) {
	int32_t iBitsPerFrame=DIV_ROUND(m_iTargetBitrate,1.0f);
	int32_t iMinBitsRatio=MAX_BITS_VARY_PERCENTAGE-((MAX_BITS_VARY_PERCENTAGE-10)>>1);
	int32_t iMaxBitsRatio=MAX_BITS_VARY_PERCENTAGE_x3d2;
	const int64_t kdConstraitBits=((int64_t)iBitsPerFrame)*WEIGHT_MULTIPLY;
	int32_t iMinBitsTl=DIV_ROUND(kdConstraitBits*iMinBitsRatio,MAX_BITS_VARY_PERCENTAGE*WEIGHT_MULTIPLY);
	int32_t iMaxBitsTl=DIV_ROUND(kdConstraitBits*iMaxBitsRatio,MAX_BITS_VARY_PERCENTAGE*WEIGHT_MULTIPLY);
	int32_t iRemainingBits=VGOP_SIZE*iBitsPerFrame;
	*iTargetBits=DIV_ROUND(static_cast<int64_t>(iRemainingBits*WEIGHT_MULTIPLY),VGOP_SIZE*WEIGHT_MULTIPLY);
	*iTargetBits=CLIP3(*iTargetBits,iMinBitsTl,iMaxBitsTl);
	int32_t iLumaQp=0;
	if(m_firstPFrame) {
		iLumaQp=m_svcRcPersist.m_iInitialQp;
	}else{
		int64_t iFrameComplexity=frameComplexity;
		int64_t iCmplxRatio=DIV_ROUND64(iFrameComplexity*INT_MULTIPLY,m_svcRcPersist.iFrameCmplxMean);
		iCmplxRatio=CLIP3(iCmplxRatio,INT_MULTIPLY-FRAME_CMPLX_RATIO_RANGE,INT_MULTIPLY+FRAME_CMPLX_RATIO_RANGE);
		int32_t iQStep=DIV_ROUND((m_svcRcPersist.iLinearCmplx*iCmplxRatio),(*iTargetBits*INT_MULTIPLY));
		iLumaQp=RcConvertQStep2Qp(iQStep);
	}
	*iMinFrameQp=CLIP3(m_svcRcPersist.m_iLastCalculatedQScale-3,GOM_MIN_QP_MODE,MAX_LOW_BR_QP);
	*iMaxFrameQp=CLIP3(m_svcRcPersist.m_iLastCalculatedQScale+5,GOM_MIN_QP_MODE,MAX_LOW_BR_QP);
	iLumaQp=CLIP3(iLumaQp,*iMinFrameQp,*iMaxFrameQp);
	m_svcRcPersist.m_iLastCalculatedQScale=iLumaQp;
	return iLumaQp;
}

int32_t NewH264SVCEncoder::InitIFrame(int32_t* iMinFrameQp,int32_t* iMaxFrameQp,int32_t* iTargetBits) {
	int32_t iBitsPerFrame=DIV_ROUND(m_iTargetBitrate,1.0f);
	*iTargetBits=iBitsPerFrame*IDR_BITRATE_RATIO;
	float dBpp=0;
//64k@6fps for 90p:     bpp 0.74    QP:24
//192k@12fps for 180p:  bpp 0.28    QP:26
//512k@24fps for 360p:  bpp 0.09    QP:30
//1500k@30fps for 720p: bpp 0.05    QP:32
	double dBppArray[4][3]={{0.5,0.75,1.0},{0.2,0.3,0.4},{0.05,0.09,0.13},{0.03,0.06,0.1}};
	int32_t dInitialQPArray[4][4]={{28,26,24,22},{30,28,26,24},{32,30,28,26},{34,32,30,28}};
	int32_t iBppIndex=0;
	int32_t iQpRangeArray[4][2]={{37,25},{36,24},{35,23},{34,22}};
	if(1.0f>EPSN && m_videoWidth && m_videoHeight)
		dBpp=(float)((double)(m_iTargetBitrate)/(double)(1.0*m_videoWidth*m_videoHeight));
	else
		dBpp=0.1;
	if(m_videoWidth*m_videoHeight<=28800)		// 90p video:160*90
		iBppIndex=0;
	else
	if(m_videoWidth*m_videoHeight<=115200)		// 180p video:320*180
		iBppIndex=1;
	else
	if(m_videoWidth*m_videoHeight<=460800)		// 360p video:640*360
		iBppIndex=2;
	else
		iBppIndex=3;
	int32_t i;
	for(i=0;i<3;i++) {
		if(dBpp<=dBppArray[iBppIndex][i])
			break;
	}
	int32_t iMaxQp=iQpRangeArray[i][0];
	int32_t iMinQp=iQpRangeArray[i][1];
	iMinQp=CLIP3(iMinQp,GOM_MIN_QP_MODE,MAX_LOW_BR_QP);
	iMaxQp=CLIP3(iMaxQp,GOM_MIN_QP_MODE,MAX_LOW_BR_QP);
	if(!m_iFrameNumber) { //the first IDR frame
		m_svcRcPersist.m_iInitialQp=dInitialQPArray[iBppIndex][i];
	}else{
		//obtain the idr qp using previous idr complexity
		if((m_blockWidth*m_blockHeight)!=m_svcRcPersist.m_iIntraMbCount) {
			m_svcRcPersist.m_iIntraComplexity=m_svcRcPersist.m_iIntraComplexity*(m_blockWidth*m_blockHeight)/m_svcRcPersist.m_iIntraMbCount;
		}
		int64_t iFrameComplexity=0;
		int64_t iCmplxRatio=DIV_ROUND64(iFrameComplexity*INT_MULTIPLY,m_svcRcPersist.m_iIntraComplxMean);
		iCmplxRatio=CLIP3(iCmplxRatio,INT_MULTIPLY-FRAME_CMPLX_RATIO_RANGE,INT_MULTIPLY+FRAME_CMPLX_RATIO_RANGE);
		int32_t iQStep=DIV_ROUND((m_svcRcPersist.m_iIntraComplexity*iCmplxRatio),(*iTargetBits*INT_MULTIPLY));
		m_svcRcPersist.m_iInitialQp=RcConvertQStep2Qp(iQStep);
	}
	m_svcRcPersist.m_iInitialQp=CLIP3(m_svcRcPersist.m_iInitialQp,iMinQp,iMaxQp);
	int32_t iGlobalQp=m_svcRcPersist.m_iInitialQp;
	m_svcRcPersist.m_iLastCalculatedQScale=iGlobalQp;
	*iMinFrameQp=CLIP3(iGlobalQp-DELTA_QP_BGD_THD,iMinQp,iMaxQp);
	*iMaxFrameQp=CLIP3(iGlobalQp+DELTA_QP_BGD_THD,iMinQp,iMaxQp);
	return iGlobalQp;
}

//initialize Wels avc encoder core library
void NewH264SVCEncoder::InitEncoderExt() {
	memset(&m_svcRcPersist,0,sizeof(m_svcRcPersist));
	AllocStrideTables();
	GetEncBlockStrideOffset(m_strideTable.m_pStrideEncBlockOffset,m_stride[0],m_stride[1]);
	m_intraFrameIndex=0;
	int32_t iNonVclLayersBsSizeCount=SSEI_BUFFER_SIZE+SPS_BUFFER_SIZE+PPS_BUFFER_SIZE;
	int32_t iLayerBsSize=ROUND(((3*m_videoWidth*m_videoHeight)>>1)*COMPRESS_RATIO_THR)+MAX_MACROBLOCK_SIZE_IN_BYTE_x2;
	iLayerBsSize=ALIGN_WITH(iLayerBsSize,4);// 4 bytes alinged
	int32_t iVclLayersBsSizeCount=iLayerBsSize;
	int32_t iCountBsLen=iNonVclLayersBsSizeCount+iVclLayersBsSizeCount;
	m_frameBs=(uint8_t*)Malloc(iCountBsLen);
	m_frameBsSize=iCountBsLen;
	m_posBsBuffer=0;
	m_refPics[0]=AllocPicture(true);
	m_refPics[1]=AllocPicture(true);
	m_prev=AllocPicture(false);
	m_cur=AllocPicture(false);
}

#define NRI_PRI_HIGHEST 3

int32_t NewH264SVCEncoder::WriteNal(SBitStringAux* pBitStringAux,uint8_t* pBsBuffer,int32_t iStartPos,const int32_t kiType) {
	const int32_t kiEndPos=BsGetBitsPos(pBitStringAux)>>3;
	uint8_t* pDstStart=m_frameBs+m_posBsBuffer;
	uint8_t* pDstPointer=pDstStart;
	uint8_t* pSrcPointer=pBsBuffer+iStartPos;
	uint8_t* pSrcEnd=pSrcPointer+(kiEndPos-iStartPos);
	static const uint8_t kuiStartCodePrefix[NAL_HEADER_SIZE]={0,0,0,1};
	ST32(pDstPointer,LD32(&kuiStartCodePrefix[0]));
	pDstPointer+=4;
	*pDstPointer++=(NRI_PRI_HIGHEST<<5)|(kiType&0x1f);			// NAL Unit Header
	int32_t iZeroCount=0;
	while(pSrcPointer<pSrcEnd) {
		if(iZeroCount==2 && *pSrcPointer<=3) {
			*pDstPointer++=3;									// add the code 03
			iZeroCount=0;
		}
		if(*pSrcPointer==0) {
			++iZeroCount;
		}else{
			iZeroCount=0;
		}
		*pDstPointer++=*pSrcPointer++;
	}
	int32_t iNalLength=(int32_t)(pDstPointer-pDstStart);		// count length of NAL Unit
	m_posBsBuffer+=iNalLength;
	return iNalLength;
}

int32_t NewH264SVCEncoder::WriteVUI(SBitStringAux* pBitStringAux) {
	assert(pBitStringAux!=NULL);
	BsWriteOneBit(pBitStringAux,false);							// aspect_ratio_info_present_flag
	BsWriteOneBit(pBitStringAux,false);							// overscan_info_present_flag
	BsWriteOneBit(pBitStringAux,false);							// video_signal_type_present_flag
	BsWriteOneBit(pBitStringAux,false);							// chroma_loc_info_present_flag
	BsWriteOneBit(pBitStringAux,true);							// timing_info_present_flag
	BsWriteBits(pBitStringAux,32,1000);							// num_units_in_tick (time_scale/(2*num_units_in_tick) = 60000/1000 = 30 fps)
	BsWriteBits(pBitStringAux,32,60000);						// time_scale
	BsWriteOneBit(pBitStringAux,true);							// fixed_frame_rate_flag
	BsWriteOneBit(pBitStringAux,false);							// nal_hrd_parameters_present_flag
	BsWriteOneBit(pBitStringAux,false);							// vcl_hrd_parameters_present_flag
	BsWriteOneBit(pBitStringAux,false);							// pic_struct_present_flag
	BsWriteOneBit(pBitStringAux,true);							// bitstream_restriction_flag
	BsWriteOneBit(pBitStringAux,true);							// motion_vectors_over_pic_boundaries_flag
	BsWriteUE(pBitStringAux,0);									// max_bytes_per_pic_denom
	BsWriteUE(pBitStringAux,0);									// max_bits_per_mb_denom
	BsWriteUE(pBitStringAux,16);								// log2_max_mv_length_horizontal
	BsWriteUE(pBitStringAux,16);								// log2_max_mv_length_vertical
	BsWriteUE(pBitStringAux,0);									// max_num_reorder_frames
	BsWriteUE(pBitStringAux,1);									// max_dec_frame_buffering
	return 0;
}
void NewH264SVCEncoder::WriteSpsNal(SBitStringAux* pBitStringAux,int32_t iSpsIdDelta) {
	const SLevelLimits* pLevelLimit=&g_ksLevelLimits[countof(g_ksLevelLimits)-1];
	for(int i=0;i<countof(g_ksLevelLimits);i++) {
		if(CheckLevelLimitation(&g_ksLevelLimits[i],m_iTargetBitrate,m_blockWidth,m_blockHeight)) {
			pLevelLimit=&g_ksLevelLimits[i];
			break;
		}
	}
	//uint8_t* p=pBitStringAux->pCurBuf;
	assert(pBitStringAux!=NULL);
	BsWriteBits(pBitStringAux,8,77);							// PRO_MAIN
	BsWriteOneBit(pBitStringAux,false);							// bConstraintSet0Flag
	BsWriteOneBit(pBitStringAux,true);							// bConstraintSet1Flag
	BsWriteOneBit(pBitStringAux,false);							// bConstraintSet2Flag
	BsWriteOneBit(pBitStringAux,false);							// bConstraintSet3Flag
	BsWriteOneBit(pBitStringAux,1);								// bConstraintSet4Flag: If profile_idc is equal to 77,88,or 100,constraint_set4_flag equal to 1 indicates that the value of frame_mbs_only_flag is equal to 1. constraint_set4_flag equal to 0 indicates that the value of frame_mbs_only_flag may or may not be equal to 1.
	BsWriteOneBit(pBitStringAux,1);								// bConstraintSet5Flag: If profile_idc is equal to 77,88,or 100,constraint_set5_flag equal to 1 indicates that B slice types are not present in the coded video sequence. constraint_set5_flag equal to 0 indicates that B slice types may or may not be present in the coded video sequence.
	BsWriteBits(pBitStringAux,2,0);								// reserved_zero_2bits,equal to 0
	BsWriteBits(pBitStringAux,8,pLevelLimit->uiLevelIdc);		// iLevelIdc
	BsWriteUE(pBitStringAux,iSpsIdDelta);						// seq_parameter_set_id
	BsWriteUE(pBitStringAux,SPS_LOG2_MAX_FRAME_NUM-4);			// log2_max_frame_num_minus4
	BsWriteUE(pBitStringAux,0);									// pic_order_cnt_type
	BsWriteUE(pBitStringAux,(1+SPS_LOG2_MAX_FRAME_NUM)-4);		// log2_max_pic_order_cnt_lsb_minus4
	BsWriteUE(pBitStringAux,1);									// max_num_ref_frames
	BsWriteOneBit(pBitStringAux,false);							// gaps_in_frame_numvalue_allowed_flag
	BsWriteUE(pBitStringAux,m_blockWidth-1);					// pic_width_in_mbs_minus1
	BsWriteUE(pBitStringAux,m_blockHeight-1);					// pic_height_in_map_units_minus1
	BsWriteOneBit(pBitStringAux,true);							// bFrameMbsOnlyFlag
	BsWriteOneBit(pBitStringAux,0);								// direct_8x8_inference_flag
	BsWriteOneBit(pBitStringAux,false);							// bFrameCroppingFlag
	if(true) {													// include VUI
		BsWriteOneBit(pBitStringAux,true);						// vui_parameters_present_flag
		WriteVUI(pBitStringAux);
	}else{
		BsWriteOneBit(pBitStringAux,false);
	}
	BsRbspTrailingBits(pBitStringAux);
}

#define MAX_SPS_COUNT 1										// Count number of SPS
#define MAX_PPS_COUNT 1										// Standard is 256
#define PIC_INIT_QP 26
#define PIC_INIT_QS 26

// write Picture Parameter Set (PPS)
void NewH264SVCEncoder::WritePpsNal(SBitStringAux* pBitStringAux) {
	BsWriteUE(pBitStringAux,m_iFrameNumber%MAX_PPS_COUNT);
	BsWriteUE(pBitStringAux,m_iFrameNumber%MAX_SPS_COUNT);
	BsWriteOneBit(pBitStringAux,true);							// EntropyCodingModeFlag
	BsWriteOneBit(pBitStringAux,false);
	BsWriteUE(pBitStringAux,0);
	BsWriteUE(pBitStringAux,0);
	BsWriteUE(pBitStringAux,0);
	BsWriteOneBit(pBitStringAux,false);
	BsWriteBits(pBitStringAux,2,0);
	BsWriteSE(pBitStringAux,PIC_INIT_QP-26);
	BsWriteSE(pBitStringAux,PIC_INIT_QS-26);
	BsWriteSE(pBitStringAux,0);									// ChromaQpIndexOffset
	BsWriteOneBit(pBitStringAux,true);							// deblocking_filter_control_present_flag
	BsWriteOneBit(pBitStringAux,false);							// constrained_intra_pred_flag
	BsWriteOneBit(pBitStringAux,false);							// redundant_pic_cnt_present_flag
	BsRbspTrailingBits(pBitStringAux);
}


ALIGNED_DECLARE (const int16_t,g_kiQuantInterFF[58][8],16)={
	{   0,1,0,1,1,1,1,1 },		// 0
	{   0,1,0,1,1,1,1,1 },		// 1
	{   1,1,1,1,1,1,1,1 },		// 2
	{   1,1,1,1,1,1,1,1 },		// 3
	{   1,1,1,1,1,2,1,2 },		// 4
	{   1,1,1,1,1,2,1,2 },		// 5
	{   1,1,1,1,1,2,1,2 },		// 6
	{   1,1,1,1,1,2,1,2 },		// 7
	{   1,2,1,2,2,3,2,3 },		// 8
	{   1,2,1,2,2,3,2,3 },		// 9
	{   1,2,1,2,2,3,2,3 },		//10
	{   1,2,1,2,2,4,2,4 },		//11
	{   2,3,2,3,3,4,3,4 },		//12
	{   2,3,2,3,3,5,3,5 },		//13
	{   2,3,2,3,3,5,3,5 },		//14
	{   2,4,2,4,4,6,4,6 },		//15
	{   3,4,3,4,4,7,4,7 },		//16
	{   3,5,3,5,5,8,5,8 },		//17
	{   3,5,3,5,5,8,5,8 },		//18
	{   4,6,4,6,6,9,6,9 },		//19
	{   4,7,4,7,7,10,7,10 },		//20
	{   5,8,5,8,8,12,8,12 },		//21
	{   5,8,5,8,8,13,8,13 },		//22
	{   6,10,6,10,10,15,10,15 },		//23
	{   7,11,7,11,11,17,11,17 },		//24
	{   7,12,7,12,12,19,12,19 },		//25
	{   9,13,9,13,13,21,13,21 },		//26
	{   9,15,9,15,15,24,15,24 },		//27
	{  11,17,11,17,17,26,17,26 },		//28
	{  12,19,12,19,19,30,19,30 },		//29
	{  13,22,13,22,22,33,22,33 },		//30
	{  15,23,15,23,23,38,23,38 },		//31
	{  17,27,17,27,27,42,27,42 },		//32
	{  19,30,19,30,30,48,30,48 },		//33
	{  21,33,21,33,33,52,33,52 },		//34
	{  24,38,24,38,38,60,38,60 },		//35
	{  27,43,27,43,43,67,43,67 },		//36
	{  29,47,29,47,47,75,47,75 },		//37
	{  35,53,35,53,53,83,53,83 },		//38
	{  37,60,37,60,60,96,60,96 },		//39
	{  43,67,43,67,67,104,67,104 },		//40
	{  48,77,48,77,77,121,77,121 },		//41
	{  53,87,53,87,87,133,87,133 },		//42
	{  59,93,59,93,93,150,93,150 },		//43
	{  69,107,69,107,107,167,107,167 },		//44
	{  75,120,75,120,120,192,120,192 },		//45
	{  85,133,85,133,133,208,133,208 },		//46
	{  96,153,96,153,153,242,153,242 },		//47
	{ 107,173,107,173,173,267,173,267 },		//48
	{ 117,187,117,187,187,300,187,300 },		//49
	{ 139,213,139,213,213,333,213,333 },		//50
	{ 149,240,149,240,240,383,240,383 },		//51
// from here below is only for intra
	{ 171,267,171,267,267,417,267,417 },		//46
	{ 192,307,192,307,307,483,307,483 },		//47
	{ 213,347,213,347,347,533,347,533 },		//48
	{ 235,373,235,373,373,600,373,600 },		//49
	{ 277,427,277,427,427,667,427,667 },		//50
	{ 299,480,299,480,480,767,480,767 },		//51
};

#define g_iQuantIntraFF (g_kiQuantInterFF+6 )

ALIGNED_DECLARE (const int16_t,g_kiQuantMF[52][8],16)={
	{26214,16132,26214,16132,16132,10486,16132,10486 },		// 0
	{23832,14980,23832,14980,14980,9320,14980,9320 },		// 1
	{20164,13108,20164,13108,13108,8388,13108,8388 },		// 2
	{18724,11650,18724,11650,11650,7294,11650,7294 },		// 3
	{16384,10486,16384,10486,10486,6710,10486,6710 },		// 4
	{14564,9118,14564,9118,9118,5786,9118,5786 },		// 5
	{13107,8066,13107,8066,8066,5243,8066,5243 },		// 6
	{11916,7490,11916,7490,7490,4660,7490,4660 },		// 7
	{10082,6554,10082,6554,6554,4194,6554,4194 },		// 8
	{ 9362,5825,9362,5825,5825,3647,5825,3647 },		// 9
	{ 8192,5243,8192,5243,5243,3355,5243,3355 },		//10
	{ 7282,4559,7282,4559,4559,2893,4559,2893 },		//11
	{ 6554,4033,6554,4033,4033,2622,4033,2622 },		//12
	{ 5958,3745,5958,3745,3745,2330,3745,2330 },		//13
	{ 5041,3277,5041,3277,3277,2097,3277,2097 },		//14
	{ 4681,2913,4681,2913,2913,1824,2913,1824 },		//15
	{ 4096,2622,4096,2622,2622,1678,2622,1678 },		//16
	{ 3641,2280,3641,2280,2280,1447,2280,1447 },		//17
	{ 3277,2017,3277,2017,2017,1311,2017,1311 },		//18
	{ 2979,1873,2979,1873,1873,1165,1873,1165 },		//19
	{ 2521,1639,2521,1639,1639,1049,1639,1049 },		//20
	{ 2341,1456,2341,1456,1456,912,1456,912 },		//21
	{ 2048,1311,2048,1311,1311,839,1311,839 },		//22
	{ 1821,1140,1821,1140,1140,723,1140,723 },		//23
	{ 1638,1008,1638,1008,1008,655,1008,655 },		//24
	{ 1490,936,1490,936,936,583,936,583 },		//25
	{ 1260,819,1260,819,819,524,819,524 },		//26
	{ 1170,728,1170,728,728,456,728,456 },		//27
	{ 1024,655,1024,655,655,419,655,419 },		//28
	{  910,570,910,570,570,362,570,362 },		//29
	{  819,504,819,504,504,328,504,328 },		//30
	{  745,468,745,468,468,291,468,291 },		//31
	{  630,410,630,410,410,262,410,262 },		//32
	{  585,364,585,364,364,228,364,228 },		//33
	{  512,328,512,328,328,210,328,210 },		//34
	{  455,285,455,285,285,181,285,181 },		//35
	{  410,252,410,252,252,164,252,164 },		//36
	{  372,234,372,234,234,146,234,146 },		//37
	{  315,205,315,205,205,131,205,131 },		//38
	{  293,182,293,182,182,114,182,114 },		//39
	{  256,164,256,164,164,105,164,105 },		//40
	{  228,142,228,142,142,90,142,90 },		//41
	{  205,126,205,126,126,82,126,82 },		//42
	{  186,117,186,117,117,73,117,73 },		//43
	{  158,102,158,102,102,66,102,66 },		//44
	{  146,91,146,91,91,57,91,57 },		//45
	{  128,82,128,82,82,52,82,52 },		//46
	{  114,71,114,71,71,45,71,45 },		//47
	{  102,63,102,63,63,41,63,41 },		//48
	{   93,59,93,59,59,36,59,36 },		//49
	{   79,51,79,51,51,33,51,33 },		//50
	{   73,46,73,46,46,28,46,28 }		//51
};

static const ALIGNED_DECLARE (int8_t,g_kiIntra16AvaliMode[8][5],16)={
	{ I16_PRED_DC_128,I16_PRED_INVALID,I16_PRED_INVALID,I16_PRED_INVALID,1 },
	{ I16_PRED_DC_L,I16_PRED_H,I16_PRED_INVALID,I16_PRED_INVALID,2 },
	{ I16_PRED_DC_T,I16_PRED_V,I16_PRED_INVALID,I16_PRED_INVALID,2 },
	{ I16_PRED_V,I16_PRED_H,I16_PRED_DC,I16_PRED_INVALID,3 },
	{ I16_PRED_DC_128,I16_PRED_INVALID,I16_PRED_INVALID,I16_PRED_INVALID,1 },
	{ I16_PRED_DC_L,I16_PRED_H,I16_PRED_INVALID,I16_PRED_INVALID,2 },
	{ I16_PRED_DC_T,I16_PRED_V,I16_PRED_INVALID,I16_PRED_INVALID,2 },
	{ I16_PRED_V,I16_PRED_H,I16_PRED_DC,I16_PRED_P,4 }
};

uint32_t NewH264SVCEncoder::GetNeighborIntra(const MBWindow& mbWindow) {
	uint32_t uiNeighborIntra=0;
	if(mbWindow.HasNeightbor(N_LEFT)) {
		uiNeighborIntra|=LEFT_MB_POS;
	}
	if(mbWindow.HasNeightbor(N_TOP)) {
		uiNeighborIntra|=TOP_MB_POS;
	}
	if(mbWindow.HasNeightbor(N_TOPLEFT)) {
		uiNeighborIntra|=0x04;
	}
	return uiNeighborIntra;
}

int32_t NewH264SVCEncoder::MdI16x16(SMbCache* pMbCache,int32_t iLambda,const MBWindow& windowMb) {
	const uint8_t* pDec=pMbCache->m_decMb[0];
	const uint8_t* pEnc=pMbCache->m_encMb[0];
	int32_t iBestCost=INT_MAX;
	int32_t iOffset=GetNeighborIntra(windowMb);
	int32_t iAvailCount=g_kiIntra16AvaliMode[iOffset][4];				// Max 4
	const int8_t* kpAvailMode=g_kiIntra16AvaliMode[iOffset];
	int32_t iBestMode=kpAvailMode[0];
	for(int32_t i=0;i<iAvailCount;++i) {
		int32_t iCurMode=kpAvailMode[i];
		assert(iCurMode>=0 && iCurMode<7);
		WelsILuma16x16Pred_c(iCurMode,pMbCache->pMemPredChroma,pDec,m_stride[0]);
		int32_t iCurCost=SampleSad16x16_c(pMbCache->pMemPredChroma,16,pEnc,m_stride[0]);
		iCurCost+=iLambda*(BsSizeUE(g_kiMapModeI16x16[iCurMode]));
		if(iCurCost<iBestCost) {
			iBestMode=iCurMode;
			iBestCost=iCurCost;
			uint8_t* p=pMbCache->pMemPredChroma;
			pMbCache->pMemPredChroma=pMbCache->pMemPredLuma;
			pMbCache->pMemPredLuma=p;
		}
	}
	pMbCache->uiLumaI16x16Mode=iBestMode;
	return iBestCost;
}

static const ALIGNED_DECLARE (int8_t,g_kiIntraChromaAvailMode[8][5],16)={
	{ C_PRED_DC_128,C_PRED_INVALID,C_PRED_INVALID,C_PRED_INVALID,1 },
	{ C_PRED_DC_L,C_PRED_H,C_PRED_INVALID,C_PRED_INVALID,2 },
	{ C_PRED_DC_T,C_PRED_V,C_PRED_INVALID,C_PRED_INVALID,2 },
	{ C_PRED_V,C_PRED_H,C_PRED_DC,C_PRED_INVALID,3 },
	{ C_PRED_DC_128,C_PRED_INVALID,C_PRED_INVALID,C_PRED_INVALID,1 },
	{ C_PRED_DC_L,C_PRED_H,C_PRED_INVALID,C_PRED_INVALID,2 },
	{ C_PRED_DC_T,C_PRED_V,C_PRED_INVALID,C_PRED_INVALID,2 },
	{ C_PRED_V,C_PRED_H,C_PRED_DC,C_PRED_P,4 }
};

uint8_t NewH264SVCEncoder::MdIntraChroma(SMbCache* pMbCache,int32_t iLambda,const MBWindow& windowMb) {
	uint8_t* p1=pMbCache->pMemPredChroma;
	uint8_t* p0=pMbCache->pMemPredChroma+128;
	const uint8_t* pEncCb=pMbCache->m_encMb[1];
	const uint8_t* pEncCr=pMbCache->m_encMb[2];
	const uint8_t* pDecCb=pMbCache->m_decMb[1];
	const uint8_t* pDecCr=pMbCache->m_decMb[2];
	int32_t iBestCost=INT_MAX;
	int32_t iOffset=GetNeighborIntra(windowMb);
	int32_t iAvailCount=g_kiIntraChromaAvailMode[iOffset][4];
	const int8_t* kpAvailMode=g_kiIntraChromaAvailMode[iOffset];
	int32_t iBestMode=kpAvailMode[0];
	for(int32_t i=0;i<iAvailCount;++i) {
		int32_t iCurMode=kpAvailMode[i];
		assert(iCurMode >=0 && iCurMode<7);
		WelsIChromaPred_c(iCurMode,p0,pDecCb,m_stride[1]);			// Cb
		int32_t iCurCost=SampleSad8x8_c(p0,8,pEncCb,m_stride[1]);
		WelsIChromaPred_c(iCurMode,p0+64,pDecCr,m_stride[1]);		// Cr
		iCurCost+=SampleSad8x8_c(p0+64,8,pEncCr,m_stride[1])+iLambda*BsSizeUE(g_kiMapModeIntraChroma[iCurMode]);
		if(iCurCost<iBestCost) {
			iBestMode=iCurMode;
			iBestCost=iCurCost;
			uint8_t* p=p0;p0=p1;p1=p;
		}
	}
	pMbCache->pBestPredIntraChroma=p1;
	return iBestMode;
}

void NewH264SVCEncoder::DctMb(int16_t* pRes,uint8_t* pEncMb,int32_t iEncStride,uint8_t* pBestPred) {
	WelsDctFourT4_c(pRes,pEncMb,iEncStride,pBestPred,16);
	WelsDctFourT4_c(pRes+64,pEncMb+8,iEncStride,pBestPred+8,16);
	WelsDctFourT4_c(pRes+128,pEncMb+8*iEncStride,iEncStride,pBestPred+128,16);
	WelsDctFourT4_c(pRes+192,pEncMb+8*iEncStride+8,iEncStride,pBestPred+136,16);
}

void NewH264SVCEncoder::EncRecI16x16Y(const MBWindow& pCurMb,SMbCache* pMbCache) {
	ENFORCE_STACK_ALIGN_1D(int16_t,aDctT4Dc,16,16)
	const int32_t kiEncStride=m_stride[0];
	int16_t* pRes=pMbCache->m_coeffLevel;
	uint8_t* pPred=pMbCache->m_decMb[0];
	const int32_t kiRecStride=m_stride[0];
	int16_t* pBlock=pMbCache->m_dct.iLumaBlock[0];
	uint8_t* pBestPred=pMbCache->pMemPredLuma;
	const uint8_t* kpNoneZeroCountIdx=&g_kuiMbCountScan4Idx[0];
	uint8_t i,uiQp=pCurMb.m_mb->uiLumaQp;
	uint32_t uiNoneZeroCount,uiNoneZeroCountMbAc=0,uiCountI16x16Dc;
	const int16_t* pMF=g_kiQuantMF[uiQp];
	const int16_t* pFF=g_iQuantIntraFF[uiQp];
	DctMb(pRes,pMbCache->m_encMb[0],kiEncStride,pBestPred);
	WelsHadamardT4Dc_c(aDctT4Dc,pRes);
	WelsQuant4x4Dc_c(aDctT4Dc,pFF[0]<<1,pMF[0]>>1);
	WelsScan4x4DcAc_c(pMbCache->m_dct.iLumaI16x16Dc,aDctT4Dc);
	uiCountI16x16Dc=WelsGetNoneZeroCount_c(pMbCache->m_dct.iLumaI16x16Dc);
	for(i=0;i<4;i++) {
		WelsQuantFour4x4_c(pRes,pFF,pMF);
		WelsScan4x4Ac_c(pBlock,pRes);
		WelsScan4x4Ac_c(pBlock+16,pRes+16);
		WelsScan4x4Ac_c(pBlock+32,pRes+32);
		WelsScan4x4Ac_c(pBlock+48,pRes+48);
		pRes+=64;
		pBlock+=64;
	}
	pRes-=256;
	pBlock-=256;
	for(i=0;i<16;i++) {
		uiNoneZeroCount=WelsGetNoneZeroCount_c(pBlock);
		pCurMb.m_mb->m_nonZeroCount[*kpNoneZeroCountIdx++]=uiNoneZeroCount;
		uiNoneZeroCountMbAc+=uiNoneZeroCount;
		pBlock+=16;
	}
	if(uiCountI16x16Dc>0) {
		if(uiQp<12) {
			FATAL("JUST TESTING");
			WelsIHadamard4x4Dc(aDctT4Dc);
			WelsDequantLumaDc4x4(aDctT4Dc,uiQp);
		}else
			WelsDequantIHadamard4x4_c(aDctT4Dc,g_kuiDequantCoeff[uiQp][0]>>2);
	}
	if(uiNoneZeroCountMbAc>0) {
		pCurMb.m_mb->uiCbp=15;
		WelsDequantFour4x4_c(pRes,g_kuiDequantCoeff[uiQp]);
		WelsDequantFour4x4_c(pRes+64,g_kuiDequantCoeff[uiQp]);
		WelsDequantFour4x4_c(pRes+128,g_kuiDequantCoeff[uiQp]);
		WelsDequantFour4x4_c(pRes+192,g_kuiDequantCoeff[uiQp]);
		pRes[0]=aDctT4Dc[0];
		pRes[16]=aDctT4Dc[1];
		pRes[32]=aDctT4Dc[4];
		pRes[48]=aDctT4Dc[5];
		pRes[64]=aDctT4Dc[2];
		pRes[80]=aDctT4Dc[3];
		pRes[96]=aDctT4Dc[6];
		pRes[112]=aDctT4Dc[7];
		pRes[128]=aDctT4Dc[8];
		pRes[144]=aDctT4Dc[9];
		pRes[160]=aDctT4Dc[12];
		pRes[176]=aDctT4Dc[13];
		pRes[192]=aDctT4Dc[10];
		pRes[208]=aDctT4Dc[11];
		pRes[224]=aDctT4Dc[14];
		pRes[240]=aDctT4Dc[15];
		WelsIDctFourT4Rec_c(pPred,kiRecStride,pBestPred,16,pRes);
		WelsIDctFourT4Rec_c(pPred+8, kiRecStride,pBestPred+8,16,pRes+64);
		WelsIDctFourT4Rec_c(pPred+kiRecStride*8,kiRecStride,pBestPred+128,16,pRes+128);
		WelsIDctFourT4Rec_c(pPred+kiRecStride*8+8,kiRecStride,pBestPred+136,16,pRes+192);
	}else
	if(uiCountI16x16Dc>0) {
		WelsIDctRecI16x16Dc_c(pPred,kiRecStride,pBestPred,16,aDctT4Dc);
	}else{
		Copy16x16_c(pPred,kiRecStride,pBestPred,16);
	}
}

void NewH264SVCEncoder::EncRecUV(const MBWindow& pCurMb,SMbCache* pMbCache,int16_t* pRes,int32_t iUV) {
	const int32_t kiInterFlag=!IS_INTRA(pCurMb.m_mb->uiMbType);
	const uint8_t kiQp=pCurMb.m_mb->GetChromaQp();
	uint8_t i,uiNoneZeroCount,uiNoneZeroCountMbAc=0,uiNoneZeroCountMbDc=0;
	uint8_t uiNoneZeroCountOffset=(iUV-1)<<1;  //UV==1 or 2
	uint8_t uiSubMbIdx=16+((iUV-1)<<2);//uiSubMbIdx==16 or 20
	int16_t* iChromaDc=pMbCache->m_dct.iChromaDc[iUV-1],*pBlock=pMbCache->m_dct.iChromaBlock[ (iUV-1)<<2];
	int16_t aDct2x2[4],j,aMax[4];
	int32_t iSingleCtr8x8=0;
	const int16_t* pMF=g_kiQuantMF[kiQp];
	const int16_t* pFF=g_kiQuantInterFF[(!kiInterFlag)*6+kiQp];
	uiNoneZeroCountMbDc=WelsHadamardQuant2x2_c(pRes,pFF[0]<<1,pMF[0]>>1,aDct2x2,iChromaDc);
	WelsQuantFour4x4Max_c(pRes,pFF,pMF,aMax);
	for(j=0;j<4;j++) {
		if(aMax[j]==0)
			memset(pBlock,0,32);
		else {
			WelsScan4x4Ac_c(pBlock,pRes);
			if(kiInterFlag) {
				if(aMax[j]>1)
					iSingleCtr8x8+=9;
				else
				if(iSingleCtr8x8<7)
					iSingleCtr8x8+=WelsCalculateSingleCtr4x4_c(pBlock);
			}else
				iSingleCtr8x8=INT_MAX;
		}
		pRes+=16;
		pBlock+=16;
	}
	pRes-=64;

	if(iSingleCtr8x8<7) { //from JVT-O079
		memset(pRes,0,128);
		ST16(&pCurMb.m_mb->m_nonZeroCount[16+uiNoneZeroCountOffset],0);
		ST16(&pCurMb.m_mb->m_nonZeroCount[20+uiNoneZeroCountOffset],0);
	}else{
		const uint8_t* kpNoneZeroCountIdx=&g_kuiMbCountScan4Idx[uiSubMbIdx];
		pBlock-=64;
		for(i=0;i<4;i++) {
			uiNoneZeroCount=WelsGetNoneZeroCount_c(pBlock);
			pCurMb.m_mb->m_nonZeroCount[*kpNoneZeroCountIdx++]=uiNoneZeroCount;
			uiNoneZeroCountMbAc+=uiNoneZeroCount;
			pBlock+=16;
		}
		WelsDequantFour4x4_c(pRes,g_kuiDequantCoeff[kiQp]);
		pCurMb.m_mb->uiCbp&=0x0F;
		pCurMb.m_mb->uiCbp|=0x20;
	}
	if(uiNoneZeroCountMbDc>0) {
		WelsDequantIHadamard2x2Dc (aDct2x2,g_kuiDequantCoeff[kiQp][0]);
		if(2!=(pCurMb.m_mb->uiCbp>>4))
			pCurMb.m_mb->uiCbp|=(0x01<<4);
		pRes[0]=aDct2x2[0];
		pRes[16]=aDct2x2[1];
		pRes[32]=aDct2x2[2];
		pRes[48]=aDct2x2[3];
	}
}

void NewH264SVCEncoder::IMbChromaEncode(const MBWindow& pCurMb,SMbCache* pMbCache) {
	const int32_t kiEncStride=m_stride[1];
	const int32_t kiCsStride=m_stride[1];
	int16_t* pCurRS=pMbCache->m_coeffLevel;
	uint8_t* pBestPred=pMbCache->pBestPredIntraChroma;
	uint8_t* pCsCb=pMbCache->m_decMb[1];
	uint8_t* pCsCr=pMbCache->m_decMb[2];
	//cb
	WelsDctFourT4_c(pCurRS,pMbCache->m_encMb[1],kiEncStride,pBestPred,8);
	EncRecUV(pCurMb,pMbCache,pCurRS,1);
	WelsIDctFourT4Rec_c(pCsCb,kiCsStride,pBestPred,8,pCurRS);
	//cr
	WelsDctFourT4_c(pCurRS+64,pMbCache->m_encMb[2],kiEncStride,pBestPred+64,8);
	EncRecUV(pCurMb,pMbCache,pCurRS+64,2);
	WelsIDctFourT4Rec_c(pCsCr,kiCsStride,pBestPred+64,8,pCurRS+64);
}

#define COST_MVD(table,mx,my) (table[mx]+table[my])

static inline void UpdateMeResults(const SMVUnitXY ksBestMv,const uint32_t kiBestSadCost,const uint8_t* pRef,SWelsME* pMe) {
	pMe->sMv=ksBestMv;
	pMe->m_pRefMb=pRef;
	pMe->uiSadCost=kiBestSadCost;
}
static inline void MeEndIntepelSearch(SWelsME* pMe) {
	pMe->sMv.iMvX<<=2;
	pMe->sMv.iMvY<<=2;
	pMe->uiSatdCost=pMe->uiSadCost;
}

// EL mb motion estimate initial point testing
bool MotionEstimateInitialPoint(SWelsME* pMe,int32_t iStride,const SMVUnitXY* kpMvcList,const uint32_t kuiMvcNum,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax) {
	const uint16_t* kpMvdCost=pMe->pMvdCost;
	uint8_t* const kpEncMb=pMe->m_pEncMb;
	int16_t iMvc0,iMvc1;
	int32_t iSadCost;
	int32_t iBestSadCost;
	const uint8_t* pFref2;
	const SMVUnitXY ksMvp=pMe->sMvp;
	SMVUnitXY sMv;
	sMv.iMvX=CLIP3((2+ksMvp.iMvX)>>2,ksMvStartMin.iMvX,ksMvStartMax.iMvX);
	sMv.iMvY=CLIP3((2+ksMvp.iMvY)>>2,ksMvStartMin.iMvY,ksMvStartMax.iMvY);
	const uint8_t* pRefMb=&pMe->m_pRefMb[sMv.iMvY*iStride+sMv.iMvX];
	iBestSadCost=SampleSad_c(pMe->uiBlockSize,kpEncMb,iStride,pRefMb,iStride);
	iBestSadCost+=COST_MVD(kpMvdCost,((sMv.iMvX)*(1<<2))-ksMvp.iMvX,((sMv.iMvY)*(1<<2))-ksMvp.iMvY);
	for(uint32_t i=0;i<kuiMvcNum;i++) {
		iMvc0=CLIP3((2+kpMvcList[i].iMvX)>>2,ksMvStartMin.iMvX,ksMvStartMax.iMvX);
		iMvc1=CLIP3((2+kpMvcList[i].iMvY)>>2,ksMvStartMin.iMvY,ksMvStartMax.iMvY);
		if(((iMvc0-sMv.iMvX) || (iMvc1-sMv.iMvY))) {
			pFref2=&pMe->m_pRefMb[iMvc1*iStride+iMvc0];
			iSadCost=SampleSad_c(pMe->uiBlockSize,kpEncMb,iStride,pFref2,iStride)+COST_MVD (kpMvdCost,(iMvc0*(1<<2))-ksMvp.iMvX,(iMvc1*(1<<2))-ksMvp.iMvY);
			if(iSadCost<iBestSadCost) {
				sMv.iMvX=iMvc0;
				sMv.iMvY=iMvc1;
				pRefMb=pFref2;
				iBestSadCost=iSadCost;
			}
		}
	}
	UpdateMeResults(sMv,iBestSadCost,pRefMb,pMe);
	if(iBestSadCost<(int32_t)(pMe->uiSadPred)) {
		MeEndIntepelSearch(pMe);
		return true;
	}
	return false;
}

// Diamond Search Basics
inline bool MeSadCostSelect(int32_t* iSadCost,const uint16_t* kpMvdCost,int32_t* pBestCost,const int32_t kiDx,const int32_t kiDy,int32_t* pIx,int32_t* pIy) {
	int32_t iTempSadCost[4];
	int32_t iInputSadCost=*pBestCost;
	iTempSadCost[0]=iSadCost[0]+COST_MVD(kpMvdCost,kiDx,kiDy-4);
	iTempSadCost[1]=iSadCost[1]+COST_MVD(kpMvdCost,kiDx,kiDy+4);
	iTempSadCost[2]=iSadCost[2]+COST_MVD(kpMvdCost,kiDx-4,kiDy);
	iTempSadCost[3]=iSadCost[3]+COST_MVD(kpMvdCost,kiDx+4,kiDy);
	if(iTempSadCost[0]<*pBestCost) {
		*pBestCost=iTempSadCost[0];
		*pIx=0;
		*pIy=1;
	}
	if(iTempSadCost[1]<*pBestCost) {
		*pBestCost=iTempSadCost[1];
		*pIx=0;
		*pIy=-1;
	}
	if(iTempSadCost[2]<*pBestCost) {
		*pBestCost=iTempSadCost[2];
		*pIx=1;
		*pIy=0;
	}
	if(iTempSadCost[3]<*pBestCost) {
		*pBestCost=iTempSadCost[3];
		*pIx=-1;
		*pIy=0;
	}
	return (*pBestCost==iInputSadCost);
}

void DiamondSearch(SWelsME* pMe,const int32_t kiStride,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax) {
	uint8_t* const kpEncMb=pMe->m_pEncMb;
	const uint16_t* kpMvdCost=pMe->pMvdCost;
	int32_t iMvDx=((pMe->sMv.iMvX)*(1<<2))-pMe->sMvp.iMvX;
	int32_t iMvDy=((pMe->sMv.iMvY)*(1<<2))-pMe->sMvp.iMvY;
	const uint8_t* pRefMb=pMe->m_pRefMb;
	int32_t iBestCost=pMe->uiSadCost;
	int32_t iTimeThreshold=ITERATIVE_TIMES;
	ENFORCE_STACK_ALIGN_1D(int32_t,iSadCosts,4,16)
	while(iTimeThreshold--) {
		pMe->sMv.iMvX=(iMvDx+pMe->sMvp.iMvX)>>2;
		pMe->sMv.iMvY=(iMvDy+pMe->sMvp.iMvY)>>2;
		bool inRange=((pMe->sMv.iMvX>=ksMvStartMin.iMvX) && (pMe->sMv.iMvX<ksMvStartMax.iMvX)) && ((pMe->sMv.iMvY>=ksMvStartMin.iMvY) && (pMe->sMv.iMvY<ksMvStartMax.iMvY));
		if(!inRange)
			continue;
		switch(pMe->uiBlockSize) {
			case BLOCK_16x16:
				SampleSadFour16x16_c(kpEncMb,kiStride,pRefMb,kiStride,&iSadCosts[0]);
				break;
			case BLOCK_8x8:
				SampleSadFour8x8_c(kpEncMb,kiStride,pRefMb,kiStride,&iSadCosts[0]);
				break;
			case BLOCK_16x8:
				SampleSadFour16x8_c(kpEncMb,kiStride,pRefMb,kiStride,&iSadCosts[0]);
				break;
			case BLOCK_8x16:
				SampleSadFour8x16_c(kpEncMb,kiStride,pRefMb,kiStride,&iSadCosts[0]);
				break;
			default:
				FATAL("WTF");
		}
		int32_t iX,iY;
		const bool kbIsBestCostWorse=MeSadCostSelect(iSadCosts,kpMvdCost,&iBestCost,iMvDx,iMvDy,&iX,&iY);
		if(kbIsBestCostWorse)
			break;
		iMvDx-=(iX*(1<<2));
		iMvDy-=(iY*(1<<2));
		pRefMb-=(iX+iY*kiStride);
	}
	// integer-pel mv
	pMe->sMv.iMvX=(iMvDx+pMe->sMvp.iMvX)>>2;
	pMe->sMv.iMvY=(iMvDy+pMe->sMvp.iMvY)>>2;
	pMe->uiSatdCost=pMe->uiSadCost=iBestCost;
	pMe->m_pRefMb=pRefMb;
}

// BL mb motion estimate search
void MotionEstimateSearch(SWelsME* pMe,const SMVUnitXY* kpMvcList,const uint32_t kuiMvcNum,uint32_t stride,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax) {
	StartTimer(1,"MotionSearch");
	if(!MotionEstimateInitialPoint(pMe,stride,kpMvcList,kuiMvcNum,ksMvStartMin,ksMvStartMax)) {
		DiamondSearch(pMe,stride,ksMvStartMin,ksMvStartMax);
		MeEndIntepelSearch(pMe);
	}
	StopTimer(1);
}

void NewH264SVCEncoder::PredInter16x8Mv(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,SMVUnitXY* sMvp) {
	const SMVComponentUnit* kpMvComp=&pMbCache->sMvComponents;
	if(iPartIdx==0) {
		const int8_t kiTopRef=kpMvComp->iRefIndexCache[1];
		if(iRef==kiTopRef) {
			*sMvp=kpMvComp->sMotionVectorCache[1];
			return;
		}
	}else{ // 8==iPartIdx
		const int8_t kiLeftRef=kpMvComp->iRefIndexCache[18];
		if(iRef==kiLeftRef) {
			*sMvp=kpMvComp->sMotionVectorCache[18];
			return;
		}
	}
	PredMv(sMvp,kpMvComp,iPartIdx,4,iRef);
}

//update uiRefIndex and pMv of only Mb_cache,only for P16x8
void NewH264SVCEncoder::UpdateP16x8Motion2Cache(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,const SMVUnitXY& pMv) {
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	for(int32_t i=0;i<2;i++,iPartIdx+=4) {
		const uint8_t kuiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		pMvComp->iRefIndexCache[kuiCacheIdx]=pMvComp->iRefIndexCache[1+kuiCacheIdx]=pMvComp->iRefIndexCache[6+kuiCacheIdx]=pMvComp->iRefIndexCache[7+kuiCacheIdx]=iRef;
		pMvComp->sMotionVectorCache[kuiCacheIdx]=pMvComp->sMotionVectorCache[1+kuiCacheIdx]=pMvComp->sMotionVectorCache[6+kuiCacheIdx]=pMvComp->sMotionVectorCache[7+kuiCacheIdx]=pMv;
	}
}

int32_t NewH264SVCEncoder::MdP16x8(SMbCache* pMbCache,SWelsMD* pWelsMd,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax) {
	int32_t iCostP16x8=0;
	for(int32_t i=0;i!=2;i++) {
		SWelsME* sMe16x8=&pWelsMd->sMe.sMe16x8[i];
		int32_t iPixelY=i<<3;
		InitMe(sMe16x8,*pWelsMd,BLOCK_16x8,pMbCache->m_encMb[0]+(iPixelY*m_stride[0]),pMbCache->m_refMb[0]+(iPixelY*m_stride[0]));
		//not putting the lines below into InitMe to avoid judging mode in InitMe
		sMe16x8->uiSadPred=pWelsMd->iSadPredMb>>1;
		SMVUnitXY sMv={0,0};
		PredInter16x8Mv(pMbCache,i<<3,0,&sMe16x8->sMvp);
		MotionEstimateSearch(sMe16x8,&sMv,1,m_stride[0],ksMvStartMin,ksMvStartMax);
		UpdateP16x8Motion2Cache(pMbCache,i<<3,pWelsMd->m_uiRef,sMe16x8->sMv);
		iCostP16x8+=sMe16x8->uiSatdCost;
	}
	return iCostP16x8;
}

//update uiRefIndex and pMv of only Mb_cache,only for P8x16
void NewH264SVCEncoder::UpdateP8x16Motion2Cache(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,const SMVUnitXY& pMv) {
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	for(int32_t i=0;i<2;i++,iPartIdx+=8) {
		const uint8_t kuiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		pMvComp->iRefIndexCache[kuiCacheIdx]=pMvComp->iRefIndexCache[1+kuiCacheIdx]=pMvComp->iRefIndexCache[6+kuiCacheIdx]=pMvComp->iRefIndexCache[7+kuiCacheIdx]=iRef;
		pMvComp->sMotionVectorCache[kuiCacheIdx]=pMvComp->sMotionVectorCache[1+kuiCacheIdx]=pMvComp->sMotionVectorCache[6+kuiCacheIdx]=pMvComp->sMotionVectorCache[7+kuiCacheIdx]=pMv;
	}
}

//update uiRefIndex and pMv of only Mb_cache,only for P8x8
void NewH264SVCEncoder::UpdateP8x8Motion2Cache(SMbCache* pMbCache,int32_t iPartIdx,int8_t pRef,const SMVUnitXY& pMv) {
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	const uint8_t kuiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
	pMvComp->iRefIndexCache[kuiCacheIdx]=pMvComp->iRefIndexCache[1+kuiCacheIdx]=pMvComp->iRefIndexCache[6+kuiCacheIdx]=pMvComp->iRefIndexCache[7+kuiCacheIdx]=pRef;
	pMvComp->sMotionVectorCache[kuiCacheIdx]=pMvComp->sMotionVectorCache[1+kuiCacheIdx]=pMvComp->sMotionVectorCache[6+kuiCacheIdx]=pMvComp->sMotionVectorCache[7+kuiCacheIdx]=pMv;
}

int32_t NewH264SVCEncoder::MdP8x8(SMbCache* pMbCache,SWelsMD* pWelsMd,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax) {
	int32_t iLineSizeEnc=m_stride[0];
	int32_t iLineSizeRef=m_stride[0];
	SWelsME* sMe8x8;
	int32_t i,iIdxX,iIdxY,iPixelX,iPixelY,iStrideEnc,iStrideRef;
	int32_t iCostP8x8=0;
	for(i=0;i<4;i++) {
		iIdxX=i&1;
		iIdxY=i>>1;
		iPixelX=(iIdxX<<3);
		iPixelY=(iIdxY<<3);
		iStrideEnc=iPixelX+(iPixelY*iLineSizeEnc);
		iStrideRef=iPixelX+(iPixelY*iLineSizeRef);
		sMe8x8=&pWelsMd->sMe.sMe8x8[i];
		InitMe(sMe8x8,*pWelsMd,BLOCK_8x8,pMbCache->m_encMb[0]+iStrideEnc,pMbCache->m_refMb[0]+iStrideRef);
		//not putting these three lines below into InitMe to avoid judging mode in InitMe
		sMe8x8->uiSadPred=pWelsMd->iSadPredMb>>2;
		SMVUnitXY sMv={0,0};
		PredMv(&sMe8x8->sMvp,&pMbCache->sMvComponents,i<<2,2,pWelsMd->m_uiRef);
		MotionEstimateSearch(sMe8x8,&sMv,1,m_stride[0],ksMvStartMin,ksMvStartMax);
		UpdateP8x8Motion2Cache(pMbCache,i<<2,pWelsMd->m_uiRef,sMe8x8->sMv);
		iCostP8x8+=sMe8x8->uiSatdCost;
	}
	return iCostP8x8;
}

void NewH264SVCEncoder::PredInter8x16Mv(SMbCache* pMbCache,int32_t iPartIdx,int8_t iRef,SMVUnitXY* sMvp) {
	const SMVComponentUnit* kpMvComp=&pMbCache->sMvComponents;
	if(iPartIdx==0) {
		const int8_t kiLeftRef=kpMvComp->iRefIndexCache[6];
		if(iRef==kiLeftRef) {
			*sMvp=kpMvComp->sMotionVectorCache[6];
			return;
		}
	}else{ // 1==iPartIdx
		int8_t iDiagonalRef=kpMvComp->iRefIndexCache[5];//top-right
		int8_t iIndex=5;
		if(REF_NOT_AVAIL==iDiagonalRef) {
			iDiagonalRef=kpMvComp->iRefIndexCache[2];//top-left for 8*8 block(iIndex 1)
			iIndex=2;
		}
		if(iRef==iDiagonalRef) {
			*sMvp=kpMvComp->sMotionVectorCache[iIndex];
			return;
		}
	}
	PredMv(sMvp,kpMvComp,iPartIdx,2,iRef);
}


int32_t NewH264SVCEncoder::MdP8x16(SMbCache* pMbCache,SWelsMD* pWelsMd,SMVUnitXY ksMvStartMin,SMVUnitXY ksMvStartMax) {
	int32_t iCostP8x16=0;
	for(int32_t i=0;i!=2;i++) {
		int32_t iPixelX=i<<3;
		SWelsME* sMe8x16=&pWelsMd->sMe.sMe8x16[i];
		InitMe(sMe8x16,*pWelsMd,BLOCK_8x16,pMbCache->m_encMb[0]+iPixelX,pMbCache->m_refMb[0]+iPixelX);
		//not putting the lines below into InitMe to avoid judging mode in InitMe
		sMe8x16->uiSadPred=pWelsMd->iSadPredMb>>1;
		SMVUnitXY sMv={0,0};
		PredInter8x16Mv(pMbCache,i<<2,0,&sMe8x16->sMvp);
		MotionEstimateSearch(sMe8x16,&sMv,1,m_stride[0],ksMvStartMin,ksMvStartMax);
		UpdateP8x16Motion2Cache(pMbCache,i<<2,pWelsMd->m_uiRef,sMe8x16->sMv);
		iCostP8x16+=sMe8x16->uiSatdCost;
	}
	return iCostP8x16;
}

#define INTPEL_NEEDED_MARGIN 3

void NewH264SVCEncoder::MdInterFinePartitionVaa(SMbCache* pMbCache,const SVAAFrameInfo& vaa,SWelsMD* pWelsMd,MBWindow* windowMb,int32_t iBestCost,int32_t iMvRange) {
	int32_t iCostP8x16,iCostP16x8,iCostP8x8;
	uint8_t uiMbSign=MdInterAnalysisVaaInfo_c(&vaa.m_sad8x8[windowMb->iMbXY*4]);
	if(uiMbSign==15) {
		return;
	}
	SMVUnitXY sMvStartMin,sMvStartMax;
	sMvStartMin.iMvX=MAX(-((windowMb->iMbX+1)<<4)+INTPEL_NEEDED_MARGIN,-iMvRange);
	sMvStartMin.iMvY=MAX(-((windowMb->iMbY+1)<<4)+INTPEL_NEEDED_MARGIN,-iMvRange);
	sMvStartMax.iMvX=MIN(((m_blockWidth-windowMb->iMbX)<<4)-INTPEL_NEEDED_MARGIN,iMvRange);
	sMvStartMax.iMvY=MIN(((m_blockHeight-windowMb->iMbY)<<4)-INTPEL_NEEDED_MARGIN,iMvRange);
	switch(uiMbSign) {
		case 3:
		case 12:
			iCostP16x8=MdP16x8(pMbCache,pWelsMd,sMvStartMin,sMvStartMax);
			if(iCostP16x8<iBestCost) {
				iBestCost=iCostP16x8;
				windowMb->m_mb->uiMbType=MB_TYPE_16x8;
			}
			break;
		case 5:
		case 10:
			iCostP8x16=MdP8x16(pMbCache,pWelsMd,sMvStartMin,sMvStartMax);
			if(iCostP8x16<iBestCost) {
				iBestCost=iCostP8x16;
				windowMb->m_mb->uiMbType=MB_TYPE_8x16;
			}
			break;
		case 6:
		case 9:
			iCostP8x8=MdP8x8(pMbCache,pWelsMd,sMvStartMin,sMvStartMax);
			if(iCostP8x8<iBestCost) {
				iBestCost=iCostP8x8;
				windowMb->m_mb->uiMbType=MB_TYPE_8x8;
			}
			break;
		default:
			iCostP8x8=MdP8x8(pMbCache,pWelsMd,sMvStartMin,sMvStartMax);
			if(iCostP8x8<iBestCost) {
				iBestCost=iCostP8x8;
				windowMb->m_mb->uiMbType=MB_TYPE_8x8;
				iCostP16x8=MdP16x8(pMbCache,pWelsMd,sMvStartMin,sMvStartMax);
				if(iCostP16x8 <=iBestCost) {
					iBestCost=iCostP16x8;
					windowMb->m_mb->uiMbType=MB_TYPE_16x8;
				}
				iCostP8x16=MdP8x16(pMbCache,pWelsMd,sMvStartMin,sMvStartMax);
				if(iCostP8x16 <=iBestCost) {
					iBestCost=iCostP8x16;
					windowMb->m_mb->uiMbType=MB_TYPE_8x16;
				}
			}
			break;
	}
	pWelsMd->iCostLuma=iBestCost;
}

bool NewH264SVCEncoder::TryPYskip(MBWindow* pCurMb,SMbCache* pMbCache) {
	int32_t iSingleCtrMb=0;
	int16_t* pRes=pMbCache->m_coeffLevel;
	const uint8_t kuiQp=pCurMb->m_mb->uiLumaQp;
	int16_t* pBlock=pMbCache->m_dct.iLumaBlock[0];
	uint16_t aMax[4],i,j;
	const int16_t* pMF=g_kiQuantMF[kuiQp];
	const int16_t* pFF=g_kiQuantInterFF[kuiQp];
	for(i=0;i<4;i++) {
		WelsQuantFour4x4Max_c(pRes,pFF,pMF,(int16_t*)aMax);
		for(j=0;j<4;j++) {
			if(aMax[j]>1)
				return false;// iSingleCtrMb+=9,can't be P_SKIP
			else
			if(aMax[j]==1) {
				WelsScan4x4DcAc_c(pBlock,pRes);
				iSingleCtrMb+=WelsCalculateSingleCtr4x4_c(pBlock);
			}
			if(iSingleCtrMb >=6)
				return false;//from JVT-O079
			pRes+=16;
			pBlock+=16;
		}
	}
	return true;
}

bool NewH264SVCEncoder::TryPUVskip(MBWindow* pCurMb,SMbCache* pMbCache,int32_t iUV) {
	int16_t* pRes=((iUV==1) ?&(pMbCache->m_coeffLevel[256]) :&(pMbCache->m_coeffLevel[256+64]));
	const uint8_t kuiQp=g_kuiChromaQpTable[CLIP3_QP_0_51(pCurMb->m_mb->uiLumaQp)];
	const int16_t* pMF=g_kiQuantMF[kuiQp];
	const int16_t* pFF=g_kiQuantInterFF[kuiQp];
	if(WelsHadamardQuant2x2Skip_c(pRes,pFF[0]<<1,pMF[0]>>1))
		return false;
	uint16_t aMax[4],j;
	int32_t iSingleCtrMb=0;
	int16_t* pBlock=pMbCache->m_dct.iChromaBlock[ (iUV-1)<<2];
	WelsQuantFour4x4Max_c(pRes,pFF,pMF,(int16_t*)aMax);
	for(j=0;j<4;j++) {
		if(aMax[j]>1)
			return false;  // iSingleCtrMb+=9,can't be P_SKIP
		else
		if(aMax[j]==1) {
			WelsScan4x4Ac_c(pBlock,pRes);
			iSingleCtrMb+=WelsCalculateSingleCtr4x4_c(pBlock);
		}
		if(iSingleCtrMb >=7)
			return false;//from JVT-O079
		pRes+=16;
		pBlock+=16;
	}
	return true;
}

bool NewH264SVCEncoder::MdPSkipEnc(SWelsMD* pWelsMd,MBWindow* windowMb,SMbCache* pMbCache,const SPicture* pRefPic,int32_t iSadPredSkip) {
	uint8_t* pRefLuma=pMbCache->m_refMb[0];
	uint8_t* pRefCb=pMbCache->m_refMb[1];
	uint8_t* pRefCr=pMbCache->m_refMb[2];
	int32_t iLineSizeY=m_stride[0];
	int32_t iLineSizeUV=m_stride[1];
	uint8_t* pDstLuma=pMbCache->m_skipMb;
	uint8_t* pDstCb=pMbCache->m_skipMb+256;
	uint8_t* pDstCr=pMbCache->m_skipMb+256+64;
	SMVUnitXY sMvp={0};
	int32_t n;
	int32_t iEncStride=m_stride[0];
	uint8_t* pEncMb=pMbCache->m_encMb[0];
	int32_t* pEncBlockOffset;
	int32_t iSadCostLuma=0;
	int32_t iSadCostChroma=0;
	int32_t iSadCostMb=0;
	PredSkipMv(*pMbCache,&sMvp);
	// Special case,need to clip the vector //
	SMVUnitXY sQpelMvp={(int16_t)(sMvp.iMvX>>2),(int16_t)(sMvp.iMvY>>2) };
	n=(windowMb->iMbX<<4)+sQpelMvp.iMvX;
	if(n<-29)
		return false;
	else
	if(n>(int32_t)((m_blockWidth<<4)+12))
		return false;

	n=(windowMb->iMbY<<4)+sQpelMvp.iMvY;
	if(n<-29)
		return false;
	else
	if(n>(int32_t)((m_blockHeight<<4)+12))
		return false;
	//luma
	pRefLuma+=sQpelMvp.iMvY*iLineSizeY+sQpelMvp.iMvX;
	McLuma_c(pRefLuma,iLineSizeY,pDstLuma,16,sMvp.iMvX,sMvp.iMvY,16,16);
	iSadCostLuma=SampleSad16x16_c(pMbCache->m_encMb[0],m_stride[0],pDstLuma,16);
	const int32_t iStrideUV=(sQpelMvp.iMvY>>1)*iLineSizeUV+(sQpelMvp.iMvX>>1);
	pRefCb+=iStrideUV;
	McChroma_c(pRefCb,iLineSizeUV,pDstCb,8,sMvp.iMvX,sMvp.iMvY,8,8);//Cb
	iSadCostChroma=SampleSad8x8_c(pMbCache->m_encMb[1],m_stride[1],pDstCb,8);
	pRefCr+=iStrideUV;
	McChroma_c(pRefCr,iLineSizeUV,pDstCr,8,sMvp.iMvX,sMvp.iMvY,8,8);//Cr
	iSadCostChroma+=SampleSad8x8_c(pMbCache->m_encMb[2],m_stride[2],pDstCr,8);
	iSadCostMb=iSadCostLuma+iSadCostChroma;
	if(iSadCostMb==0 || iSadCostMb<iSadPredSkip || (pRefPic->m_type==P_SLICE && pMbCache->m_uiRefMbType==MB_TYPE_SKIP && iSadCostMb<pRefPic->pMbSkipSad[windowMb->iMbXY])) {
		//update motion info to current MB
		ST32(windowMb->m_mb->m_refIndex,0);
		UpdateMbMv_c(windowMb->m_mb->m_mv,sMvp);
		windowMb->m_mb->m_sadCost=iSadCostLuma;
		pWelsMd->iCostLuma=windowMb->m_mb->m_sadCost;
		pWelsMd->iCostSkipMb=iSadCostMb;
		windowMb->m_mb->sP16x16Mv=sMvp;
		return true;
	}
	DctMb(pMbCache->m_coeffLevel,pEncMb,iEncStride,pDstLuma);
	if(TryPYskip(windowMb,pMbCache)) {
		int32_t* pStrideEncBlockOffset=m_strideTable.m_pStrideEncBlockOffset;
		iEncStride=m_stride[1];
		pEncMb=pMbCache->m_encMb[1];
		pEncBlockOffset=pStrideEncBlockOffset+16;
		WelsDctFourT4_c(pMbCache->m_coeffLevel+256,& (pEncMb[*pEncBlockOffset]),iEncStride,pMbCache->m_skipMb+256,8);
		if(TryPUVskip(windowMb,pMbCache,1)) {
			pEncMb=pMbCache->m_encMb[2];
			pEncBlockOffset=pStrideEncBlockOffset+20;
			WelsDctFourT4_c(pMbCache->m_coeffLevel+320,& (pEncMb[*pEncBlockOffset]),iEncStride,pMbCache->m_skipMb+320,8);
			if(TryPUVskip(windowMb,pMbCache,2)) {
				//update motion info to current MB
				ST32(windowMb->m_mb->m_refIndex,0);
				UpdateMbMv_c(windowMb->m_mb->m_mv,sMvp);
				windowMb->m_mb->m_sadCost=iSadCostLuma;
				pWelsMd->iCostLuma=windowMb->m_mb->m_sadCost;
				pWelsMd->iCostSkipMb=iSadCostMb;
				windowMb->m_mb->sP16x16Mv=sMvp;
				return true;
			}
		}
	}
	return false;
}

int32_t NewH264SVCEncoder::PredictSadSkip(const int8_t* pRefIndexCache,const bool* pMbSkipCache,const int32_t* pSadCostCache) {
	const int32_t kiRefB=pRefIndexCache[1];
	int32_t iRefC=pRefIndexCache[5];
	const int32_t kiRefA=pRefIndexCache[6];
	const int32_t kiSadB=(pMbSkipCache[1]==1 ? pSadCostCache[1] : 0);
	int32_t iSadC=(pMbSkipCache[2]==1 ? pSadCostCache[2] : 0);
	const int32_t kiSadA=(pMbSkipCache[3]==1 ? pSadCostCache[3] : 0);
	int32_t iRefSkip=pMbSkipCache[2];
	int32_t iCount=0;
	if(iRefC==REF_NOT_AVAIL) {
		iRefC=pRefIndexCache[0];
		iSadC=(pMbSkipCache[0]==1 ? pSadCostCache[0] : 0);
		iRefSkip=pMbSkipCache[0];
	}
	int32_t iSadPredSkip=kiSadA;
	if(!(kiRefB==REF_NOT_AVAIL && iRefC==REF_NOT_AVAIL && kiRefA!=REF_NOT_AVAIL)) {
		iCount=((!kiRefA) && (pMbSkipCache[3]==1))<<MB_LEFT_BIT;
		iCount|=((!kiRefB) && (pMbSkipCache[1]==1))<<MB_TOP_BIT;
		iCount|=((!iRefC) && (iRefSkip==1))<<MB_TOPRIGHT_BIT;
		switch(iCount) {
			case LEFT_MB_POS:// A
				iSadPredSkip=kiSadA;
				break;
			case TOP_MB_POS:// B
				iSadPredSkip=kiSadB;
				break;
			case TOPRIGHT_MB_POS:// C or D
				iSadPredSkip=iSadC;
				break;
			default:
				iSadPredSkip=Median(kiSadA,kiSadB,iSadC);
				break;
		}
	}
	return iSadPredSkip;
}

int32_t NewH264SVCEncoder::PredictSad(const int8_t* pRefIndexCache,const int32_t* pSadCostCache) {
	const int32_t kiRefB=pRefIndexCache[1];					// top g_uiCache12_8x8RefIdx[0]-4
	int32_t iRefC=pRefIndexCache[5];						// top-right g_uiCache12_8x8RefIdx[0]-2
	const int32_t kiRefA=pRefIndexCache[6];					// left g_uiCache12_8x8RefIdx[0]-1
	const int32_t kiSadB=pSadCostCache[1];
	int32_t iSadC=pSadCostCache[2];
	const int32_t kiSadA=pSadCostCache[3];
	if(iRefC==REF_NOT_AVAIL) {
		iRefC=pRefIndexCache[0];							// top-left g_uiCache12_8x8RefIdx[0]-4-1
		iSadC=pSadCostCache[0];
	}
	int32_t pSadPred=kiSadA;
	if(!(kiRefB==REF_NOT_AVAIL && iRefC==REF_NOT_AVAIL && kiRefA!=REF_NOT_AVAIL)) {
		int32_t iCount=(0==kiRefA)<<MB_LEFT_BIT;
		iCount|=(0==kiRefB)<<MB_TOP_BIT;
		iCount|=(0==iRefC)<<MB_TOPRIGHT_BIT;
		switch(iCount) {
			case LEFT_MB_POS:// A
				pSadPred=kiSadA;
				break;
			case TOP_MB_POS:// B
				pSadPred=kiSadB;
				break;
			case TOPRIGHT_MB_POS:// C or D
				pSadPred=iSadC;
				break;
			default:
				pSadPred=Median(kiSadA,kiSadB,iSadC);
				break;
		}
	}
#define REPLACE_SAD_MULTIPLY(x)   ((x)-(x>>3)+(x >>5))				// it's 0.90625,very close to 0.9
	int32_t iCount=(pSadPred)<<6;									// here *64 will not overflow. SAD range 0~ 255*256(max 2^16),int32_t is enough
	pSadPred=(REPLACE_SAD_MULTIPLY(iCount)+32)>>6;
#undef REPLACE_SAD_MULTIPLY
	return pSadPred;
}

int32_t NewH264SVCEncoder::MdP16x16(SMbCache* pMbCache,SWelsMD* pWelsMd,const MBWindow& mbWindow,const SPicture* pRefPic,int32_t iMvRange) {
	SMVUnitXY sMvStartMin,sMvStartMax;
	sMvStartMin.iMvX=MAX(-((mbWindow.iMbX+1)<<4)+INTPEL_NEEDED_MARGIN,-iMvRange);
	sMvStartMin.iMvY=MAX(-((mbWindow.iMbY+1)<<4)+INTPEL_NEEDED_MARGIN,-iMvRange);
	sMvStartMax.iMvX=MIN(((m_blockWidth-mbWindow.iMbX)<<4)-INTPEL_NEEDED_MARGIN,iMvRange);
	sMvStartMax.iMvY=MIN(((m_blockHeight-mbWindow.iMbY)<<4)-INTPEL_NEEDED_MARGIN,iMvRange);
	SWelsME* pMe16x16=&pWelsMd->sMe.sMe16x16;
	InitMe(pMe16x16,*pWelsMd,BLOCK_16x16,pMbCache->m_encMb[0],pMbCache->m_refMb[0]);
	//not putting the line below into InitMe to avoid judging mode in InitMe
	pMe16x16->uiSadPred=pWelsMd->iSadPredMb;
	SMVUnitXY sMvc[5];
	uint8_t uiMvcNum=0;
	sMvc[uiMvcNum++]={0,0};
	//spatial motion vector predictors
	if(mbWindow.HasNeightbor(N_LEFT)) {						//left available
		sMvc[uiMvcNum++]=mbWindow.GetNeighbor(N_LEFT)->sP16x16Mv;
	}
	if(mbWindow.HasNeightbor(N_TOP)) {						//top available
		sMvc[uiMvcNum++]=mbWindow.GetNeighbor(N_TOP)->sP16x16Mv;
	}
	//temporal motion vector predictors
	if(pRefPic->m_type==P_SLICE) {
		if(mbWindow.iMbX<m_blockWidth-1) {
			sMvc[uiMvcNum++]=pRefPic->sMvList[mbWindow.iMbXY+1];
		}
		if(mbWindow.iMbY<m_blockHeight-1) {
			sMvc[uiMvcNum++]=pRefPic->sMvList[mbWindow.iMbXY+m_blockWidth];
		}
	}
	PredMv(&pMe16x16->sMvp,&pMbCache->sMvComponents,0,4,0);
	MotionEstimateSearch(pMe16x16,&sMvc[0],uiMvcNum,m_stride[0],sMvStartMin,sMvStartMax);
	mbWindow.m_mb->sP16x16Mv=pMe16x16->sMv;
	return pMe16x16->uiSatdCost;
}

#define ME_REFINE_BUF_STRIDE       32
#define ME_REFINE_BUF_WIDTH_BLK8   16
#define ME_REFINE_BUF_STRIDE_BLK8  320

#define REFINE_ME_NO_BEST_HALF_PIXEL 0 //( 0,0)
#define REFINE_ME_HALF_PIXEL_LEFT    3 //(-2,0)
#define REFINE_ME_HALF_PIXEL_RIGHT   4 //( 2,0)
#define REFINE_ME_HALF_PIXEL_TOP     1 //( 0,-2)
#define REFINE_ME_HALF_PIXEL_BOTTOM  2 //( 0,2)

#define ME_NO_BEST_QUAR_PIXEL 1 //( 0,0) or best half pixel
#define ME_QUAR_PIXEL_LEFT    2 //(-1,0)
#define ME_QUAR_PIXEL_RIGHT   3 //( 1,0)
#define ME_QUAR_PIXEL_TOP     4 //( 0,-1)
#define ME_QUAR_PIXEL_BOTTOM  5 //( 0,1)

#define NO_BEST_FRAC_PIX   1 // REFINE_ME_NO_BEST_HALF_PIXEL+ME_NO_BEST_QUAR_PIXEL

const int32_t g_kiPixStrideIdx8x8[4]={0,ME_REFINE_BUF_WIDTH_BLK8,ME_REFINE_BUF_STRIDE_BLK8,ME_REFINE_BUF_STRIDE_BLK8+ME_REFINE_BUF_WIDTH_BLK8};

#define SWITCH_BEST_TMP_BUF(prev_best,curr_best){\
	pParams->iBestCost=iCurCost;\
	pTmp=prev_best;\
	prev_best=curr_best;\
	curr_best=pTmp;\
}

#define CALC_COST(me_buf,lm) (SampleSatd_c(kuiPixel,pEncMb,iStrideEnc,me_buf,ME_REFINE_BUF_STRIDE)+lm)

struct SQuarRefineParams {
	int32_t iBestCost;
	int32_t iStrideA;
	int32_t iStrideB;
	const uint8_t* pSrcB[4];
	const uint8_t* pSrcA[4];
	int32_t iLms[4];
	int32_t iBestQuarPix;
};

void MeRefineQuarPixel(SWelsME* pMe,SMeRefinePointer* pMeRefine,const int32_t kiWidth,const int32_t kiHeight,SQuarRefineParams* pParams,int32_t iStrideEnc) {
	int32_t iCurCost;
	uint8_t* pEncMb=pMe->m_pEncMb;
	uint8_t* pTmp=NULL;
	const uint8_t kuiPixel=pMe->uiBlockSize;
	PixelAvg_c(pMeRefine->pQuarPixTmp,ME_REFINE_BUF_STRIDE,pParams->pSrcA[0],ME_REFINE_BUF_STRIDE,pParams->pSrcB[0],pParams->iStrideA,kiWidth,kiHeight);
	iCurCost=CALC_COST(pMeRefine->pQuarPixTmp,pParams->iLms[0]);
	if(iCurCost<pParams->iBestCost) {
		pParams->iBestQuarPix=ME_QUAR_PIXEL_TOP;
		SWITCH_BEST_TMP_BUF(pMeRefine->pQuarPixBest,pMeRefine->pQuarPixTmp);
	}
	//=========================(0,1)=======================//
	PixelAvg_c(pMeRefine->pQuarPixTmp,ME_REFINE_BUF_STRIDE,pParams->pSrcA[1],ME_REFINE_BUF_STRIDE,pParams->pSrcB[1],pParams->iStrideA,kiWidth,kiHeight);
	iCurCost=CALC_COST (pMeRefine->pQuarPixTmp,pParams->iLms[1]);
	if(iCurCost<pParams->iBestCost) {
		pParams->iBestQuarPix=ME_QUAR_PIXEL_BOTTOM;
		SWITCH_BEST_TMP_BUF(pMeRefine->pQuarPixBest,pMeRefine->pQuarPixTmp);
	}
	//==========================(-1,0)=========================//
	PixelAvg_c(pMeRefine->pQuarPixTmp,ME_REFINE_BUF_STRIDE,pParams->pSrcA[2],ME_REFINE_BUF_STRIDE,pParams->pSrcB[2],pParams->iStrideB,kiWidth,kiHeight);
	iCurCost=CALC_COST (pMeRefine->pQuarPixTmp,pParams->iLms[2]);
	if(iCurCost<pParams->iBestCost) {
		pParams->iBestQuarPix=ME_QUAR_PIXEL_LEFT;
		SWITCH_BEST_TMP_BUF(pMeRefine->pQuarPixBest,pMeRefine->pQuarPixTmp);
	}
	//==========================(1,0)=========================//
	PixelAvg_c(pMeRefine->pQuarPixTmp,ME_REFINE_BUF_STRIDE,pParams->pSrcA[3],ME_REFINE_BUF_STRIDE,pParams->pSrcB[3],pParams->iStrideB,kiWidth,kiHeight);
	iCurCost=CALC_COST (pMeRefine->pQuarPixTmp,pParams->iLms[3]);
	if(iCurCost<pParams->iBestCost) {
		pParams->iBestQuarPix=ME_QUAR_PIXEL_RIGHT;
		SWITCH_BEST_TMP_BUF(pMeRefine->pQuarPixBest,pMeRefine->pQuarPixTmp);
	}
}

void NewH264SVCEncoder::MeRefineFracPixel(uint8_t* pMemPredInterMb,SWelsME* pMe,SMeRefinePointer* pMeRefine,int32_t iWidth,int32_t iHeight) {
	StartTimer(3,"Refine");
	int16_t iMvx=pMe->sMv.iMvX;
	int16_t iMvy=pMe->sMv.iMvY;
	int16_t iHalfMvx=iMvx;
	int16_t iHalfMvy=iMvy;
	const int32_t kiStrideEnc=m_stride[0];
	const int32_t kiStrideRef=m_stride[0];
	uint8_t* pEncData=pMe->m_pEncMb;
	const uint8_t* pRef=pMe->m_pRefMb;
	int32_t iBestQuarPix=ME_NO_BEST_QUAR_PIXEL;
	SQuarRefineParams sParams;
	static const int32_t iMvQuarAddX[10]={0,0,-1,1,0,0,0,-1,1,0};
	const int32_t* pMvQuarAddY=iMvQuarAddX+3;
	const uint8_t* pBestPredInter=pRef;
	int32_t iInterBlk4Stride=ME_REFINE_BUF_STRIDE;
	int32_t iBestCost=SampleSatd_c(pMe->uiBlockSize,pEncData,kiStrideEnc,pRef,kiStrideRef)+	COST_MVD (pMe->pMvdCost,iMvx-pMe->sMvp.iMvX,iMvy-pMe->sMvp.iMvY);
	int32_t iBestHalfPix=REFINE_ME_NO_BEST_HALF_PIXEL;
	McHorVer02_c(pRef-kiStrideRef,kiStrideRef,pMeRefine->pHalfPixV,ME_REFINE_BUF_STRIDE,iWidth,iHeight+1);
	//step 1: get [iWidth][iHeight+1] half pixel from vertical filter
	//===========================(0,-2)==============================//
	int32_t iCurCost=SampleSatd_c(pMe->uiBlockSize,pEncData,kiStrideEnc,pMeRefine->pHalfPixV,ME_REFINE_BUF_STRIDE)+COST_MVD (pMe->pMvdCost,iMvx-pMe->sMvp.iMvX,iMvy-2-pMe->sMvp.iMvY);
	if(iCurCost<iBestCost) {
		iBestCost=iCurCost;
		iBestHalfPix=REFINE_ME_HALF_PIXEL_TOP;
		pBestPredInter=pMeRefine->pHalfPixV;
	}
	//===========================(0,2)==============================//
	iCurCost=SampleSatd_c(pMe->uiBlockSize,pEncData,kiStrideEnc,pMeRefine->pHalfPixV+ME_REFINE_BUF_STRIDE,ME_REFINE_BUF_STRIDE)+COST_MVD (pMe->pMvdCost,iMvx-pMe->sMvp.iMvX,iMvy+2-pMe->sMvp.iMvY);
	if(iCurCost<iBestCost) {
		iBestCost=iCurCost;
		iBestHalfPix=REFINE_ME_HALF_PIXEL_BOTTOM;
		pBestPredInter=pMeRefine->pHalfPixV+ME_REFINE_BUF_STRIDE;
	}
	McHorVer20_c(pRef-1,kiStrideRef,pMeRefine->pHalfPixH,ME_REFINE_BUF_STRIDE,iWidth+1,iHeight);
	//step 2: get [iWidth][iHeight+1] half pixel from horizon filter
	//===========================(-2,0)==============================//
	iCurCost=SampleSatd_c(pMe->uiBlockSize,pEncData,kiStrideEnc,pMeRefine->pHalfPixH,ME_REFINE_BUF_STRIDE)+COST_MVD (pMe->pMvdCost,iMvx-2-pMe->sMvp.iMvX,iMvy-pMe->sMvp.iMvY);
	if(iCurCost<iBestCost) {
		iBestCost=iCurCost;
		iBestHalfPix=REFINE_ME_HALF_PIXEL_LEFT;
		pBestPredInter=pMeRefine->pHalfPixH;
	}
	//===========================(2,0)===============================//
	iCurCost=SampleSatd_c(pMe->uiBlockSize,pEncData,kiStrideEnc,pMeRefine->pHalfPixH+1,ME_REFINE_BUF_STRIDE)+COST_MVD (pMe->pMvdCost,iMvx+2-pMe->sMvp.iMvX,iMvy-pMe->sMvp.iMvY);
	if(iCurCost<iBestCost) {
		iBestCost=iCurCost;
		iBestHalfPix=REFINE_ME_HALF_PIXEL_RIGHT;
		pBestPredInter=pMeRefine->pHalfPixH+1;
	}
	sParams.iBestCost=iBestCost;
	sParams.iBestQuarPix=ME_NO_BEST_QUAR_PIXEL;
	//step 5: if no best half-pixel prediction,try quarter pixel prediction
	//        if yes,must get [X+1][X+1] half-pixel from (2,2) horizontal and vertical filter
	if(REFINE_ME_NO_BEST_HALF_PIXEL==iBestHalfPix) {
		sParams.iStrideA=kiStrideRef;
		sParams.iStrideB=kiStrideRef;
		sParams.pSrcA[0]=pMeRefine->pHalfPixV;
		sParams.pSrcA[1]=pMeRefine->pHalfPixV+ME_REFINE_BUF_STRIDE;
		sParams.pSrcA[2]=pMeRefine->pHalfPixH;
		sParams.pSrcA[3]=pMeRefine->pHalfPixH+1;
		sParams.pSrcB[0]=sParams.pSrcB[1]=sParams.pSrcB[2]=sParams.pSrcB[3]=pRef;
		sParams.iLms[0]=COST_MVD (pMe->pMvdCost,iHalfMvx-pMe->sMvp.iMvX,iHalfMvy-1-pMe->sMvp.iMvY);
		sParams.iLms[1]=COST_MVD (pMe->pMvdCost,iHalfMvx-pMe->sMvp.iMvX,iHalfMvy+1-pMe->sMvp.iMvY);
		sParams.iLms[2]=COST_MVD (pMe->pMvdCost,iHalfMvx-1-pMe->sMvp.iMvX,iHalfMvy-pMe->sMvp.iMvY);
		sParams.iLms[3]=COST_MVD (pMe->pMvdCost,iHalfMvx+1-pMe->sMvp.iMvX,iHalfMvy-pMe->sMvp.iMvY);
	}else{ //must get [X+1][X+1] half-pixel from (2,2) horizontal and vertical filter
		switch(iBestHalfPix) {
			case REFINE_ME_HALF_PIXEL_LEFT: {
				pMeRefine->pHalfPixHV=pMeRefine->pHalfPixV;//reuse pBuffer,here only h&hv
				McHorVer22_c(pRef-1-kiStrideRef,kiStrideRef,pMeRefine->pHalfPixHV,ME_REFINE_BUF_STRIDE,iWidth+1,iHeight+1);
				iHalfMvx-=2;
				sParams.iStrideA=ME_REFINE_BUF_STRIDE;
				sParams.iStrideB=kiStrideRef;
				sParams.pSrcA[0]=pMeRefine->pHalfPixH;
				sParams.pSrcA[3]=sParams.pSrcA[2]=sParams.pSrcA[1]=sParams.pSrcA[0];
				sParams.pSrcB[0]=pMeRefine->pHalfPixHV;
				sParams.pSrcB[1]=pMeRefine->pHalfPixHV+ME_REFINE_BUF_STRIDE;
				sParams.pSrcB[2]=pRef-1;
				sParams.pSrcB[3]=pRef;
				break;
			}
			case REFINE_ME_HALF_PIXEL_RIGHT: {
				pMeRefine->pHalfPixHV=pMeRefine->pHalfPixV;//reuse pBuffer,here only h&hv
				McHorVer22_c(pRef-1-kiStrideRef,kiStrideRef,pMeRefine->pHalfPixHV,ME_REFINE_BUF_STRIDE,iWidth+1,iHeight+1);
				iHalfMvx+=2;
				sParams.iStrideA=ME_REFINE_BUF_STRIDE;
				sParams.iStrideB=kiStrideRef;
				sParams.pSrcA[0]=pMeRefine->pHalfPixH+1;
				sParams.pSrcA[3]=sParams.pSrcA[2]=sParams.pSrcA[1]=sParams.pSrcA[0];
				sParams.pSrcB[0]=pMeRefine->pHalfPixHV+1;
				sParams.pSrcB[1]=pMeRefine->pHalfPixHV+1+ME_REFINE_BUF_STRIDE;
				sParams.pSrcB[2]=pRef;
				sParams.pSrcB[3]=pRef+1;
				break;
			}
			case REFINE_ME_HALF_PIXEL_TOP: {
				pMeRefine->pHalfPixHV=pMeRefine->pHalfPixH;//reuse pBuffer,here only v&hv
				McHorVer22_c(pRef-1-kiStrideRef,kiStrideRef,pMeRefine->pHalfPixHV,ME_REFINE_BUF_STRIDE,iWidth+1,iHeight+1);
				iHalfMvy-=2;
				sParams.iStrideA=kiStrideRef;
				sParams.iStrideB=ME_REFINE_BUF_STRIDE;
				sParams.pSrcA[0]=pMeRefine->pHalfPixV;
				sParams.pSrcA[3]=sParams.pSrcA[2]=sParams.pSrcA[1]=sParams.pSrcA[0];
				sParams.pSrcB[0]=pRef-kiStrideRef;
				sParams.pSrcB[1]=pRef;
				sParams.pSrcB[2]=pMeRefine->pHalfPixHV;
				sParams.pSrcB[3]=pMeRefine->pHalfPixHV+1;
				break;
			}
			case REFINE_ME_HALF_PIXEL_BOTTOM: {
				pMeRefine->pHalfPixHV=pMeRefine->pHalfPixH;//reuse pBuffer,here only v&hv
				McHorVer22_c(pRef-1-kiStrideRef,kiStrideRef,pMeRefine->pHalfPixHV,ME_REFINE_BUF_STRIDE,iWidth+1,iHeight+1);
				iHalfMvy+=2;
				sParams.iStrideA=kiStrideRef;
				sParams.iStrideB=ME_REFINE_BUF_STRIDE;
				sParams.pSrcA[0]=pMeRefine->pHalfPixV+ME_REFINE_BUF_STRIDE;
				sParams.pSrcA[3]=sParams.pSrcA[2]=sParams.pSrcA[1]=sParams.pSrcA[0];
				sParams.pSrcB[0]=pRef;
				sParams.pSrcB[1]=pRef+kiStrideRef;
				sParams.pSrcB[2]=pMeRefine->pHalfPixHV+ME_REFINE_BUF_STRIDE;
				sParams.pSrcB[3]=pMeRefine->pHalfPixHV+ME_REFINE_BUF_STRIDE+1;
				break;
			}
			default:
				break;
		}
		sParams.iLms[0]=COST_MVD(pMe->pMvdCost,iHalfMvx-pMe->sMvp.iMvX,iHalfMvy-1-pMe->sMvp.iMvY);
		sParams.iLms[1]=COST_MVD(pMe->pMvdCost,iHalfMvx-pMe->sMvp.iMvX,iHalfMvy+1-pMe->sMvp.iMvY);
		sParams.iLms[2]=COST_MVD(pMe->pMvdCost,iHalfMvx-1-pMe->sMvp.iMvX,iHalfMvy-pMe->sMvp.iMvY);
		sParams.iLms[3]=COST_MVD(pMe->pMvdCost,iHalfMvx+1-pMe->sMvp.iMvX,iHalfMvy-pMe->sMvp.iMvY);
	}
	MeRefineQuarPixel(pMe,pMeRefine,iWidth,iHeight,&sParams,kiStrideEnc);
	if(iBestCost>sParams.iBestCost) {
		pBestPredInter=pMeRefine->pQuarPixBest;
		iBestCost=sParams.iBestCost;
	}
	iBestQuarPix=sParams.iBestQuarPix;
	//update final best MV
	pMe->sMv.iMvX=iHalfMvx+iMvQuarAddX[iBestQuarPix];
	pMe->sMv.iMvY=iHalfMvy+pMvQuarAddY[iBestQuarPix];
	pMe->uiSatdCost=iBestCost;
	//No half or quarter pixel best,so do MC with integer pixel MV
	if(iBestHalfPix+iBestQuarPix==NO_BEST_FRAC_PIX) {
		pBestPredInter=pRef;
		iInterBlk4Stride=kiStrideRef;
	}
	pMeRefine->pfCopyBlockByMode(pMemPredInterMb,MB_WIDTH_LUMA,pBestPredInter,iInterBlk4Stride);
	StopTimer(3);
}

#define BUTTERFLY4x8(dw) (((uint64_t)(dw)<<32) | (dw))
#define BUTTERFLY1x2(b) (((b)<<8) | (b))
#define BUTTERFLY2x4(wd) (((uint32_t)(wd)<<16) |(wd))

void NewH264SVCEncoder::UpdateP8x8MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int32_t kiPartIdx,const int8_t kiRef,SMVUnitXY* pMv) {
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	const uint32_t kuiMv32=LD32(pMv);
	const uint64_t kuiMv64=BUTTERFLY4x8 (kuiMv32);
	const int16_t kiScan4Idx=g_kuiMbCountScan4Idx[kiPartIdx];
	const int16_t kiCacheIdx=g_kuiCache30ScanIdx[kiPartIdx];
	const int16_t kiCacheIdx1=1+kiCacheIdx;
	const int16_t kiCacheIdx6=6+kiCacheIdx;
	const int16_t kiCacheIdx7=7+kiCacheIdx;
	ST64(&pCurMb->m_mb->m_mv[kiScan4Idx],kuiMv64);
	ST64(&pCurMb->m_mb->m_mv[4+kiScan4Idx],kuiMv64);
	pMvComp->iRefIndexCache[kiCacheIdx]=pMvComp->iRefIndexCache[kiCacheIdx1]=pMvComp->iRefIndexCache[kiCacheIdx6]=pMvComp->iRefIndexCache[kiCacheIdx7]=kiRef;
	pMvComp->sMotionVectorCache[kiCacheIdx]=pMvComp->sMotionVectorCache[kiCacheIdx1]=pMvComp->sMotionVectorCache[kiCacheIdx6]=pMvComp->sMotionVectorCache[kiCacheIdx7]=*pMv;
}

//update pMv and uiRefIndex cache for current MB,only for P_16*16 (SKIP inclusive)
void NewH264SVCEncoder::UpdateP16x16MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int8_t kiRef,SMVUnitXY* pMv) {
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	const uint32_t kuiMv32=LD32(pMv);
	const uint64_t kuiMv64=BUTTERFLY4x8 (kuiMv32);
	uint64_t uiMvBuf[8]={kuiMv64,kuiMv64,kuiMv64,kuiMv64,kuiMv64,kuiMv64,kuiMv64,kuiMv64};
	const uint16_t kuiRef16=BUTTERFLY1x2(kiRef);
	const uint32_t kuiRef32=BUTTERFLY2x4(kuiRef16);
	ST32(pCurMb->m_mb->m_refIndex,kuiRef32);
	// update pMv range from 0~15
	memcpy(pCurMb->m_mb->m_mv,uiMvBuf,sizeof(uiMvBuf));
	//blocks 0: 7~10,1: 13~16,2: 19~22,3: 25~28
	pMvComp->iRefIndexCache[7]=kiRef;
	ST16(&pMvComp->iRefIndexCache[8],kuiRef16);
	pMvComp->iRefIndexCache[10]=kiRef;
	pMvComp->iRefIndexCache[13]=kiRef;
	ST16(&pMvComp->iRefIndexCache[14],kuiRef16);
	pMvComp->iRefIndexCache[16]=kiRef;
	pMvComp->iRefIndexCache[19]=kiRef;
	ST16(&pMvComp->iRefIndexCache[20],kuiRef16);
	pMvComp->iRefIndexCache[22]=kiRef;
	pMvComp->iRefIndexCache[25]=kiRef;
	ST16(&pMvComp->iRefIndexCache[26],kuiRef16);
	pMvComp->iRefIndexCache[28]=kiRef;
	//blocks 0: 7~10,1: 13~16,2: 19~22,3: 25~28
	pMvComp->sMotionVectorCache[7]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[8],kuiMv64);
	pMvComp->sMotionVectorCache[10]=*pMv;
	pMvComp->sMotionVectorCache[13]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[14],kuiMv64);
	pMvComp->sMotionVectorCache[16]=*pMv;
	pMvComp->sMotionVectorCache[19]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[20],kuiMv64);
	pMvComp->sMotionVectorCache[22]=*pMv;
	pMvComp->sMotionVectorCache[25]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[26],kuiMv64);
	pMvComp->sMotionVectorCache[28]=*pMv;
}

//update uiRefIndex and pMv of both SMB and Mb_cache,only for P8x16
void NewH264SVCEncoder::UpdateP8x16MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int32_t kiPartIdx,const int8_t kiRef,SMVUnitXY* pMv) {
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	const uint32_t kuiMv32=LD32(pMv);
	const uint64_t kuiMv64=BUTTERFLY4x8 (kuiMv32);
	const int16_t kiScan4Idx=g_kuiMbCountScan4Idx[kiPartIdx];
	const int16_t kiCacheIdx=g_kuiCache30ScanIdx[kiPartIdx];
	const int16_t kiCacheIdx1=1+kiCacheIdx;
	const int16_t kiCacheIdx3=3+kiCacheIdx;
	const int16_t kiCacheIdx12=12+kiCacheIdx;
	const int16_t kiCacheIdx13=13+kiCacheIdx;
	const int16_t kiCacheIdx15=15+kiCacheIdx;
	const int16_t kiBlkIdx=kiPartIdx>>2;
	const uint16_t kuiRef16=BUTTERFLY1x2(kiRef);
	pCurMb->m_mb->m_refIndex[kiBlkIdx]=kiRef;
	pCurMb->m_mb->m_refIndex[2+kiBlkIdx]=kiRef;
	ST64(&pCurMb->m_mb->m_mv[kiScan4Idx],kuiMv64);
	ST64(&pCurMb->m_mb->m_mv[4+kiScan4Idx],kuiMv64);
	ST64(&pCurMb->m_mb->m_mv[8+kiScan4Idx],kuiMv64);
	ST64(&pCurMb->m_mb->m_mv[12+kiScan4Idx],kuiMv64);
	//blocks 0: g_kuiCache30ScanIdx[iPartIdx]~g_kuiCache30ScanIdx[iPartIdx]+3,1: g_kuiCache30ScanIdx[iPartIdx]+6~g_kuiCache30ScanIdx[iPartIdx]+9
	pMvComp->iRefIndexCache[kiCacheIdx]=kiRef;
	ST16(&pMvComp->iRefIndexCache[kiCacheIdx1],kuiRef16);
	pMvComp->iRefIndexCache[kiCacheIdx3]=kiRef;
	pMvComp->iRefIndexCache[kiCacheIdx12]=kiRef;
	ST16(&pMvComp->iRefIndexCache[kiCacheIdx13],kuiRef16);
	pMvComp->iRefIndexCache[kiCacheIdx15]=kiRef;
	//blocks 0: g_kuiCache30ScanIdx[iPartIdx]~g_kuiCache30ScanIdx[iPartIdx]+3,1: g_kuiCache30ScanIdx[iPartIdx]+6~g_kuiCache30ScanIdx[iPartIdx]+9
	pMvComp->sMotionVectorCache[kiCacheIdx]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[kiCacheIdx1],kuiMv64);
	pMvComp->sMotionVectorCache[kiCacheIdx3]=*pMv;
	pMvComp->sMotionVectorCache[kiCacheIdx12]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[kiCacheIdx13],kuiMv64);
	pMvComp->sMotionVectorCache[kiCacheIdx15]=*pMv;
}

const uint8_t g_kuiSmb4AddrIn256[16]={
  0, 4,  16*4,16*4+4,
  8, 12, 16*4+8,16*4+12,
  16*8,16*8+4,16*12,16*12+4,
  16*8+8,16*8+12,16*12+8,16*12+12
};

//update uiRefIndex and pMv of both SMB and Mb_cache,only for P16x8
void NewH264SVCEncoder::UpdateP16x8MotionInfo(SMbCache* pMbCache,const MBWindow* pCurMb,const int32_t kiPartIdx,const int8_t kiRef,SMVUnitXY* pMv) {
	// optimized 11/25/2011
	SMVComponentUnit* pMvComp=&pMbCache->sMvComponents;
	const uint32_t kuiMv32=LD32(pMv);
	const uint64_t kuiMv64=BUTTERFLY4x8 (kuiMv32);
	uint64_t uiMvBuf[4]={ kuiMv64,kuiMv64,kuiMv64,kuiMv64 };
	const int16_t kiScan4Idx=g_kuiMbCountScan4Idx[kiPartIdx];
	const int16_t kiCacheIdx=g_kuiCache30ScanIdx[kiPartIdx];
	const int16_t kiCacheIdx1=1+kiCacheIdx;
	const int16_t kiCacheIdx3=3+kiCacheIdx;
	const int16_t kiCacheIdx6=6+kiCacheIdx;
	const int16_t kiCacheIdx7=7+kiCacheIdx;
	const int16_t kiCacheIdx9=9+kiCacheIdx;
	const uint16_t kuiRef16=BUTTERFLY1x2 (kiRef);

	ST16(&pCurMb->m_mb->m_refIndex[ (kiPartIdx>>2)],kuiRef16);
	memcpy (&pCurMb->m_mb->m_mv[kiScan4Idx],uiMvBuf,sizeof(uiMvBuf));

	//blocks 0: g_kuiCache30ScanIdx[iPartIdx]~g_kuiCache30ScanIdx[iPartIdx]+3,1: g_kuiCache30ScanIdx[iPartIdx]+6~g_kuiCache30ScanIdx[iPartIdx]+9
	pMvComp->iRefIndexCache[kiCacheIdx]=kiRef;
	ST16(&pMvComp->iRefIndexCache[kiCacheIdx1],kuiRef16);
	pMvComp->iRefIndexCache[kiCacheIdx3]=kiRef;
	pMvComp->iRefIndexCache[kiCacheIdx6]=kiRef;
	ST16(&pMvComp->iRefIndexCache[kiCacheIdx7],kuiRef16);
	pMvComp->iRefIndexCache[kiCacheIdx9]=kiRef;

	//blocks 0: g_kuiCache30ScanIdx[iPartIdx]~g_kuiCache30ScanIdx[iPartIdx]+3,1: g_kuiCache30ScanIdx[iPartIdx]+6~g_kuiCache30ScanIdx[iPartIdx]+9
	pMvComp->sMotionVectorCache[kiCacheIdx]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[kiCacheIdx1],kuiMv64);
	pMvComp->sMotionVectorCache[kiCacheIdx3]=*pMv;
	pMvComp->sMotionVectorCache[kiCacheIdx6]=*pMv;
	ST64(&pMvComp->sMotionVectorCache[kiCacheIdx7],kuiMv64);
	pMvComp->sMotionVectorCache[kiCacheIdx9]=*pMv;
}

void InitMeRefinePointer(SMeRefinePointer* pMeRefine,uint8_t* bufferInterPredMe,int32_t iStride) {
	pMeRefine->pHalfPixH=&bufferInterPredMe[0]+iStride;
	pMeRefine->pHalfPixV=&bufferInterPredMe[640]+iStride;
	pMeRefine->pQuarPixBest=&bufferInterPredMe[1280]+iStride;
	pMeRefine->pQuarPixTmp=&bufferInterPredMe[1920]+iStride;
}

void NewH264SVCEncoder::MdInterMbRefinement(SWelsMD* pWelsMd,const MBWindow* pCurMb,SMbCache* pMbCache) {
	uint8_t* pTmpRefCb,*pTmpRefCr,*pTmpDstCb,*pTmpDstCr;
	int32_t iMvStride,iRefBlk4Stride,iDstBlk4Stride;
	int32_t iBestSadCost=0,iBestSatdCost=0;
	ENFORCE_STACK_ALIGN_1D(uint8_t,bufferInterPredMe,4*640,16);
	SMeRefinePointer sMeRefine;
	uint8_t* pRefCb=pMbCache->m_refMb[1];
	uint8_t* pRefCr=pMbCache->m_refMb[2];
	uint8_t* pDstCb=pMbCache->pMemPredChroma;
	uint8_t* pDstCr=pMbCache->pMemPredChroma+64;
	uint8_t* pDstLuma=pMbCache->pMemPredLuma;
	int32_t iLineSizeRefUV=m_stride[1];
	switch(pCurMb->m_mb->uiMbType) {
		case MB_TYPE_16x16: {
			//luma
			InitMeRefinePointer(&sMeRefine,bufferInterPredMe,0);
			sMeRefine.pfCopyBlockByMode=Copy16x16_c;
			MeRefineFracPixel(pDstLuma,&pWelsMd->sMe.sMe16x16,&sMeRefine,16,16);
			UpdateP16x16MotionInfo(pMbCache,pCurMb,pWelsMd->m_uiRef,&pWelsMd->sMe.sMe16x16.sMv);
			pMbCache->sMbMvp[0]=pWelsMd->sMe.sMe16x16.sMvp;
			//save the best cost of final mode
			iBestSadCost=pWelsMd->sMe.sMe16x16.uiSadCost;
			iBestSatdCost=pWelsMd->sMe.sMe16x16.uiSatdCost;
			//chroma
			SMVUnitXY* pMv=&pWelsMd->sMe.sMe16x16.sMv;
			iMvStride=(pMv->iMvY>>3)*iLineSizeRefUV+(pMv->iMvX>>3);
			pTmpRefCb=pRefCb+iMvStride;
			pTmpRefCr=pRefCr+iMvStride;
			McChroma_c(pTmpRefCb,iLineSizeRefUV,pDstCb,8,pMv->iMvX,pMv->iMvY,8,8);//Cb
			McChroma_c(pTmpRefCr,iLineSizeRefUV,pDstCr,8,pMv->iMvX,pMv->iMvY,8,8);//Cr
			pWelsMd->iCostSkipMb=SampleSad16x16_c(pMbCache->m_encMb[0],m_stride[0],pDstLuma,16);
			pWelsMd->iCostSkipMb+=SampleSad8x8_c(pMbCache->m_encMb[1],m_stride[1],pDstCb,8);
			pWelsMd->iCostSkipMb+=SampleSad8x8_c(pMbCache->m_encMb[2],m_stride[2],pDstCr,8);
			break;
		}
		case MB_TYPE_16x8: {
			sMeRefine.pfCopyBlockByMode=Copy16x8_c;
			for(int32_t i=0;i<2;i++) {			//NEWDG
				//luma
				int32_t iIdx=i<<3;
				InitMeRefinePointer(&sMeRefine,bufferInterPredMe,i?ME_REFINE_BUF_STRIDE_BLK8:0);
				PredInter16x8Mv(pMbCache,iIdx,pWelsMd->m_uiRef,&pWelsMd->sMe.sMe16x8[i].sMvp);
				MeRefineFracPixel(pDstLuma+g_kuiSmb4AddrIn256[iIdx],&pWelsMd->sMe.sMe16x8[i],&sMeRefine,16,8);
				UpdateP16x8MotionInfo(pMbCache,pCurMb,iIdx,pWelsMd->m_uiRef,&pWelsMd->sMe.sMe16x8[i].sMv);
				pMbCache->sMbMvp[i]=pWelsMd->sMe.sMe16x8[i].sMvp;
				//save the best cost of final mode
				iBestSadCost+=pWelsMd->sMe.sMe16x8[i].uiSadCost;
				iBestSatdCost+=pWelsMd->sMe.sMe16x8[i].uiSatdCost;
				//chroma
				iRefBlk4Stride=(i<<2)*iLineSizeRefUV;
				iDstBlk4Stride=i<<5;// 4*8
				SMVUnitXY* pMv=&pWelsMd->sMe.sMe16x8[i].sMv;
				iMvStride=(pMv->iMvY>>3)*iLineSizeRefUV+(pMv->iMvX>>3);
				pTmpRefCb=pRefCb+iRefBlk4Stride+iMvStride;
				pTmpRefCr=pRefCr+iRefBlk4Stride+iMvStride;
				pTmpDstCb=pDstCb+iDstBlk4Stride;
				pTmpDstCr=pDstCr+iDstBlk4Stride;
				McChroma_c(pTmpRefCb,iLineSizeRefUV,pTmpDstCb,8,pMv->iMvX,pMv->iMvY,8,4);//Cb
				McChroma_c(pTmpRefCr,iLineSizeRefUV,pTmpDstCr,8,pMv->iMvX,pMv->iMvY,8,4);//Cr
			}
			break;
		}
		case MB_TYPE_8x16: {
			sMeRefine.pfCopyBlockByMode=Copy8x16_c;
			for(int32_t i=0;i<2;i++) {
				//luma
				int32_t iIdx=i<<2;
				InitMeRefinePointer(&sMeRefine,bufferInterPredMe,i?ME_REFINE_BUF_WIDTH_BLK8:0);
				PredInter8x16Mv(pMbCache,iIdx,pWelsMd->m_uiRef,&pWelsMd->sMe.sMe8x16[i].sMvp);
				MeRefineFracPixel(pDstLuma+g_kuiSmb4AddrIn256[iIdx],&pWelsMd->sMe.sMe8x16[i],&sMeRefine,8,16);
				UpdateP8x16MotionInfo(pMbCache,pCurMb,iIdx,pWelsMd->m_uiRef,&pWelsMd->sMe.sMe8x16[i].sMv);
				pMbCache->sMbMvp[i]=pWelsMd->sMe.sMe8x16[i].sMvp;
				//save the best cost of final mode
				iBestSadCost+=pWelsMd->sMe.sMe8x16[i].uiSadCost;
				iBestSatdCost+=pWelsMd->sMe.sMe8x16[i].uiSatdCost;
				//chroma
				iRefBlk4Stride=iIdx;//4
				SMVUnitXY* pMv=&pWelsMd->sMe.sMe8x16[i].sMv;
				iMvStride=(pMv->iMvY>>3)*iLineSizeRefUV+(pMv->iMvX>>3);
				pTmpRefCb=pRefCb+iRefBlk4Stride+iMvStride;
				pTmpRefCr=pRefCr+iRefBlk4Stride+iMvStride;
				pTmpDstCb=pDstCb+iRefBlk4Stride;
				pTmpDstCr=pDstCr+iRefBlk4Stride;
				McChroma_c(pTmpRefCb,iLineSizeRefUV,pTmpDstCb,8,pMv->iMvX,pMv->iMvY,4,8);//Cb
				McChroma_c(pTmpRefCr,iLineSizeRefUV,pTmpDstCr,8,pMv->iMvX,pMv->iMvY,4,8);//Cr
			}
			break;
		}
		case MB_TYPE_8x8: {
			pMbCache->sMvComponents.iRefIndexCache[9]=pMbCache->sMvComponents.iRefIndexCache [21]=REF_NOT_AVAIL;
			sMeRefine.pfCopyBlockByMode=Copy8x8_c;
			for(int32_t i=0;i<4;i++) {
				int32_t iBlk8Idx=i<<2;//0,4,8,12
				int32_t iBlk4X,iBlk4Y;
				pCurMb->m_mb->m_refIndex[i]=pWelsMd->m_uiRef;
				//luma
				InitMeRefinePointer(&sMeRefine,bufferInterPredMe,g_kiPixStrideIdx8x8[i]);
				PredMv(&pWelsMd->sMe.sMe8x8[i].sMvp,&pMbCache->sMvComponents,iBlk8Idx,2,pWelsMd->m_uiRef);
				MeRefineFracPixel(pDstLuma+g_kuiSmb4AddrIn256[iBlk8Idx],&pWelsMd->sMe.sMe8x8[i],&sMeRefine,8,8);
				UpdateP8x8MotionInfo(pMbCache,pCurMb,iBlk8Idx,pWelsMd->m_uiRef,&pWelsMd->sMe.sMe8x8[i].sMv);
				pMbCache->sMbMvp[g_kuiMbCountScan4Idx[iBlk8Idx]]=pWelsMd->sMe.sMe8x8[i].sMvp;
				iBestSadCost+=pWelsMd->sMe.sMe8x8[i].uiSadCost;
				iBestSatdCost+=pWelsMd->sMe.sMe8x8[i].uiSatdCost;
				//chroma
				SMVUnitXY* pMv=&pWelsMd->sMe.sMe8x8[i].sMv;
				iMvStride=(pMv->iMvY>>3)*iLineSizeRefUV+(pMv->iMvX>>3);
				iBlk4X=(i&1)<<2;
				iBlk4Y=(i>>1)<<2;
				iRefBlk4Stride=iBlk4Y*iLineSizeRefUV+iBlk4X;
				iDstBlk4Stride=(iBlk4Y<<3)+iBlk4X;
				pTmpRefCb=pRefCb+iRefBlk4Stride;
				pTmpDstCb=pDstCb+iDstBlk4Stride;
				pTmpRefCr=pRefCr+iRefBlk4Stride;
				pTmpDstCr=pDstCr+iDstBlk4Stride;
				McChroma_c(pTmpRefCb+iMvStride,iLineSizeRefUV,pTmpDstCb,8,pMv->iMvX,pMv->iMvY,4,4);//Cb
				McChroma_c(pTmpRefCr+iMvStride,iLineSizeRefUV,pTmpDstCr,8,pMv->iMvX,pMv->iMvY,4,4);//Cr
			}
			break;
		}
		default:
			break;
	}
	pCurMb->m_mb->m_sadCost=iBestSadCost;
	pWelsMd->iCostLuma=iBestSadCost;
}

void NewH264SVCEncoder::EncInterY(MBWindow* pCurMb,SMbCache* pMbCache) {
	int16_t* pRes=pMbCache->m_coeffLevel;
	int32_t iSingleCtrMb=0,iSingleCtr8x8[4];
	int16_t* pBlock=pMbCache->m_dct.iLumaBlock[0];
	uint8_t uiQp=pCurMb->m_mb->uiLumaQp;
	const int16_t* pMF=g_kiQuantMF[uiQp];
	const int16_t* pFF=g_kiQuantInterFF[uiQp];
	int16_t aMax[16];
	int32_t i,j,iNoneZeroCountMbDcAc=0,iNoneZeroCount=0;
	for(i=0;i<4;i++) {
		WelsQuantFour4x4Max_c(pRes,pFF,pMF,aMax+(i<<2));
		iSingleCtr8x8[i]=0;
		for(j=0;j<4;j++) {
			if(aMax[ (i<<2)+j]==0)
				memset(pBlock,0,32);
			else{
				WelsScan4x4DcAc_c(pBlock,pRes);
				if(aMax[ (i<<2)+j]>1)
					iSingleCtr8x8[i]+=9;
				else
				if(iSingleCtr8x8[i]<6)
					iSingleCtr8x8[i]+=WelsCalculateSingleCtr4x4_c(pBlock);
			}
			pRes+=16;
			pBlock+=16;
		}
		iSingleCtrMb+=iSingleCtr8x8[i];
	}
	pBlock-=256;
	pRes-=256;
	memset(pCurMb->m_mb->m_nonZeroCount,0,16);
	if(iSingleCtrMb<6) {								//from JVT-O079
		iNoneZeroCountMbDcAc=0;
		memset(pRes,0,768);
	}else{
		const uint8_t* kpNoneZeroCountIdx=g_kuiMbCountScan4Idx;
		for(i=0;i<4;i++) {
			if(iSingleCtr8x8[i] >=4) {
				for(j=0;j<4;j++) {
					iNoneZeroCount=WelsGetNoneZeroCount_c(pBlock);
					pCurMb->m_mb->m_nonZeroCount[*kpNoneZeroCountIdx++]=iNoneZeroCount;
					iNoneZeroCountMbDcAc+=iNoneZeroCount;
					pBlock+=16;
				}
				WelsDequantFour4x4_c(pRes,g_kuiDequantCoeff[uiQp]);
				pCurMb->m_mb->uiCbp|=1<<i;
			}else{										// set zero for an 8x8 pBlock
				memset(pRes,0,128);
				kpNoneZeroCountIdx+=4;
				pBlock+=64;
			}
			pRes+=64;
		}
	}
}

void NewH264SVCEncoder::InterMbEncode(SMbCache* pMbCache,MBWindow* pCurMb) {
	DctMb(pMbCache->m_coeffLevel,pMbCache->m_encMb[0],m_stride[0],pMbCache->pMemPredLuma);
	EncInterY(pCurMb,pMbCache);
}

void NewH264SVCEncoder::PMbChromaEncode(SMbCache* pMbCache,MBWindow* pCurMb) {
	const int32_t kiEncStride=m_stride[1];
	int16_t* pCurRS=pMbCache->m_coeffLevel+256;
	uint8_t* pBestPred=pMbCache->pMemPredChroma;
	WelsDctFourT4_c(pCurRS,pMbCache->m_encMb[1],kiEncStride,pBestPred,8);
	WelsDctFourT4_c(pCurRS+64,pMbCache->m_encMb[2],kiEncStride,pBestPred+64,8);
	EncRecUV(*pCurMb,pMbCache,pCurRS,1);
	EncRecUV(*pCurMb,pMbCache,pCurRS+64,2);
}

//  doublecheck if current MBTYPE is Pskip
void NewH264SVCEncoder::MdInterDoubleCheckPskip(MBWindow* pCurMb,SMbCache* pMbCache) {
	if(pCurMb->m_mb->uiMbType==MB_TYPE_16x16 && !pCurMb->m_mb->uiCbp) {
		if(!pCurMb->m_mb->m_refIndex[0]) {
			SMVUnitXY sMvp={ 0 };
			PredSkipMv(*pMbCache,&sMvp);
			if(LD32(&sMvp)==LD32(&pCurMb->m_mb->m_mv[0])) {
				pCurMb->m_mb->uiMbType=MB_TYPE_SKIP;
			}
		}
		pMbCache->bCollocatedPredFlag=(LD32(&pCurMb->m_mb->m_mv[0])==0);
	}
}

//fill cache of neighbor MB,containing pNonZeroCount,sample_avail,pIntra4x4PredMode
void NewH264SVCEncoder::FillNeighborCacheIntra(SMbCache* pMbCache,const MBWindow& mbWindow) {
	if(mbWindow.HasNeightbor(N_LEFT)) { //LEFT MB
		const int8_t* pLeftMbNonZeroCount=mbWindow.GetNeighbor(N_LEFT)->m_nonZeroCount;
		pMbCache->iNonZeroCoeffCount[8]=pLeftMbNonZeroCount[ 3];
		pMbCache->iNonZeroCoeffCount[16]=pLeftMbNonZeroCount[ 7];
		pMbCache->iNonZeroCoeffCount[24]=pLeftMbNonZeroCount[11];
		pMbCache->iNonZeroCoeffCount[32]=pLeftMbNonZeroCount[15];
		pMbCache->iNonZeroCoeffCount[ 13]=pLeftMbNonZeroCount[17];
		pMbCache->iNonZeroCoeffCount[21]=pLeftMbNonZeroCount[21];
		pMbCache->iNonZeroCoeffCount[37]=pLeftMbNonZeroCount[19];
		pMbCache->iNonZeroCoeffCount[45]=pLeftMbNonZeroCount[23];
		pMbCache->iIntraPredMode[8]=pMbCache->iIntraPredMode[16]=pMbCache->iIntraPredMode[24]=pMbCache->iIntraPredMode[32]=2;//DC
	}else{
		pMbCache->iNonZeroCoeffCount[ 8]=pMbCache->iNonZeroCoeffCount[16]=pMbCache->iNonZeroCoeffCount[24]=pMbCache->iNonZeroCoeffCount[32]=-1;
		pMbCache->iNonZeroCoeffCount[13]=pMbCache->iNonZeroCoeffCount[21]=pMbCache->iNonZeroCoeffCount[37]=pMbCache->iNonZeroCoeffCount[45]=-1;
		pMbCache->iIntraPredMode[8]=pMbCache->iIntraPredMode[16]=pMbCache->iIntraPredMode[24]=pMbCache->iIntraPredMode[32]=-1;
	}
	if(mbWindow.HasNeightbor(N_TOP)) { //TOP MB
		const MacroBlock* pTopMb=mbWindow.GetNeighbor(N_TOP);
		ST32(&pMbCache->iNonZeroCoeffCount[1],LD32(&pTopMb->m_nonZeroCount[12]));
		ST16(&pMbCache->iNonZeroCoeffCount[6],LD16(&pTopMb->m_nonZeroCount[20]));
		ST16(&pMbCache->iNonZeroCoeffCount[30],LD16(&pTopMb->m_nonZeroCount[22]));
		ST32(pMbCache->iIntraPredMode+1,0x02020202);
	}else{
		ST32(pMbCache->iIntraPredMode+1,0xffffffff);
		ST32(&pMbCache->iNonZeroCoeffCount[1],0xffffffff);
		ST16(&pMbCache->iNonZeroCoeffCount[6],0xffff);
		ST16(&pMbCache->iNonZeroCoeffCount[30],0xffff);
	}
}

void NewH264SVCEncoder::MdIntraInit(SMbCache* pMbCache,const SPicture* pEncPic,const SPicture* pDecPic,const MBWindow& mbWindow) {
	const int32_t kiMbX=mbWindow.iMbX;
	const int32_t kiMbY=mbWindow.iMbY;
	int32_t iOffsetY=(kiMbX+kiMbY*m_stride[0])<<4;
	int32_t iOffsetUV=(kiMbX+kiMbY*m_stride[1])<<3;
	pMbCache->m_encMb[0]=pEncPic->pData[0]+iOffsetY;
	pMbCache->m_encMb[1]=pEncPic->pData[1]+iOffsetUV;
	pMbCache->m_encMb[2]=pEncPic->pData[2]+iOffsetUV;
	pMbCache->m_decMb[0]=pDecPic->pData[0]+iOffsetY;
	pMbCache->m_decMb[1]=pDecPic->pData[1]+iOffsetUV;
	pMbCache->m_decMb[2]=pDecPic->pData[2]+iOffsetUV;
	mbWindow.m_mb->uiCbp=0;
	FillNeighborCacheIntra(pMbCache,mbWindow);
	pMbCache->pMemPredLuma=pMbCache->m_memPredMb;
	pMbCache->pMemPredChroma=pMbCache->m_memPredMb+256;
}

void NewH264SVCEncoder::MdInterInit(SMbCache* pMbCache,const SVAAFrameInfo& vaa,SPicture* pDecPic,const MBWindow& mbWindow,const SPicture* pRefPic,int32_t iMvRange) {
	const int32_t kiMbX=mbWindow.iMbX;
	const int32_t kiMbY=mbWindow.iMbY;
	const int32_t kiMbXY=mbWindow.iMbXY;
	pMbCache->pEncSad=&pDecPic->pMbSkipSad[kiMbXY];
	FillNeighborCacheInterWithBGD(pMbCache,mbWindow,m_blockWidth,vaa.m_vaaBackgroundMbFlag+kiMbXY);//BGD spatial pFunc
	const int32_t kiCurStrideY=(kiMbX+kiMbY*m_stride[0])<<4;
	const int32_t kiCurStrideUV=(kiMbX+kiMbY*m_stride[1])<<3;
	pMbCache->m_refMb[0]=pRefPic->pData[0]+kiCurStrideY;
	pMbCache->m_refMb[1]=pRefPic->pData[1]+kiCurStrideUV;
	pMbCache->m_refMb[2]=pRefPic->pData[2]+kiCurStrideUV;
	pMbCache->m_uiRefMbType=pRefPic->m_uiRefMbType[kiMbXY];
	pMbCache->bCollocatedPredFlag=false;
	ST32(&mbWindow.m_mb->sP16x16Mv,0);
	ST32(&pDecPic->sMvList[kiMbXY],0);
}

void NewH264SVCEncoder::UpdateNonZeroCountCache(const MBWindow& pMb,SMbCache* pMbCache) {
	ST32(&pMbCache->iNonZeroCoeffCount[9],LD32(&pMb.m_mb->m_nonZeroCount[0]));
	ST32(&pMbCache->iNonZeroCoeffCount[17],LD32(&pMb.m_mb->m_nonZeroCount[4]));
	ST32(&pMbCache->iNonZeroCoeffCount[25],LD32(&pMb.m_mb->m_nonZeroCount[8]));
	ST32(&pMbCache->iNonZeroCoeffCount[33],LD32(&pMb.m_mb->m_nonZeroCount[12]));
	ST16(&pMbCache->iNonZeroCoeffCount[14],LD16(&pMb.m_mb->m_nonZeroCount[16]));
	ST16(&pMbCache->iNonZeroCoeffCount[38],LD16(&pMb.m_mb->m_nonZeroCount[18]));
	ST16(&pMbCache->iNonZeroCoeffCount[22],LD16(&pMb.m_mb->m_nonZeroCount[20]));
	ST16(&pMbCache->iNonZeroCoeffCount[46],LD16(&pMb.m_mb->m_nonZeroCount[22]));
}

void NewH264SVCEncoder::OutputPMbWithoutConstructCsRsNoCopy(SMbCache* pMbCache,const MBWindow& mb) {
	if(IS_INTER(mb.m_mb->uiMbType) && !IS_SKIP(mb.m_mb->uiMbType)) {
		uint8_t* pDecY=pMbCache->m_decMb[0];
		uint8_t* pDecU=pMbCache->m_decMb[1];
		uint8_t* pDecV=pMbCache->m_decMb[2];
		int16_t* pScaledTcoeff=pMbCache->m_coeffLevel;
		const int32_t kiDecStrideLuma=m_stride[0];
		const int32_t kiDecStrideChroma=m_stride[1];
		WelsIDctT4RecOnMb(pDecY,kiDecStrideLuma,pDecY,kiDecStrideLuma,pScaledTcoeff);
		WelsIDctFourT4Rec_c(pDecU,kiDecStrideChroma,pDecU,kiDecStrideChroma,pScaledTcoeff+256);
		WelsIDctFourT4Rec_c(pDecV,kiDecStrideChroma,pDecV,kiDecStrideChroma,pScaledTcoeff+320);
	}
}

void NewH264SVCEncoder::CopyMb(SPicture* pDstPic,const uint8_t* pY,const uint8_t* pU,const uint8_t* pV,int32_t sY,int32_t sUV,const MBWindow& mb) {
	int32_t iOffsetY=(mb.iMbX+mb.iMbY*m_stride[0])<<4;
	int32_t iOffsetUV=(mb.iMbX+mb.iMbY*m_stride[1])<<3;
	Copy16x16_c(pDstPic->pData[0]+iOffsetY,m_stride[0],pY,sY);
	Copy8x8_c(pDstPic->pData[1]+iOffsetUV,m_stride[1],pU,sUV);
	Copy8x8_c(pDstPic->pData[2]+iOffsetUV,m_stride[1],pV,sUV);
}

void NewH264SVCEncoder::CopyMb(SPicture* pDstPic,const SPicture* pSrcPic,const MBWindow& mb) {
	int32_t iOffsetY=(mb.iMbX+mb.iMbY*m_stride[0])<<4;
	int32_t iOffsetUV=(mb.iMbX+mb.iMbY*m_stride[1])<<3;
	Copy16x16_c(pDstPic->pData[0]+iOffsetY,m_stride[0],pSrcPic->pData[0]+iOffsetY,m_stride[0]);									// Copy prev orig frame macroblock pixels into current frame macroblock
	Copy8x8_c(pDstPic->pData[1]+iOffsetUV,m_stride[1],pSrcPic->pData[1]+iOffsetUV,m_stride[1]);
	Copy8x8_c(pDstPic->pData[2]+iOffsetUV,m_stride[1],pSrcPic->pData[2]+iOffsetUV,m_stride[1]);
}


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

void NewH264SVCEncoder::EncodePFrame(const SVAAFrameInfo& vaa,SPicture* pEncPic,SPicture* pDecPic,const SPicture* pRefPic,int32_t numberMbGom,uint32_t frameComplexity) {
	SBitStringAux bitStringAux;
	uint8_t* pBsBuffer=(uint8_t*)Mallocz(m_frameBsSize);
	InitBits(&bitStringAux,pBsBuffer,m_frameBsSize);
	int32_t iMinFrameQp;
	int32_t iMaxFrameQp;
	int32_t iTargetBits;
	int32_t iGlobalQp=InitPFrame(&iMinFrameQp,&iMaxFrameQp,&iTargetBits,frameComplexity);
	int32_t iStartPos=BsGetBitsPos(&bitStringAux)>>3;
	SliceHeaderWrite(&bitStringAux,iGlobalQp,pEncPic->m_type);
	//ClearPicture(pDecPic,true,true);
	START_TIMER(epInitTimer,&m_profiler,"InitBlocks",0xff00ff);
	SRCSlicing sSOverRc;
	memset(&sSOverRc,0,sizeof(sSOverRc));
	int32_t iBitsPerMb=DIV_ROUND64((int64_t)iTargetBits*INT_MULTIPLY,(m_blockWidth*m_blockHeight));
	sSOverRc.iTargetBitsSlice=DIV_ROUND64((int64_t)iBitsPerMb*(m_blockWidth*m_blockHeight),INT_MULTIPLY);
	sSOverRc.iCalculatedQpSlice=iGlobalQp;
	const SLevelLimits* pLevelLimit=&g_ksLevelLimits[countof(g_ksLevelLimits)-1];
	for(int i=0;i<countof(g_ksLevelLimits);i++) {
		if(CheckLevelLimitation(&g_ksLevelLimits[i],m_iTargetBitrate,m_blockWidth,m_blockHeight)) {
			pLevelLimit=&g_ksLevelLimits[i];
			break;
		}
	}
	int32_t iMvRange=MIN(MIN(ABS((pLevelLimit->iMinVmv)>>2),(pLevelLimit->iMaxVmv)>>2),CAMERA_STARTMV_RANGE);
	int32_t iMvdRange1=MIN((iMvRange+1)<<1,CAMERA_MVD_RANGE);
	const uint32_t kuiMvdInterTableSize=iMvdRange1<<2;									// intepel*4=qpel
	const uint32_t kuiMvdInterTableStride=1+(kuiMvdInterTableSize<<1);					// qpel_mv_range*2=(+/-);
	uint16_t* pMvdCostTableBase=(uint16_t*)Mallocz(52*kuiMvdInterTableStride*sizeof(uint16_t));
	uint16_t* pMvdCostTable=&pMvdCostTableBase[kuiMvdInterTableSize];
	MvdCostInit(pMvdCostTableBase,kuiMvdInterTableStride);
	const int32_t blocksAlloc=m_blockWidth*3;
	MacroBlock* macroBlocks=(MacroBlock*)Mallocz(blocksAlloc*sizeof(MacroBlock));
	END_TIMER(epInitTimer,&m_profiler);
	START_TIMER(blockTimer,&m_profiler,"Block",0x70ff20);
	START_TIMER(initTimer,&m_profiler,"Init",0x0000ff);
	END_TIMER(initTimer,&m_profiler);
	START_TIMER(testTimer,&m_profiler,"Begin",0x00ff00);
	END_TIMER(testTimer,&m_profiler);
	START_TIMER(predictTimer,&m_profiler,"Predict",0x80ff80);
	END_TIMER(predictTimer,&m_profiler);
	START_TIMER(intraTimer,&m_profiler,"Intra",0xff0020);
	END_TIMER(intraTimer,&m_profiler);
	START_TIMER(interTimer,&m_profiler,"Inter",0xff0000);
	START_TIMER(encodeTimer,&m_profiler,"Encode",0x00ffff);
	END_TIMER(encodeTimer,&m_profiler);
	END_TIMER(interTimer,&m_profiler);
	START_TIMER(updateTimer,&m_profiler,"Update",0x7f60ff);
	END_TIMER(updateTimer,&m_profiler);
	END_TIMER(blockTimer,&m_profiler);
	START_TIMER(deblockTimer,&m_profiler,"Deblock",0x2fff80);
	END_TIMER(deblockTimer,&m_profiler);
	uint8_t uiLastMbQp=iGlobalQp;
	int32_t iTotalMbSlice=0;
	int32_t iTotalQpSlice=0;
	SCabacCtxEnc sCabacCtx;
	BsAlign(&bitStringAux);
	sCabacCtx.InitSliceCabac(bitStringAux,iGlobalQp,pEncPic->m_type);
	for(int32_t y=0;y!=m_blockHeight;y++) {
		for(int32_t x=0;x!=m_blockWidth;x++) {
			m_profiler.StartSubTime(blockTimer);
			m_profiler.StartSubTime(initTimer);
			StartTimer(0,"Macroblock");
			MBWindow mb;
			mb.InitFull(x,y,m_blockWidth,m_blockHeight,macroBlocks);
			RcMbInitGom(&mb,&sSOverRc,numberMbGom,iMinFrameQp,iMaxFrameQp);
			SMbCache mbCache;
			memset(&mbCache,0,sizeof(mbCache));
			MdIntraInit(&mbCache,pEncPic,pDecPic,mb);
			MdInterInit(&mbCache,vaa,pDecPic,mb,pRefPic,iMvRange);
			SWelsMD sMd;
			memset(&sMd,0,sizeof(sMd));
			sMd.m_iLambda=g_kiQpCostTable[mb.m_mb->uiLumaQp];
			sMd.m_pMvdCost=&pMvdCostTable[mb.m_mb->uiLumaQp*kuiMvdInterTableStride];
			m_profiler.StopSubTime(initTimer);
			uint8_t uiChmaI8x8Mode=0;
			bool bMbLeftAvailPskip=mb.HasNeightbor(N_LEFT) ? IS_SKIP(mb.GetNeighbor(N_LEFT)->uiMbType) : false;
			bool bMbTopAvailPskip=mb.HasNeightbor(N_TOP) ? IS_SKIP(mb.GetNeighbor(N_TOP)->uiMbType) : false;
			bool bMbTopRightAvailPskip=mb.HasNeightbor(N_TOPRIGHT) ? IS_SKIP(mb.GetNeighbor(N_TOPRIGHT)->uiMbType) : false;
			bool bKeepSkip=bMbLeftAvailPskip && bMbTopAvailPskip && bMbTopRightAvailPskip;
			m_profiler.StartSubTime(testTimer);
			if(!MdInterJudgeBGDPskip(&mb,&mbCache,&bKeepSkip,pRefPic,vaa)) {
				StartTimer(6,"Not skip");
				bool bMbTopLeftAvailPskip=mb.HasNeightbor(N_TOPLEFT) ? IS_SKIP(mb.GetNeighbor(N_TOPLEFT)->uiMbType) : false;
				bool bTrySkip=bMbLeftAvailPskip || bMbTopAvailPskip || bMbTopLeftAvailPskip || bMbTopRightAvailPskip;
				bool skipMb=false;
				bool bSkip=false;
				if(((pRefPic->m_type==P_SLICE) && (mbCache.m_uiRefMbType==MB_TYPE_SKIP)) || bTrySkip) {
					int32_t iSadPredSkip=PredictSadSkip(mbCache.sMvComponents.iRefIndexCache,mbCache.m_mbTypeSkip,mbCache.iSadCostSkip);
					bSkip=MdPSkipEnc(&sMd,&mb,&mbCache,pRefPic,iSadPredSkip);
					if(bSkip &&	bKeepSkip)
						skipMb=true;
				}
				m_profiler.StopSubTime(testTimer);
				if(!skipMb) {
					if(!bSkip) {
						m_profiler.StartSubTime(predictTimer);
						sMd.iSadPredMb=PredictSad(mbCache.sMvComponents.iRefIndexCache,mbCache.iSadCost);
						sMd.iCostLuma=MdP16x16(&mbCache,&sMd,mb,pRefPic,iMvRange);		// Lowest cost best mv
						mb.m_mb->uiMbType=MB_TYPE_16x16;
						m_profiler.StopSubTime(predictTimer);
					}
					int32_t iCostI16x16=MdI16x16(&mbCache,sMd.m_iLambda,mb);
					if(iCostI16x16<sMd.iCostLuma) {										// intra or inter
						m_profiler.StartSubTime(intraTimer);
						mb.m_mb->uiMbType=MB_TYPE_INTRA16x16;
						sMd.iCostLuma=iCostI16x16;
						mb.m_mb->uiCbp=0;
						EncRecI16x16Y(mb,&mbCache);
						uiChmaI8x8Mode=MdIntraChroma(&mbCache,sMd.m_iLambda,mb);
						IMbChromaEncode(mb,&mbCache);
						mb.m_mb->uiChromPredMode=uiChmaI8x8Mode;
						mb.m_mb->m_sadCost=0;
						m_profiler.StopSubTime(intraTimer);
					}else{
						if(bSkip) {
							skipMb=true;
						}else{
							m_profiler.StartSubTime(interTimer);
							MdInterFinePartitionVaa(&mbCache,vaa,&sMd,&mb,sMd.iCostLuma,iMvRange);
							m_profiler.StartSubTime(encodeTimer);
							MdInterMbRefinement(&sMd,&mb,&mbCache);
							m_profiler.StopSubTime(encodeTimer);
							mb.m_mb->uiCbp=0;
							InterMbEncode(&mbCache,&mb);
							PMbChromaEncode(&mbCache,&mb);
							CopyMb(pDecPic,mbCache.pMemPredLuma,mbCache.pMemPredChroma,mbCache.pMemPredChroma+64,16,8,mb);
							MdInterDoubleCheckPskip(&mb,&mbCache);
							m_profiler.StopSubTime(interTimer);
						}
					}
				}
				if(skipMb) {
					mb.m_mb->uiMbType=MB_TYPE_SKIP;
					CopyMb(pDecPic,mbCache.m_skipMb,mbCache.m_skipMb+256,mbCache.m_skipMb+320,16,8,mb);
					memset(mb.m_mb->m_nonZeroCount,0,24);
					mb.m_mb->uiCbp=0;
					mb.m_mb->uiLumaQp=uiLastMbQp;
					mbCache.bCollocatedPredFlag=mb.m_mb->m_mv[0].IsZero();
				}
				StopTimer(6);
			}else{
				SMVUnitXY sVaaPredSkipMv={0};
				PredSkipMv(mbCache,&sVaaPredSkipMv);
				mb.m_mb->uiCbp=0;
				mbCache.bCollocatedPredFlag=true;
				ST32(&mb.m_mb->sP16x16Mv,0);
				if(sVaaPredSkipMv.IsZero()) {
					McCopyWidthEq16_c(mbCache.m_refMb[0],m_stride[0],mbCache.m_skipMb,16,16);
					McCopyWidthEq8_c(mbCache.m_refMb[1],m_stride[1],mbCache.m_skipMb+256,8,8);
					McCopyWidthEq8_c(mbCache.m_refMb[2],m_stride[1],mbCache.m_skipMb+256+64,8,8);
					mb.m_mb->m_sadCost=SampleSad16x16_c(mbCache.m_encMb[0],m_stride[0],mbCache.m_refMb[0],m_stride[0]);
					sMd.iCostLuma=0;
					mb.m_mb->uiMbType=MB_TYPE_SKIP;
					ST32(mb.m_mb->m_refIndex,0);
					UpdateMbMv_c(mb.m_mb->m_mv,{0,0});
					mb.m_mb->uiLumaQp=uiLastMbQp;
					CopyMb(pDecPic,mbCache.m_skipMb,mbCache.m_skipMb+256,mbCache.m_skipMb+320,16,8,mb);
					CopyMb(pEncPic,m_prev,mb);																										// Copy prev orig frame macroblock pixels into current frame macroblock
					memset(mb.m_mb->m_nonZeroCount,0,24);
				}else{
					McCopyWidthEq16_c(mbCache.m_refMb[0],m_stride[0],mbCache.pMemPredLuma,16,16);
					McCopyWidthEq8_c(mbCache.m_refMb[1],m_stride[1],mbCache.pMemPredChroma,8,8);
					McCopyWidthEq8_c(mbCache.m_refMb[2],m_stride[1],mbCache.pMemPredChroma+64,8,8);
					mb.m_mb->m_sadCost=SampleSad16x16_c(mbCache.m_encMb[0],m_stride[0],mbCache.m_refMb[0],m_stride[0]);
					mb.m_mb->uiMbType=MB_TYPE_16x16;
					sMd.sMe.sMe16x16.sMv={0,0};
					PredMv(&sMd.sMe.sMe16x16.sMvp,&mbCache.sMvComponents,0,4,sMd.m_uiRef);
					mbCache.sMbMvp[0]=sMd.sMe.sMe16x16.sMvp;
					UpdateP16x16MotionInfo(&mbCache,&mb,sMd.m_uiRef,&sMd.sMe.sMe16x16.sMv);
					sMd.iCostLuma=mb.m_mb->m_sadCost;
					InterMbEncode(&mbCache,&mb);
					PMbChromaEncode(&mbCache,&mb);
					CopyMb(pDecPic,mbCache.pMemPredLuma,mbCache.pMemPredChroma,mbCache.pMemPredChroma+64,16,8,mb);
				}
				m_profiler.StopSubTime(testTimer);
			}
			m_profiler.StartSubTime(updateTimer);
			mbCache.pEncSad[0]=mb.m_mb->uiMbType==MB_TYPE_SKIP ? sMd.iCostSkipMb : 0;
			pDecPic->m_uiRefMbType[mb.iMbXY]=mb.m_mb->uiMbType;
			pDecPic->sMvList[mb.iMbXY]=mb.m_mb->sP16x16Mv;
			MdUpdateBGDInfo(pDecPic,mb,mbCache.bCollocatedPredFlag,pRefPic->m_type,pRefPic);
			UpdateNonZeroCountCache(mb,&mbCache);
 			OutputPMbWithoutConstructCsRsNoCopy(&mbCache,mb);
			int32_t iCurMbBits=sCabacCtx.SpatialWriteMbSynCabac(pEncPic->m_type,&mb,&uiLastMbQp,mbCache,uiChmaI8x8Mode);
			sSOverRc.iFrameBitsSlice+=iCurMbBits;
			sSOverRc.iGomBitsSlice+=iCurMbBits;
			if(iCurMbBits>0) {
				iTotalQpSlice+=mb.m_mb->uiLumaQp;
				iTotalMbSlice++;
			}
			m_profiler.StopSubTime(updateTimer);
			m_profiler.StopSubTime(blockTimer);
			StopTimer(0);
		}
		if(y>0) {		// Deblocking one line delayed. Should run one more time after loop?
			m_profiler.StartSubTime(deblockTimer);
			SDeblockingFilter filter;
			filter.pCsData[0]=pDecPic->pData[0]+(((y-1)*m_stride[0])*16);
			filter.pCsData[1]=pDecPic->pData[1]+(((y-1)*m_stride[1])*8);
			filter.pCsData[2]=pDecPic->pData[2]+(((y-1)*m_stride[2])*8);
			for(int32_t x=0;x!=m_blockWidth;x++) {
				MBWindow mb;
				mb.InitTopLeft(x,y-1,m_blockWidth,m_blockHeight,macroBlocks);
				DeblockingMbAvcbase(mb,&filter);
				filter.pCsData[0]+=MB_WIDTH_LUMA;
				filter.pCsData[1]+=MB_WIDTH_CHROMA;
				filter.pCsData[2]+=MB_WIDTH_CHROMA;
			}
			m_profiler.StopSubTime(deblockTimer);
		}
	}
	Free(pMvdCostTableBase);
	Free(macroBlocks);
	bitStringAux.pCurBuf=sCabacCtx.WriteSliceEndSyn();
	int32_t iAverageFrameQp=DIV_ROUND(INT_MULTIPLY*iTotalQpSlice,iTotalMbSlice*INT_MULTIPLY);
	int32_t iSliceSize=WriteNal(&bitStringAux,pBsBuffer,iStartPos,NAL_UNIT_CODED_SLICE);
	EndPFrame(iAverageFrameQp,frameComplexity,iSliceSize);
	Free(pBsBuffer);
}

void NewH264SVCEncoder::EncodeIFrame(SPicture* pEncPic,SPicture* pDecPic,const SPicture* pRefPic,int32_t numberMbGom) {
	SBitStringAux bitStringAux;
	uint8_t* pBsBuffer=(uint8_t*)Mallocz(m_frameBsSize);
	InitBits(&bitStringAux,pBsBuffer,m_frameBsSize);

	int32_t iStartPos=BsGetBitsPos(&bitStringAux)>>3;
	WriteSpsNal(&bitStringAux,m_iFrameNumber%MAX_SPS_COUNT);
	WriteNal(&bitStringAux,pBsBuffer,iStartPos,NAL_UNIT_SPS);
	iStartPos=BsGetBitsPos(&bitStringAux)>>3;
	WritePpsNal(&bitStringAux);
	WriteNal(&bitStringAux,pBsBuffer,iStartPos,NAL_UNIT_PPS);

	int32_t iMinFrameQp;
	int32_t iMaxFrameQp;
	int32_t iTargetBits;
	int32_t iGlobalQp=InitIFrame(&iMinFrameQp,&iMaxFrameQp,&iTargetBits);
	iStartPos=BsGetBitsPos(&bitStringAux)>>3;
	SliceHeaderWrite(&bitStringAux,iGlobalQp,pEncPic->m_type);
	SRCSlicing sSOverRc;
	memset(&sSOverRc,0,sizeof(sSOverRc));
	int32_t iBitsPerMb=DIV_ROUND64((int64_t)iTargetBits*INT_MULTIPLY,(m_blockWidth*m_blockHeight));
	sSOverRc.iTargetBitsSlice=DIV_ROUND64((int64_t)(iBitsPerMb)*(m_blockWidth*m_blockHeight),INT_MULTIPLY);
	sSOverRc.iCalculatedQpSlice=iGlobalQp;
	uint8_t uiLastMbQp=iGlobalQp;
	const int32_t blocksAlloc=m_blockWidth*3;
	MacroBlock* macroBlocks=(MacroBlock*)Mallocz(blocksAlloc*sizeof(MacroBlock));
	SCabacCtxEnc sCabacCtx;
	BsAlign(&bitStringAux);
	sCabacCtx.InitSliceCabac(bitStringAux,iGlobalQp,pEncPic->m_type);
	for(int32_t y=0;y!=m_blockHeight;y++) {
		for(int32_t x=0;x!=m_blockWidth;x++) {
			SMbCache mbCache;
			memset(&mbCache,0,sizeof(mbCache));
			SWelsMD sMd;
			MBWindow mb;
			mb.InitFull(x,y,m_blockWidth,m_blockHeight,macroBlocks);
			RcMbInitGom(&mb,&sSOverRc,numberMbGom,iMinFrameQp,iMaxFrameQp);
			MdIntraInit(&mbCache,pEncPic,pDecPic,mb);
			sMd.m_iLambda=g_kiQpCostTable[mb.m_mb->uiLumaQp];
			sMd.iCostLuma=MdI16x16(&mbCache,sMd.m_iLambda,mb);
			mb.m_mb->uiMbType=MB_TYPE_INTRA16x16;
			mb.m_mb->uiCbp=0;
			EncRecI16x16Y(mb,&mbCache);
			uint8_t uiChmaI8x8Mode=MdIntraChroma(&mbCache,sMd.m_iLambda,mb);
			IMbChromaEncode(mb,&mbCache);
			mb.m_mb->uiChromPredMode=uiChmaI8x8Mode;
			mb.m_mb->m_sadCost=0;
			UpdateNonZeroCountCache(mb,&mbCache);
			MdUpdateBGDInfo(pDecPic,mb,mbCache.bCollocatedPredFlag,I_SLICE,pRefPic);
			int32_t iCurMbBits=sCabacCtx.SpatialWriteMbSynCabac(pEncPic->m_type,&mb,&uiLastMbQp,mbCache,uiChmaI8x8Mode);
			sSOverRc.iFrameBitsSlice+=iCurMbBits;
			sSOverRc.iGomBitsSlice+=iCurMbBits;
		}
		if(y>0) {		// Deblocking one line delayed. Should run one more time after loop?
			SDeblockingFilter filter;
			filter.pCsData[0]=pDecPic->pData[0]+(((y-1)*m_stride[0])*16);
			filter.pCsData[1]=pDecPic->pData[1]+(((y-1)*m_stride[1])*8);
			filter.pCsData[2]=pDecPic->pData[2]+(((y-1)*m_stride[2])*8);
			for(int32_t x=0;x!=m_blockWidth;x++) {
				MBWindow mb;
				mb.InitTopLeft(x,y-1,m_blockWidth,m_blockHeight,macroBlocks);
				DeblockingMbAvcbase(mb,&filter);
				filter.pCsData[0]+=MB_WIDTH_LUMA;
				filter.pCsData[1]+=MB_WIDTH_CHROMA;
				filter.pCsData[2]+=MB_WIDTH_CHROMA;
			}
		}
	}
	bitStringAux.pCurBuf=sCabacCtx.WriteSliceEndSyn();
	Free(macroBlocks);
	int32_t iSliceSize=WriteNal(&bitStringAux,pBsBuffer,iStartPos,NAL_UNIT_CODED_SLICE_IDR);
	EndIFrame(iGlobalQp,iSliceSize);
	Free(pBsBuffer);
}

void NewH264SVCEncoder::SliceHeaderWrite(SBitStringAux* pBs,int32_t iGlobalQp,ESliceType eSliceType) {
	BsWriteUE(pBs,0);									// iFirstMbInSlice
	BsWriteUE(pBs,eSliceType);
	BsWriteUE(pBs,m_iFrameNumber%MAX_PPS_COUNT);
	BsWriteBits(pBs,SPS_LOG2_MAX_FRAME_NUM,m_intraFrameIndex);
	if(eSliceType==I_SLICE) {				// NAL IDR
		BsWriteUE(pBs,m_iFrameNumber+1);
	}
	BsWriteBits(pBs,(1+SPS_LOG2_MAX_FRAME_NUM),m_intraFrameIndex*2);
	if(eSliceType==P_SLICE) {
		BsWriteOneBit(pBs,true);						// NumRefIdxActiveOverrideFlag
		BsWriteUE(pBs,0);								// CLIP3(uiNumRefIdxL0Active-1,0,2)
		BsWriteOneBit(pBs,true);
		BsWriteUE(pBs,0);								// uiReorderingOfPicNumsIdc[0]
		BsWriteUE(pBs,0);								// uiAbsDiffPicNumMinus1
		BsWriteUE(pBs,3);								// uiReorderingOfPicNumsIdc[1]
		BsWriteOneBit(pBs,false);						// AdaptiveRefPicMarkingModeFlag
		BsWriteUE(pBs,0);								// iCabacInitIdc
	}else{
		BsWriteOneBit(pBs,false);						// NoOutputOfPriorPicsFlag
		BsWriteOneBit(pBs,false);						// LongTermRefFlag
	}
	BsWriteSE(pBs,iGlobalQp-PIC_INIT_QP);				// pSlice qp delta
	BsWriteUE(pBs,0);									// DisableDeblockingFilterIdc
	BsWriteSE(pBs,0);									// AlphaC0Offset>>1
	BsWriteSE(pBs,0);									// BetaOffset>>1
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

#define TC0_TBL_LOOKUP(tc,iIndexA,pBS,bChroma) {\
	tc[0]=g_kiTc0Table(iIndexA)[pBS[0]&3]+bChroma;\
	tc[1]=g_kiTc0Table(iIndexA)[pBS[1]&3]+bChroma;\
	tc[2]=g_kiTc0Table(iIndexA)[pBS[2]&3]+bChroma;\
	tc[3]=g_kiTc0Table(iIndexA)[pBS[3]&3]+bChroma;\
}

#define g_kuiAlphaTable(x) g_kuiAlphaTable[(x)]
#define g_kiBetaTable(x) g_kiBetaTable[(x)]
#define g_kiTc0Table(x) g_kiTc0Table[(x)] 

#define GET_ALPHA_BETA_FROM_QP(iQp,iIndex,iAlpha,iBeta) {\
	iIndex=(iQp);\
	iAlpha=g_kuiAlphaTable(iIndex);\
	iBeta=g_kiBetaTable((iQp));\
}

void NewH264SVCEncoder::FilteringEdgeLumaIntraV(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS) {
	int32_t iIdexA;
	int32_t iAlpha;
	int32_t iBeta;
	GET_ALPHA_BETA_FROM_QP(filter.uiLumaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		DeblockLumaEq4H_c(pPix,iStride,iAlpha,iBeta);
	}
}

void NewH264SVCEncoder::FilteringEdgeLumaIntraH(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS) {
	int32_t iIdexA;
	int32_t iAlpha;
	int32_t iBeta;
	GET_ALPHA_BETA_FROM_QP(filter.uiLumaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		DeblockLumaEq4V_c(pPix,iStride,iAlpha,iBeta);
	}
}

void NewH264SVCEncoder::FilteringEdgeLumaHV(SDeblockingFilter* pFilter,const MBWindow& mbWindow) {
	int32_t iLineSize=m_stride[0];
	int32_t iIdexA,iAlpha,iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBSx4,4,4);
	uint8_t* pDestY=pFilter->pCsData[0];
	int8_t iCurQp=mbWindow.m_mb->uiLumaQp;
	*(uint32_t*)uiBSx4=0x03030303;
	// luma v
	if(mbWindow.iMbX>0) {
		pFilter->uiLumaQP=(iCurQp+mbWindow.GetNeighbor(N_LEFT)->uiLumaQp+1)>>1;
		FilteringEdgeLumaIntraV(pDestY,*pFilter,iLineSize,NULL);
	}
	pFilter->uiLumaQP=iCurQp;
	GET_ALPHA_BETA_FROM_QP(pFilter->uiLumaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		TC0_TBL_LOOKUP(iTc,iIdexA,uiBSx4,0);
		DeblockLumaLt4H_c(&pDestY[1<<2],iLineSize,iAlpha,iBeta,iTc);
		DeblockLumaLt4H_c(&pDestY[2<<2],iLineSize,iAlpha,iBeta,iTc);
		DeblockLumaLt4H_c(&pDestY[3<<2],iLineSize,iAlpha,iBeta,iTc);
	}
	// luma h
	if(mbWindow.iMbY>0) {
		pFilter->uiLumaQP=(iCurQp+mbWindow.GetNeighbor(N_TOP)->uiLumaQp+1)>>1;
		FilteringEdgeLumaIntraH(pDestY,*pFilter,iLineSize,NULL);
	}
	pFilter->uiLumaQP=iCurQp;
	if(iAlpha | iBeta) {
		DeblockLumaLt4V_c(&pDestY[(1<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		DeblockLumaLt4V_c(&pDestY[(2<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		DeblockLumaLt4V_c(&pDestY[(3<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
	}
}

void NewH264SVCEncoder::FilteringEdgeChromaIntraV(uint8_t* pPixCb,const SDeblockingFilter& filter,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS) {
	int32_t iIdexA;
	int32_t iAlpha;
	int32_t iBeta;
	GET_ALPHA_BETA_FROM_QP(filter.uiChromaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		DeblockChromaEq4H_c(pPixCb,pPixCr,iStride,iAlpha,iBeta);
	}
}
void NewH264SVCEncoder::FilteringEdgeChromaIntraH(uint8_t* pPixCb,const SDeblockingFilter& filter,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS) {
	int32_t iIdexA;
	int32_t iAlpha;
	int32_t iBeta;
	GET_ALPHA_BETA_FROM_QP(filter.uiChromaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		DeblockChromaEq4V_c(pPixCb,pPixCr,iStride,iAlpha,iBeta);
	}
}

void NewH264SVCEncoder::FilteringEdgeLumaH(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS) {
	int32_t iIdexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	GET_ALPHA_BETA_FROM_QP(filter.uiLumaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		TC0_TBL_LOOKUP(iTc,iIdexA,pBS,0);
		DeblockLumaLt4V_c(pPix,iStride,iAlpha,iBeta,iTc);
	}
}

void NewH264SVCEncoder::FilteringEdgeChromaHV(SDeblockingFilter* pFilter,const MBWindow& mbWindow) {
	int32_t iLineSize=m_stride[1];
	uint8_t*  pDestCb,*pDestCr;
	int8_t iCurQp;
	int32_t iIdexA,iAlpha,iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBSx4,4,4);
	pDestCb=pFilter->pCsData[1];
	pDestCr=pFilter->pCsData[2];
	iCurQp=mbWindow.m_mb->GetChromaQp();
	* (uint32_t*)uiBSx4=0x03030303;
	// chroma v
	if(mbWindow.iMbX>0) {
		pFilter->uiChromaQP=(iCurQp+mbWindow.GetNeighbor(N_LEFT)->GetChromaQp()+1)>>1;
		FilteringEdgeChromaIntraV(pDestCb,*pFilter,pDestCr,iLineSize,NULL);
	}
	pFilter->uiChromaQP=iCurQp;
	GET_ALPHA_BETA_FROM_QP(pFilter->uiChromaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		TC0_TBL_LOOKUP(iTc,iIdexA,uiBSx4,1);
		DeblockChromaLt4H_c(&pDestCb[2<<1],&pDestCr[2<<1],iLineSize,iAlpha,iBeta,iTc);
	}
	// chroma h
	if(mbWindow.iMbY>0) {
		pFilter->uiChromaQP=(iCurQp+mbWindow.GetNeighbor(N_TOP)->GetChromaQp()+1)>>1;
		FilteringEdgeChromaIntraH(pDestCb,*pFilter,pDestCr,iLineSize,NULL);
	}
	pFilter->uiChromaQP=iCurQp;
	if(iAlpha | iBeta) {
		DeblockChromaLt4V_c(&pDestCb[(2<<1)*iLineSize],&pDestCr[ (2<<1)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
	}
}

void NewH264SVCEncoder::FilteringEdgeLumaV(uint8_t* pPix,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS) {
	int32_t  iIdexA;
	int32_t  iAlpha;
	int32_t  iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	GET_ALPHA_BETA_FROM_QP(filter.uiLumaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		TC0_TBL_LOOKUP(iTc,iIdexA,pBS,0);
		DeblockLumaLt4H_c(pPix,iStride,iAlpha,iBeta,iTc);
	}
}

void NewH264SVCEncoder::FilteringEdgeChromaV(uint8_t* pPixCb,uint8_t* pPixCr,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS) {
	int32_t iIdexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	GET_ALPHA_BETA_FROM_QP(filter.uiChromaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		TC0_TBL_LOOKUP(iTc,iIdexA,pBS,1);
		DeblockChromaLt4H_c(pPixCb,pPixCr,iStride,iAlpha,iBeta,iTc);
	}
}

void NewH264SVCEncoder::FilteringEdgeChromaH(uint8_t* pPixCb,uint8_t* pPixCr,const SDeblockingFilter& filter,int32_t iStride,uint8_t* pBS) {
	int32_t iIdexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	GET_ALPHA_BETA_FROM_QP(filter.uiChromaQP,iIdexA,iAlpha,iBeta);
	if(iAlpha | iBeta) {
		TC0_TBL_LOOKUP(iTc,iIdexA,pBS,1);
		DeblockChromaLt4V_c(pPixCb,pPixCr,iStride,iAlpha,iBeta,iTc);
	}
}

void NewH264SVCEncoder::DeblockingInterMb(SDeblockingFilter* pFilter,const MBWindow& mbWindow,uint8_t uiBS[2][4][4]) {
	int8_t iCurLumaQp=mbWindow.m_mb->uiLumaQp;
	int8_t iCurChromaQp=mbWindow.m_mb->GetChromaQp();
	int32_t iLineSize=m_stride[0];
	int32_t iLineSizeUV=m_stride[1];
	uint8_t* pDestY,*pDestCb,*pDestCr;
	pDestY=pFilter->pCsData[0];
	pDestCb=pFilter->pCsData[1];
	pDestCr=pFilter->pCsData[2];
	if(mbWindow.iMbX>0) {
		pFilter->uiLumaQP=(iCurLumaQp+mbWindow.GetNeighbor(N_LEFT)->uiLumaQp+1)>>1;
		pFilter->uiChromaQP=(iCurChromaQp+mbWindow.GetNeighbor(N_LEFT)->GetChromaQp()+1)>>1;
		if(uiBS[0][0][0]==0x04) {
			FilteringEdgeLumaIntraV(pDestY,*pFilter,iLineSize ,NULL);
			FilteringEdgeChromaIntraV(pDestCb,*pFilter,pDestCr,iLineSizeUV,NULL);
		}else{
			if(*(uint32_t*)uiBS[0][0]!=0) {
				FilteringEdgeLumaV(pDestY,*pFilter,iLineSize,uiBS[0][0]);
				FilteringEdgeChromaV(pDestCb,pDestCr,*pFilter,iLineSizeUV,uiBS[0][0]);
			}
		}
	}
	pFilter->uiLumaQP=iCurLumaQp;
	pFilter->uiChromaQP=iCurChromaQp;
	if(*(uint32_t*)uiBS[0][1]!=0) {
		FilteringEdgeLumaV(&pDestY[1<<2],*pFilter,iLineSize,uiBS[0][1]);
	}
	if(*(uint32_t*)uiBS[0][2]!=0) {
		FilteringEdgeLumaV(&pDestY[2<<2],*pFilter,iLineSize,uiBS[0][2]);
		FilteringEdgeChromaV(&pDestCb[2<<1],&pDestCr[2<<1],*pFilter,iLineSizeUV,uiBS[0][2]);
	}
	if(*(uint32_t*)uiBS[0][3]!=0) {
		FilteringEdgeLumaV(&pDestY[3<<2],*pFilter,iLineSize,uiBS[0][3]);
	}
	if(mbWindow.iMbY>0) {
		pFilter->uiLumaQP=(iCurLumaQp+mbWindow.GetNeighbor(N_TOP)->uiLumaQp+1)>>1;
		pFilter->uiChromaQP=(iCurChromaQp+mbWindow.GetNeighbor(N_TOP)->GetChromaQp()+1)>>1;
		if(uiBS[1][0][0]==0x04) {
			FilteringEdgeLumaIntraH(pDestY,*pFilter,iLineSize ,NULL);
			FilteringEdgeChromaIntraH(pDestCb,*pFilter,pDestCr,iLineSizeUV,NULL);
		}else{
			if(* (uint32_t*)uiBS[1][0]!=0) {
				FilteringEdgeLumaH(pDestY,*pFilter,iLineSize,uiBS[1][0]);
				FilteringEdgeChromaH(pDestCb,pDestCr,*pFilter,iLineSizeUV,uiBS[1][0]);
			}
		}
	}
	pFilter->uiLumaQP=iCurLumaQp;
	pFilter->uiChromaQP=iCurChromaQp;
	if(*(uint32_t*)uiBS[1][1]!=0) {
		FilteringEdgeLumaH(&pDestY[(1<<2)*iLineSize],*pFilter,iLineSize,uiBS[1][1]);
	}
	if(*(uint32_t*)uiBS[1][2]!=0) {
		FilteringEdgeLumaH(&pDestY[(2<<2)*iLineSize],*pFilter,iLineSize,uiBS[1][2]);
		FilteringEdgeChromaH(&pDestCb[(2<<1)*iLineSizeUV],&pDestCr[(2<<1)*iLineSizeUV],*pFilter,iLineSizeUV,uiBS[1][2]);
	}
	if(*(uint32_t*)uiBS[1][3]!=0) {
		FilteringEdgeLumaH(&pDestY[ (3<<2)*iLineSize],*pFilter,iLineSize,uiBS[1][3]);
	}
}

void NewH264SVCEncoder::DeblockingMbAvcbase(const MBWindow& mbWindow,SDeblockingFilter* pFilter) {
	switch(mbWindow.m_mb->uiMbType) {
		case MB_TYPE_INTRA16x16:
			FilteringEdgeLumaHV(pFilter,mbWindow);
			FilteringEdgeChromaHV(pFilter,mbWindow);
			break;
		default:
			DeblockingBSCalc_c(pFilter,mbWindow.m_mb->uiMbType,mbWindow);
			break;
	}
}

void NewH264SVCEncoder::ExpandReferencingPicture(uint8_t* pData[3],int32_t iWidth,int32_t iHeight,int32_t iStride[3]) {
	uint8_t* pPicY=pData[0];
	uint8_t* pPicCb=pData[1];
	uint8_t* pPicCr=pData[2];
	const int32_t kiWidthY=iWidth;
	const int32_t kiHeightY=iHeight;
	const int32_t kiWidthUV=kiWidthY>>1;
	const int32_t kiHeightUV=kiHeightY>>1;
	ExpandPictureLuma_c(pPicY,iStride[0],kiWidthY,kiHeightY);
	ExpandPictureChroma_c(pPicCb,iStride[1],kiWidthUV,kiHeightUV);
	ExpandPictureChroma_c(pPicCr,iStride[2],kiWidthUV,kiHeightUV);
}

const uint8_t* NewH264SVCEncoder::EncodeFrame(uint32_t* byteSize,const SSourcePicture* pSrcPic) {
#ifdef CABAC_TEST
	TestCABAC();
#endif
	uint32_t memoryBlockPos=m_memoryBlockPos;
	START_TIMER(enctimer,&m_profiler,"Encode",0xffff20);
	CopyPicture(m_cur,pSrcPic);
	EVideoFrameType eFrameType=(m_intraFrameIndex>=15 || m_iFrameNumber==-1) ? videoFrameTypeIDR : videoFrameTypeP;
	SPicture* pEncPic=m_cur;
	if(eFrameType==videoFrameTypeP) {
		pEncPic->m_type=P_SLICE;
		m_intraFrameIndex++;
	}else{
		m_iFrameNumber++;
		pEncPic->m_type=I_SLICE;
		m_intraFrameIndex=0;
	}
	SPicture* pRefPic=m_refPics[(m_frameNumber+1)&1];									// Decoded image previous frame
	SPicture* pDecPic=m_refPics[m_frameNumber&1];										// This buffer will contain the current decoded image after encode
	//ClearPicture(pDecPic,true,true);													// Not needed, but good for validation
	pDecPic->m_type=pEncPic->m_type;
	int32_t numberMbGom=m_videoWidth<=480 ? m_blockWidth:m_blockWidth*2;
	memset(m_frameBs,0,m_frameBsSize);
	m_posBsBuffer=0;		// reset bs pBuffer position
	if(eFrameType==videoFrameTypeP) {
		START_TIMER(encptimer,&m_profiler,"EncodeP",0xff0020);
		START_TIMER(bgTimer,&m_profiler,"Background",0x80ffff);
		SVAAFrameInfo vaa;
		memset(&vaa,0,sizeof(vaa));
		int32_t blocks=m_blockWidth*m_blockHeight;
		vaa.m_vaaBackgroundMbFlag=(int8_t*)Mallocz(blocks*sizeof(int8_t));
		vaa.m_sad8x8=(int32_t*)Mallocz(blocks*4*sizeof(int32_t));
		int32_t* sumOfDiff8x8=(int32_t*)Mallocz(blocks*4*sizeof(int32_t));
		vaa.m_mad8x8=(uint8_t*)Mallocz(blocks*4*sizeof(uint8_t));
		VAACalcSadBgd_c(pEncPic->pData[0],m_prev->pData[0],m_videoWidth,m_videoHeight,m_stride[0],(int32_t*)vaa.m_sad8x8,(int32_t*)sumOfDiff8x8,(uint8_t*)vaa.m_mad8x8);
		BackgroundDetection(&vaa,pEncPic,m_prev,(int*)sumOfDiff8x8);
		Free(sumOfDiff8x8);
		uint32_t frameComplexity=AnalyzePictureComplexity(&vaa,pEncPic,pRefPic,numberMbGom);
		Free(vaa.m_mad8x8);
		vaa.m_mad8x8=0;
		END_TIMER(bgTimer,&m_profiler);
		EncodePFrame(vaa,pEncPic,pDecPic,pRefPic,numberMbGom,frameComplexity);
		Free(vaa.m_vaaBackgroundMbFlag);
		Free(vaa.m_sad8x8);
		END_TIMER(encptimer,&m_profiler);
	}else{
		START_TIMER(encptimer,&m_profiler,"EncodeI",0x00ff20);
		EncodeIFrame(pEncPic,pDecPic,pRefPic,numberMbGom);
		END_TIMER(encptimer,&m_profiler);
	}

	ExpandReferencingPicture(pDecPic->pData,m_videoWidth,m_videoHeight,m_stride);
	SPicture* tmp=m_cur;												//exchange cur/prev
	m_cur=m_prev;
	m_prev=tmp;
	m_frameNumber++;
	if(pEncPic->m_type!=I_SLICE) {
		m_firstPFrame=false;
	}
	END_TIMER(enctimer,&m_profiler);
	m_profilerDisplay.clear();
	m_profiler.GetDisplayTimers(m_profilerDisplay);
	m_profiler.Reset();
	m_memoryBlockPos=memoryBlockPos;				// Reset allocator
	*byteSize=m_posBsBuffer;
	/*
	{
		TestDec::ISVCDecoderBase* pSvcDecoder=0;
		CreateDecoder(&pSvcDecoder);
		TestDec::SDecodingParam sDecParam={0};
		pSvcDecoder->Initialize(&sDecParam);
		if(!pSvcDecoder->DecodeFrame(m_frameBs,*byteSize))
			FATAL("Decodeframe returned false");

	}
	*/
	return m_frameBs;
}

int NewH264SVCEncoder::Initialize(const SEncParamBase& paramBase) {
	m_profiler.SetRange(40);
	m_videoWidth=paramBase.iPicWidth;
	m_videoHeight=paramBase.iPicHeight;
	if(m_videoWidth&0xf || m_videoHeight&0xf)
		FATAL("Width %d Height %s invalid, must be multiplex of 16",m_videoWidth,m_videoHeight);

	m_blockWidth=m_videoWidth>>4;
	m_blockHeight=m_videoHeight>>4;
	int32_t width=m_videoWidth+(PADDING_LENGTH<<1);
	m_stride[0]=width;
	m_stride[1]=width>>1;
	m_stride[2]=width>>1;

	m_iTargetBitrate=paramBase.iTargetBitrate;	// target bitrate for current spatial layer
	InitEncoderExt();
	return cmResultSuccess;
}

void ClearEndOfLinePadding(uint8_t* pData,int32_t iStride,int32_t iWidth,int32_t iHeight) {
	if(iWidth<iStride) {
		for(int32_t i=0;i<iHeight;++i)
			memset(pData+i*iStride+iWidth,0,iStride-iWidth);
	}
}

//i420_to_i420_c
static void WelsMoveMemory_c(uint8_t* pDstY,uint8_t* pDstU,uint8_t* pDstV,int32_t iDstStrideY,int32_t iDstStrideUV,uint8_t* pSrcY,uint8_t* pSrcU,uint8_t* pSrcV,int32_t iSrcStrideY,int32_t iSrcStrideUV,int32_t iWidth,int32_t iHeight) {
	int32_t iWidth2=iWidth>>1;
	int32_t iHeight2=iHeight>>1;
	int32_t j;
	for(j=iHeight;j;j--) {
		::memcpy(pDstY,pSrcY,iWidth);
		pDstY+=iDstStrideY;
		pSrcY+=iSrcStrideY;
	}
	for(j=iHeight2;j;j--) {
		::memcpy(pDstU,pSrcU,iWidth2);
		::memcpy(pDstV,pSrcV,iWidth2);
		pDstU+=iDstStrideUV;
		pDstV+=iDstStrideUV;
		pSrcU+=iSrcStrideUV;
		pSrcV+=iSrcStrideUV;
	}
}

void NewH264SVCEncoder::VAACalcSadBgd_c(const uint8_t* pCurData,const uint8_t* pRefData,int32_t iPicWidth,int32_t iPicHeight,int32_t iPicStride,int32_t* pSad8x8,int32_t* pSd8x8,uint8_t* pMad8x8) {
	const uint8_t* tmp_ref=pRefData;
	const uint8_t* tmp_cur=pCurData;
	int32_t iMbWidth=(iPicWidth>>4);
	int32_t mb_height=(iPicHeight>>4);
	int32_t mb_index=0;
	int32_t pic_stride_x8=iPicStride<<3;
	int32_t step=(iPicStride<<4)-iPicWidth;
	//*pFrameSad=0;
	for(int32_t i=0;i<mb_height;i++) {
		for(int32_t j=0;j<iMbWidth;j++) {
			int32_t k,l;
			int32_t l_sad,l_sd,l_mad;
			const uint8_t* tmp_cur_row;
			const uint8_t* tmp_ref_row;
			l_mad=l_sd=l_sad=0;
			tmp_cur_row=tmp_cur;
			tmp_ref_row=tmp_ref;
			for(k=0;k<8;k++) {
				for(l=0;l<8;l++) {
					int32_t diff=tmp_cur_row[l]-tmp_ref_row[l];
					int32_t abs_diff=ABS(diff);
					l_sd+=diff;
					l_sad+=abs_diff;
					if(abs_diff>l_mad) {
						l_mad=abs_diff;
					}
				}
				tmp_cur_row+=iPicStride;
				tmp_ref_row+=iPicStride;
			}
			//*pFrameSad+=l_sad;
			pSad8x8[(mb_index<<2)+0]=l_sad;
			pSd8x8[(mb_index<<2)+0]=l_sd;
			pMad8x8[(mb_index<<2)+0]=l_mad;

			l_mad=l_sd=l_sad=0;
			tmp_cur_row=tmp_cur+8;
			tmp_ref_row=tmp_ref+8;
			for(k=0;k<8;k++) {
				for(l=0;l<8;l++) {
					int32_t diff=tmp_cur_row[l]-tmp_ref_row[l];
					int32_t abs_diff=ABS(diff);
					l_sd+=diff;
					l_sad+=abs_diff;
					if(abs_diff>l_mad) {
						l_mad=abs_diff;
					}
				}
				tmp_cur_row+=iPicStride;
				tmp_ref_row+=iPicStride;
			}
			//*pFrameSad+=l_sad;
			pSad8x8[ (mb_index<<2)+1]=l_sad;
			pSd8x8[ (mb_index<<2)+1]=l_sd;
			pMad8x8[ (mb_index<<2)+1]=l_mad;
			l_mad=l_sd=l_sad=0;
			tmp_cur_row=tmp_cur+pic_stride_x8;
			tmp_ref_row=tmp_ref+pic_stride_x8;
			for(k=0;k<8;k++) {
				for(l=0;l<8;l++) {
					int32_t diff=tmp_cur_row[l]-tmp_ref_row[l];
					int32_t abs_diff=ABS(diff);
					l_sd+=diff;
					l_sad+=abs_diff;
					if(abs_diff>l_mad) {
						l_mad=abs_diff;
					}
				}
				tmp_cur_row+=iPicStride;
				tmp_ref_row+=iPicStride;
			}
			//*pFrameSad+=l_sad;
			pSad8x8[(mb_index<<2)+2]=l_sad;
			pSd8x8[(mb_index<<2)+2]=l_sd;
			pMad8x8[(mb_index<<2)+2]=l_mad;

			l_mad=l_sd=l_sad=0;
			tmp_cur_row=tmp_cur+pic_stride_x8+8;
			tmp_ref_row=tmp_ref+pic_stride_x8+8;
			for(k=0;k<8;k++) {
				for(l=0;l<8;l++) {
					int32_t diff=tmp_cur_row[l]-tmp_ref_row[l];
					int32_t abs_diff=ABS(diff);
					l_sd+=diff;
					l_sad+=abs_diff;
					if(abs_diff>l_mad) {
						l_mad=abs_diff;
					}
				}
				tmp_cur_row+=iPicStride;
				tmp_ref_row+=iPicStride;
			}
			//*pFrameSad+=l_sad;
			pSad8x8[(mb_index<<2)+3]=l_sad;
			pSd8x8[(mb_index<<2)+3]=l_sd;
			pMad8x8[(mb_index<<2)+3]=l_mad;
			tmp_ref+=16;
			tmp_cur+=16;
			++mb_index;
		}
		tmp_ref+=step;
		tmp_cur+=step;
	}
}

NewH264SVCEncoder::NewH264SVCEncoder() {
	CreateMalloc();
	m_videoWidth=0;
	m_videoHeight=0;
}

NewH264SVCEncoder::~NewH264SVCEncoder() {
	Free(m_strideTable.m_allocBase);
	Free(m_frameBs);

	FreePicture(m_refPics[0]);
	FreePicture(m_refPics[1]);
	FreePicture(m_prev);
	FreePicture(m_cur);

	delete m_refPics[0];
	delete m_refPics[1];
	delete m_cur;
	delete m_prev;
	DestroyMalloc();

	PrintTimers();
}

uint32_t NewH264SVCEncoder::AnalyzePictureComplexity(SVAAFrameInfo* vaa,SPicture* pCurPicture,const SPicture* pRefPic,int32_t iMbNumInGom) {
	uint8_t* pBackgroundMbFlag=(uint8_t*)vaa->m_vaaBackgroundMbFlag;
	int32_t iMbNum=m_blockWidth*m_blockHeight;
	int32_t iGomMbNum=(iMbNum+iMbNumInGom-1)/iMbNumInGom;
	uint32_t* uiRefMbType=pRefPic->m_uiRefMbType;
	uint32_t uiFrameSad=0;
	for(int32_t j=0;j<iGomMbNum;j++) {
		int32_t iGomMbStartIndex=j*iMbNumInGom;
		int32_t iGomMbEndIndex=MIN((j+1)*iMbNumInGom,iMbNum);
		for(int32_t i=iGomMbStartIndex;i<iGomMbEndIndex;i++) {
			if(pBackgroundMbFlag[i]==0 || IS_INTRA(uiRefMbType[i])) {
				uiFrameSad+=vaa->m_sad8x8[i*4+0];
				uiFrameSad+=vaa->m_sad8x8[i*4+1];
				uiFrameSad+=vaa->m_sad8x8[i*4+2];
				uiFrameSad+=vaa->m_sad8x8[i*4+3];
			}
		}
	}
	return uiFrameSad;
}

void NewH264SVCEncoder::CopyPicture(SPicture* pDstPic,const SSourcePicture* kpSrc) {
	int32_t  iSrcWidth=kpSrc->iPicWidth;
	int32_t  iSrcHeight=kpSrc->iPicHeight;
	const int32_t kiSrcTopOffsetY=0;
	const int32_t kiSrcTopOffsetUV=(kiSrcTopOffsetY>>1);
	const int32_t kiSrcLeftOffsetY=0;
	const int32_t kiSrcLeftOffsetUV=(kiSrcLeftOffsetY>>1);
	int32_t  iSrcOffset[3]={0,0,0};
	iSrcOffset[0]=kpSrc->iStride[0]*kiSrcTopOffsetY+kiSrcLeftOffsetY;
	iSrcOffset[1]=kpSrc->iStride[1]*kiSrcTopOffsetUV+kiSrcLeftOffsetUV;
	iSrcOffset[2]=kpSrc->iStride[2]*kiSrcTopOffsetUV+kiSrcLeftOffsetUV;
	uint8_t* pSrcY=kpSrc->pData[0]+iSrcOffset[0];
	uint8_t* pSrcU=kpSrc->pData[1]+iSrcOffset[1];
	uint8_t* pSrcV=kpSrc->pData[2]+iSrcOffset[2];
	const int32_t kiSrcStrideY=kpSrc->iStride[0];
	const int32_t kiSrcStrideUV=kpSrc->iStride[1];
	uint8_t* pDstY=pDstPic->pData[0];
	uint8_t* pDstU=pDstPic->pData[1];
	uint8_t* pDstV=pDstPic->pData[2];
	const int32_t kiDstStrideY=m_stride[0];
	const int32_t kiDstStrideUV=m_stride[1];
	WelsMoveMemory_c(pDstY,pDstU,pDstV,kiDstStrideY,kiDstStrideUV,pSrcY,pSrcU,pSrcV,kiSrcStrideY,kiSrcStrideUV,iSrcWidth,iSrcHeight);
}
#define LOG2_BGD_OU_SIZE (4)
#define LOG2_BGD_OU_SIZE_UV (LOG2_BGD_OU_SIZE-1)
#define BGD_OU_SIZE (1<<LOG2_BGD_OU_SIZE)
#define BGD_OU_SIZE_UV (BGD_OU_SIZE>>1)
#define BGD_THD_SAD (2*BGD_OU_SIZE*BGD_OU_SIZE)
#define BGD_THD_ASD_UV (4*BGD_OU_SIZE_UV)
#define LOG2_MB_SIZE (4)
#define OU_SIZE_IN_MB (BGD_OU_SIZE>>4)
#define Q_FACTOR (8)

#define OU_LEFT (0x01)
#define OU_RIGHT (0x02)
#define OU_TOP (0x04)
#define OU_BOTTOM (0x08)

void NewH264SVCEncoder::BackgroundDetection(SVAAFrameInfo* vaa,SPicture* pCurPicture,SPicture* pRefPicture,int* sumOfDiff8x8) {
	if(m_prev->m_type==I_SLICE) {
		memset(vaa->m_vaaBackgroundMbFlag,0,m_blockWidth*m_blockHeight);
		return;
	}
	SBackgroundOU* pBackgroundOUStart=(SBackgroundOU*)alloca(m_blockWidth*m_blockHeight*sizeof(SBackgroundOU));
	SBackgroundOU* pBackgroundOU=pBackgroundOUStart;
	int32_t iPicWidthInOU=m_videoWidth >>LOG2_BGD_OU_SIZE;
	int32_t iPicHeightInOU=m_videoHeight>>LOG2_BGD_OU_SIZE;
	int32_t iPicWidthInMb=(15+m_videoWidth)>>4;
	for(int32_t j=0;j<iPicHeightInOU;j++) {
		for(int32_t i=0;i<iPicWidthInOU;i++) {
			int32_t iMbIndex=(j*iPicWidthInMb+i)<<(LOG2_BGD_OU_SIZE-LOG2_MB_SIZE);
			SBackgroundOU* pBgdOU=pBackgroundOU;
			int32_t* pSad8x8=(int32_t*)vaa->m_sad8x8;
			uint8_t* pMad8x8=(uint8_t*)vaa->m_mad8x8;
			int32_t iSubSAD_0=pSad8x8[iMbIndex*4+0];
			int32_t iSubSAD_1=pSad8x8[iMbIndex*4+1];
			int32_t iSubSAD_2=pSad8x8[iMbIndex*4+2];
			int32_t iSubSAD_3=pSad8x8[iMbIndex*4+3];
			int32_t iSubSD_0=sumOfDiff8x8[iMbIndex*4+0];
			int32_t iSubSD_1=sumOfDiff8x8[iMbIndex*4+1];
			int32_t iSubSD_2=sumOfDiff8x8[iMbIndex*4+2];
			int32_t iSubSD_3=sumOfDiff8x8[iMbIndex*4+3];
			uint8_t iSubMAD_0=pMad8x8[iMbIndex*4+0];
			uint8_t iSubMAD_1=pMad8x8[iMbIndex*4+1];
			uint8_t iSubMAD_2=pMad8x8[iMbIndex*4+2];
			uint8_t iSubMAD_3=pMad8x8[iMbIndex*4+3];
			pBgdOU->iSD=iSubSD_0+iSubSD_1+iSubSD_2+iSubSD_3;
			pBgdOU->iSAD=iSubSAD_0+iSubSAD_1+iSubSAD_2+iSubSAD_3;
			pBgdOU->iSD=ABS(pBgdOU->iSD);
			// get the max absolute difference (MAD) of OU and min value of the MAD of sub-blocks of OU
			pBgdOU->iMAD=MAX(MAX(iSubMAD_0,iSubMAD_1),MAX(iSubMAD_2,iSubMAD_3));
			pBgdOU->iMinSubMad=MIN(MIN(iSubMAD_0,iSubMAD_1),MIN(iSubMAD_2,iSubMAD_3));
			// get difference between the max and min SD of the SDs of sub-blocks of OU
			pBgdOU->iMaxDiffSubSd=MAX(MAX(iSubSD_0,iSubSD_1),MAX(iSubSD_2,iSubSD_3))-MIN(MIN(iSubSD_0,iSubSD_1),MIN(iSubSD_2,iSubSD_3));
			pBackgroundOU->iBackgroundFlag=0;
			if(pBackgroundOU->iMAD>63) {
				pBackgroundOU++;
				continue;
			}
			if((pBackgroundOU->iMaxDiffSubSd <=pBackgroundOU->iSAD>>3 || pBackgroundOU->iMaxDiffSubSd <=(BGD_OU_SIZE*Q_FACTOR)) && pBackgroundOU->iSAD<(BGD_THD_SAD<<1)) { //BGD_OU_SIZE*BGD_OU_SIZE>>2
				if(pBackgroundOU->iSAD <=BGD_OU_SIZE*Q_FACTOR) {
					pBackgroundOU->iBackgroundFlag=1;
				}else{
					pBackgroundOU->iBackgroundFlag=pBackgroundOU->iSAD<BGD_THD_SAD ? (pBackgroundOU->iSD<(pBackgroundOU->iSAD*3)>>2) : (pBackgroundOU->iSD<<1<pBackgroundOU->iSAD);
				}
			}
			pBackgroundOU++;
		}
	}
	ForegroundDilationAndBackgroundErosion(vaa,pBackgroundOUStart,pCurPicture->pData[1],pCurPicture->pData[2],pRefPicture->pData[1],pRefPicture->pData[2]);
}
inline int32_t NewH264SVCEncoder::CalculateAsdChromaEdge(uint8_t* pOriRef,uint8_t* pOriCur,int32_t iStride) {
	int32_t ASD=0;
	int32_t idx;
	for(idx=0;idx<BGD_OU_SIZE_UV;idx++) {
		ASD+=*pOriCur-*pOriRef;
		pOriRef+=iStride;
		pOriCur+=iStride;
	}
	return ABS(ASD);
}

inline bool NewH264SVCEncoder::ForegroundDilation23Luma(SBackgroundOU* pBackgroundOU,SBackgroundOU* pOUNeighbours[]) {
	SBackgroundOU* pOU_L=pOUNeighbours[0];
	SBackgroundOU* pOU_R=pOUNeighbours[1];
	SBackgroundOU* pOU_U=pOUNeighbours[2];
	SBackgroundOU* pOU_D=pOUNeighbours[3];
	if(pBackgroundOU->iMAD>pBackgroundOU->iMinSubMad<<1) {
		int32_t iMaxNbrForegroundMad;
		int32_t iMaxNbrBackgroundMad;
		int32_t aBackgroundMad[4];
		int32_t aForegroundMad[4];
		aForegroundMad[0]=(pOU_L->iBackgroundFlag-1)&pOU_L->iMAD;
		aForegroundMad[1]=(pOU_R->iBackgroundFlag-1)&pOU_R->iMAD;
		aForegroundMad[2]=(pOU_U->iBackgroundFlag-1)&pOU_U->iMAD;
		aForegroundMad[3]=(pOU_D->iBackgroundFlag-1)&pOU_D->iMAD;
		iMaxNbrForegroundMad=MAX(MAX(aForegroundMad[0],aForegroundMad[1]),MAX(aForegroundMad[2],aForegroundMad[3]));
		aBackgroundMad[0]=((!pOU_L->iBackgroundFlag)-1)&pOU_L->iMAD;
		aBackgroundMad[1]=((!pOU_R->iBackgroundFlag)-1)&pOU_R->iMAD;
		aBackgroundMad[2]=((!pOU_U->iBackgroundFlag)-1)&pOU_U->iMAD;
		aBackgroundMad[3]=((!pOU_D->iBackgroundFlag)-1)&pOU_D->iMAD;
		iMaxNbrBackgroundMad=MAX(MAX(aBackgroundMad[0],aBackgroundMad[1]),MAX(aBackgroundMad[2],aBackgroundMad[3]));
		return ((iMaxNbrForegroundMad>pBackgroundOU->iMinSubMad<<2) || (pBackgroundOU->iMAD>iMaxNbrBackgroundMad<<1	&& pBackgroundOU->iMAD <=(iMaxNbrForegroundMad*3)>>1));
	}
	return 0;
}

inline bool NewH264SVCEncoder::ForegroundDilation23Chroma(int8_t iNeighbourForegroundFlags,int32_t iStartSamplePos,int32_t iPicStrideUV,uint8_t* pCurU,uint8_t* pCurV,uint8_t* pRefU,uint8_t* pRefV) {
	static const int8_t kaOUPos[4]={OU_LEFT,OU_RIGHT,OU_TOP,OU_BOTTOM};
	int32_t aEdgeOffset[4]={0,BGD_OU_SIZE_UV-1,0,iPicStrideUV* (BGD_OU_SIZE_UV-1)};
	int32_t iStride[4]={iPicStrideUV,iPicStrideUV,1,1};
	// V component first,high probability because V stands for red color and human skin colors have more weight on this component
	for(int32_t i=0;i<4;i++) {
		if(iNeighbourForegroundFlags&kaOUPos[i]) {
			uint8_t* pRefC=pRefV+iStartSamplePos+aEdgeOffset[i];
			uint8_t* pCurC=pCurV+iStartSamplePos+aEdgeOffset[i];
			if(CalculateAsdChromaEdge(pRefC,pCurC,iStride[i])>BGD_THD_ASD_UV) {
				return 1;
			}
		}
	}
	// U component,which stands for blue color,low probability
	for(int32_t i=0;i<4;i++) {
		if(iNeighbourForegroundFlags&kaOUPos[i]) {
			uint8_t* pRefC=pRefU+iStartSamplePos+aEdgeOffset[i];
			uint8_t* pCurC=pCurU+iStartSamplePos+aEdgeOffset[i];
			if(CalculateAsdChromaEdge(pRefC,pCurC,iStride[i])>BGD_THD_ASD_UV) {
				return 1;
			}
		}
	}
	return 0;
}
inline void NewH264SVCEncoder::ForegroundDilation(SBackgroundOU* pBackgroundOU,SBackgroundOU* pOUNeighbours[],int32_t iChromaSampleStartPos,uint8_t* pCurU,uint8_t* pCurV,uint8_t* pRefU,uint8_t* pRefV) {
	int32_t iPicStrideUV=m_stride[1];
	int32_t iSumNeighBackgroundFlags=pOUNeighbours[0]->iBackgroundFlag+pOUNeighbours[1]->iBackgroundFlag+pOUNeighbours[2]->iBackgroundFlag+pOUNeighbours[3]->iBackgroundFlag;
	if(pBackgroundOU->iSAD>BGD_OU_SIZE*Q_FACTOR) {
		switch(iSumNeighBackgroundFlags) {
			case 0:
			case 1:
				pBackgroundOU->iBackgroundFlag=0;
				break;
			case 2:
			case 3: {
				pBackgroundOU->iBackgroundFlag=!ForegroundDilation23Luma(pBackgroundOU,pOUNeighbours);
				// chroma component check
				if(pBackgroundOU->iBackgroundFlag==1) {
					int8_t iNeighbourForegroundFlags=(!pOUNeighbours[0]->iBackgroundFlag)|((!pOUNeighbours[1]->iBackgroundFlag)<<1)|((!pOUNeighbours[2]->iBackgroundFlag)<<2)|((!pOUNeighbours[3]->iBackgroundFlag)<<3);
					pBackgroundOU->iBackgroundFlag=!ForegroundDilation23Chroma(iNeighbourForegroundFlags,iChromaSampleStartPos,iPicStrideUV,pCurU,pCurV,pRefU,pRefV);
				}
				break;
			}
			default:
				break;
		}
	}
}
inline void NewH264SVCEncoder::BackgroundErosion(SBackgroundOU* pBackgroundOU,SBackgroundOU* pOUNeighbours[]) {
	if(pBackgroundOU->iMaxDiffSubSd <=(BGD_OU_SIZE*Q_FACTOR)) { //BGD_OU_SIZE*BGD_OU_SIZE>>2
		int32_t iSumNeighBackgroundFlags=pOUNeighbours[0]->iBackgroundFlag+pOUNeighbours[1]->iBackgroundFlag+pOUNeighbours[2]->iBackgroundFlag+pOUNeighbours[3]->iBackgroundFlag;
		int32_t sumNbrBGsad=(pOUNeighbours[0]->iSAD&(-pOUNeighbours[0]->iBackgroundFlag))+(pOUNeighbours[2]->iSAD&(-pOUNeighbours[2]->iBackgroundFlag))+(pOUNeighbours[1]->iSAD&(-pOUNeighbours[1]->iBackgroundFlag))+(pOUNeighbours[3]->iSAD&(-pOUNeighbours[3]->iBackgroundFlag));
		if(pBackgroundOU->iSAD*iSumNeighBackgroundFlags <=(3*sumNbrBGsad)>>1) {
			if(iSumNeighBackgroundFlags==4) {
				pBackgroundOU->iBackgroundFlag=1;
			}else{
				if((pOUNeighbours[0]->iBackgroundFlag&pOUNeighbours[1]->iBackgroundFlag) || (pOUNeighbours[2]->iBackgroundFlag&pOUNeighbours[3]->iBackgroundFlag)) {
					pBackgroundOU->iBackgroundFlag=!ForegroundDilation23Luma(pBackgroundOU,pOUNeighbours);
				}
			}
		}
	}
}

inline void NewH264SVCEncoder::SetBackgroundMbFlag(int8_t* pBackgroundMbFlag,int32_t iPicWidthInMb,int32_t iBackgroundMbFlag) {
	*pBackgroundMbFlag=iBackgroundMbFlag;
}

inline void NewH264SVCEncoder::UpperOUForegroundCheck(SBackgroundOU* pCurOU,int8_t* pBackgroundMbFlag,int32_t iPicWidthInOU,int32_t iPicWidthInMb) {
	if(pCurOU->iSAD>BGD_OU_SIZE*Q_FACTOR) {
		SBackgroundOU* pOU_L=pCurOU-1;
		SBackgroundOU* pOU_R=pCurOU+1;
		SBackgroundOU* pOU_U=pCurOU-iPicWidthInOU;
		SBackgroundOU* pOU_D=pCurOU+iPicWidthInOU;
		if(pOU_L->iBackgroundFlag+pOU_R->iBackgroundFlag+pOU_U->iBackgroundFlag+pOU_D->iBackgroundFlag <=1) {
			SetBackgroundMbFlag(pBackgroundMbFlag,iPicWidthInMb,0);
			pCurOU->iBackgroundFlag=0;
		}
	}
}

void NewH264SVCEncoder::ForegroundDilationAndBackgroundErosion(SVAAFrameInfo* vaa,SBackgroundOU* pBackgroundOU,uint8_t* pCurU,uint8_t* pCurV,uint8_t* pRefU,uint8_t* pRefV) {
	int32_t iPicStrideUV=m_stride[1];
	int32_t iPicWidthInOU=m_videoWidth >>LOG2_BGD_OU_SIZE; 
	int32_t iPicHeightInOU=m_videoHeight>>LOG2_BGD_OU_SIZE;
	int32_t iOUStrideUV=iPicStrideUV<<(LOG2_BGD_OU_SIZE-1);
	int32_t iPicWidthInMb=(15+m_videoWidth)>>4;
	int8_t* pVaaBackgroundMbFlag=(int8_t*)vaa->m_vaaBackgroundMbFlag;
	SBackgroundOU* pOUNeighbours[4];													//0: left;1: right;2: top;3: bottom
	pOUNeighbours[2]=pBackgroundOU;														//top OU
	for(int32_t j=0;j<iPicHeightInOU;j++) {
		int8_t* pRowSkipFlag=pVaaBackgroundMbFlag;
		pOUNeighbours[0]=pBackgroundOU;													//left OU
		pOUNeighbours[3]=pBackgroundOU+(iPicWidthInOU&((j==iPicHeightInOU-1)-1));		//bottom OU
		for(int32_t i=0;i<iPicWidthInOU;i++) {
			pOUNeighbours[1]=pBackgroundOU+(i<iPicWidthInOU-1);							//right OU
			if(pBackgroundOU->iBackgroundFlag)
				ForegroundDilation(pBackgroundOU,pOUNeighbours,j*iOUStrideUV+(i<<LOG2_BGD_OU_SIZE_UV),pCurU,pCurV,pRefU,pRefV);
			else
				BackgroundErosion(pBackgroundOU,pOUNeighbours);
			// check the up OU
			if(j>1 && i>0 && i<iPicWidthInOU-1 && pOUNeighbours[2]->iBackgroundFlag==1) {
				UpperOUForegroundCheck(pOUNeighbours[2],pRowSkipFlag-OU_SIZE_IN_MB*iPicWidthInMb,iPicWidthInOU,iPicWidthInMb);
			}
			SetBackgroundMbFlag(pRowSkipFlag,iPicWidthInMb,pBackgroundOU->iBackgroundFlag);
			// preparation for the next OU
			pRowSkipFlag+=OU_SIZE_IN_MB;
			pOUNeighbours[0]=pBackgroundOU;
			pOUNeighbours[2]++;
			pOUNeighbours[3]++;
			pBackgroundOU++;
		}
		pOUNeighbours[2]=pBackgroundOU-iPicWidthInOU;
		pVaaBackgroundMbFlag+=OU_SIZE_IN_MB*iPicWidthInMb;
	}
}

void DestroySVCEncoder(ISVCEncoder* pEncoder) {
	NewH264SVCEncoder* pSVCEncoder=(NewH264SVCEncoder*)pEncoder;
	if(pSVCEncoder) {
		delete pSVCEncoder;
		pSVCEncoder=NULL;
	}
}

ISVCEncoder* CreateSVCEncoder() {
	return new NewH264SVCEncoder();
}

//SCabacCtxEnc
void SCabacCtxEnc::InitSliceCabac(const SBitStringAux& pBs,int32_t iQp,ESliceType eSliceType) {
	int32_t iModel=eSliceType==I_SLICE ? 0:1;
	for(int32_t iIdx=0;iIdx<CABAC_CONTEXT_COUNT;iIdx++) {
		int32_t m=g_kiCabacGlobalContextIdx[iIdx][iModel][0];
		int32_t n=g_kiCabacGlobalContextIdx[iIdx][iModel][1];
		int32_t iPreCtxState=CLIP3((((m*iQp)>>4)+n),1,126);
		uint8_t uiValMps=0;
		uint8_t uiStateIdx=0;
		if(iPreCtxState<=63) {
			uiStateIdx=63-iPreCtxState;
			uiValMps=0;
		}else{
			uiStateIdx=iPreCtxState-64;
			uiValMps=1;
		}
		m_sStateCtx[iIdx].Set(uiStateIdx,uiValMps);
	}
	m_uiLow=0;
	m_iLowBitCnt=9;
	m_iRenormCnt=0;
	m_uiRange=510;
	m_pBufStart=pBs.pCurBuf;
	m_pBufCur=pBs.pCurBuf;
}

inline void PropagateCarry(uint8_t* pBufCur,uint8_t* pBufStart) {
	for(;pBufCur>pBufStart;--pBufCur)
		if(++*(pBufCur-1))
			break;
}
void SCabacCtxEnc::CabacEncodeDecision(int32_t iCtx,uint32_t uiBin) {
	if(uiBin==m_sStateCtx[iCtx].Mps()) {
		const int32_t kiState=m_sStateCtx[iCtx].State();
		uint32_t uiRange=m_uiRange;
		uint32_t uiRangeLps=g_kuiCabacRangeLps[kiState][(uiRange&0xff)>>6];
		uiRange-=uiRangeLps;
		const int32_t kiRenormAmount=uiRange>>8^1;
		m_uiRange=uiRange<<kiRenormAmount;
		m_iRenormCnt+=kiRenormAmount;
		m_sStateCtx[iCtx].Set(g_kuiStateTransTable[kiState][1],uiBin);
	}else{
		const int32_t kiState=m_sStateCtx[iCtx].State();
		uint32_t uiRange=m_uiRange;
		uint32_t uiRangeLps=g_kuiCabacRangeLps[kiState][ (uiRange&0xff)>>6];
		uiRange-=uiRangeLps;
		m_sStateCtx[iCtx].Set(g_kuiStateTransTable[kiState][0],m_sStateCtx[iCtx].Mps()^(kiState==0));
		CabacEncodeUpdateLow_();
		m_uiLow+=uiRange;
		const int32_t kiRenormAmount=g_kiClz5Table[uiRangeLps>>3];
		m_uiRange=uiRangeLps<<kiRenormAmount;
		m_iRenormCnt=kiRenormAmount;
	}
}

uint8_t* SCabacCtxEnc::WriteSliceEndSyn() {
	CabacEncodeTerminate(1);
	uint64_t uiLow=m_uiLow;
	int32_t iLowBitCnt=m_iLowBitCnt;
	uint8_t* pBufCur=m_pBufCur;
	uiLow <<=CABAC_LOW_WIDTH-1-iLowBitCnt;
	if(uiLow&uint64_t(1)<<(CABAC_LOW_WIDTH-1))
		PropagateCarry(pBufCur,m_pBufStart);
	for(;(iLowBitCnt-=8)>=0;uiLow<<=8)
		*pBufCur++=(uint8_t)(uiLow>>(CABAC_LOW_WIDTH-9));
	m_pBufCur=pBufCur;
	return pBufCur;
}

void SCabacCtxEnc::CabacEncodeTerminate(uint32_t uiBin) {
	m_uiRange-=2;
	if(uiBin) {
		CabacEncodeUpdateLow_();
		m_uiLow+=m_uiRange;
		const int32_t kiRenormAmount=7;
		m_uiRange=2<<kiRenormAmount;
		m_iRenormCnt=kiRenormAmount;
		CabacEncodeUpdateLow_();
		m_uiLow|=0x80;
	}else{
		const int32_t kiRenormAmount=m_uiRange>>8 ^ 1;
		m_uiRange=m_uiRange<<kiRenormAmount;
		m_iRenormCnt+=kiRenormAmount;
	}
}

void SCabacCtxEnc::MbSkipCabac(MBWindow* mbWindow,ESliceType eSliceType,int16_t bSkipFlag) {
	int32_t iCtx=(eSliceType==P_SLICE) ? 11 : 24;
	if(mbWindow->HasNeightbor(N_LEFT)) { //LEFT MB
		if(!IS_SKIP(mbWindow->GetNeighbor(N_LEFT)->uiMbType))
			iCtx++;
	}
	if(mbWindow->HasNeightbor(N_TOP)) { //TOP MB
		if(!IS_SKIP(mbWindow->GetNeighbor(N_TOP)->uiMbType))
			iCtx++;
	}
	CabacEncodeDecision(iCtx,bSkipFlag);
	if(bSkipFlag) {
		for(int i=0;i<16;i++) {
			mbWindow->m_mb->m_mvd[i].iMvX=0;
			mbWindow->m_mb->m_mvd[i].iMvY=0;
		}
		mbWindow->m_mb->uiCbp=mbWindow->m_mb->iCbpDc=0;
	}
}

void SCabacCtxEnc::CabacMbType(const MBWindow& mbWindow,const SMbCache& pMbCache,ESliceType eSliceType) {
	if(eSliceType==I_SLICE) {
		int32_t iCtx=3;
		if((mbWindow.HasNeightbor(N_LEFT)))
			iCtx++;
		if((mbWindow.HasNeightbor(N_TOP)))
			iCtx++;
		int32_t iCbpChroma=mbWindow.m_mb->uiCbp>>4;
		int32_t iCbpLuma=mbWindow.m_mb->uiCbp&15;
		int32_t iPredMode=g_kiMapModeI16x16[pMbCache.uiLumaI16x16Mode];
		CabacEncodeDecision(iCtx,1);
		CabacEncodeTerminate(0);
		if(iCbpLuma)
			CabacEncodeDecision(6,1);
		else
			CabacEncodeDecision(6,0);
		if(iCbpChroma==0)
			CabacEncodeDecision(7,0);
		else{
			CabacEncodeDecision(7,1);
			CabacEncodeDecision(8,iCbpChroma>>1);
		}
		CabacEncodeDecision(9,iPredMode>>1);
		CabacEncodeDecision(10,iPredMode&1);
	}else
	if(eSliceType==P_SLICE) {
		uint32_t uiMbType=mbWindow.m_mb->uiMbType;
		if(uiMbType==MB_TYPE_16x16) {
			CabacEncodeDecision(14,0);
			CabacEncodeDecision(15,0);
			CabacEncodeDecision(16,0);
		}else
		if((uiMbType==MB_TYPE_16x8) || (uiMbType==MB_TYPE_8x16)) {
			CabacEncodeDecision(14,0);
			CabacEncodeDecision(15,1);
			CabacEncodeDecision(17,mbWindow.m_mb->uiMbType==MB_TYPE_16x8);
		}else
		if((uiMbType==MB_TYPE_8x8)) {
			CabacEncodeDecision(14,0);
			CabacEncodeDecision(15,0);
			CabacEncodeDecision(16,1);
		}else{
			int32_t iCbpChroma=mbWindow.m_mb->uiCbp>>4;
			int32_t iCbpLuma=mbWindow.m_mb->uiCbp&15;
			int32_t iPredMode=g_kiMapModeI16x16[pMbCache.uiLumaI16x16Mode];
		//prefix
			CabacEncodeDecision(14,1);
		//suffix
			CabacEncodeDecision(17,1);
			CabacEncodeTerminate(0);
			if(iCbpLuma)
				CabacEncodeDecision(18,1);
			else
				CabacEncodeDecision(18,0);
			if(iCbpChroma==0)
				CabacEncodeDecision(19,0);
			else {
				CabacEncodeDecision(19,1);
				CabacEncodeDecision(19,iCbpChroma>>1);
			}
			CabacEncodeDecision(20,iPredMode>>1);
			CabacEncodeDecision(20,iPredMode&1);
		}
	}
}

void SCabacCtxEnc::CabacMbIntraChromaPredMode(const MBWindow& mbWindow,uint8_t uiChmaI8x8Mode) {
	int32_t iPredMode=g_kiMapModeIntraChroma[uiChmaI8x8Mode];
	int32_t iCtx=64;
	if((mbWindow.HasNeightbor(N_LEFT)) && g_kiMapModeIntraChroma[mbWindow.GetNeighbor(N_LEFT)->uiChromPredMode]!=0)
		iCtx++;
	if((mbWindow.HasNeightbor(N_TOP)) && g_kiMapModeIntraChroma[mbWindow.GetNeighbor(N_TOP)->uiChromPredMode]!=0)
		iCtx++;

	if(iPredMode==0) {
		CabacEncodeDecision(iCtx,0);
	}else
	if(iPredMode==1) {
		CabacEncodeDecision(iCtx,1);
		CabacEncodeDecision(67,0);
	}else
	if(iPredMode==2) {
		CabacEncodeDecision(iCtx,1);
		CabacEncodeDecision(67,1);
		CabacEncodeDecision(67,0);
	}else{
		CabacEncodeDecision(iCtx,1);
		CabacEncodeDecision(67,1);
		CabacEncodeDecision(67,1);
	}
}

void SCabacCtxEnc::CabacMbRef(const SMbCache& pMbCache,int16_t iIdx) {
	const SMVComponentUnit* pMvComp=&pMbCache.sMvComponents;
	const int16_t iRefIdxA=pMvComp->iRefIndexCache[iIdx+6];
	const int16_t iRefIdxB=pMvComp->iRefIndexCache[iIdx+1];
	int16_t iRefIdx=pMvComp->iRefIndexCache[iIdx+7];
	int16_t iCtx=0;
	if((iRefIdxA>0) && (!pMbCache.m_mbTypeSkip[3]))
		iCtx++;
	if((iRefIdxB>0) && (!pMbCache.m_mbTypeSkip[1]))
		iCtx+=2;
	while(iRefIdx>0) {
		CabacEncodeDecision(54+iCtx,1);
		iCtx=(iCtx>>2)+4;
		iRefIdx--;
	}
	CabacEncodeDecision(54+iCtx,0);
}

void SCabacCtxEnc::CabacEncodeBypassOne(int32_t uiBin) {
	const uint32_t kuiBinBitmask=-uiBin;
	m_iRenormCnt++;
	CabacEncodeUpdateLow_();
	m_uiLow+=kuiBinBitmask&m_uiRange;
}

void SCabacCtxEnc::CabacEncodeUeBypass(int32_t iExpBits,uint32_t uiVal) {
	int32_t iSufS=uiVal;
	int32_t iStopLoop=0;
	int32_t k=iExpBits;
	do{
		if(iSufS>=(1<<k)) {
			CabacEncodeBypassOne(1);
			iSufS=iSufS-(1<<k);
			k++;
		}else{
			CabacEncodeBypassOne(0);
			while(k--)
				CabacEncodeBypassOne((iSufS>>k)&1);
			iStopLoop=1;
		}
	}while(!iStopLoop);
}

void SCabacCtxEnc::CabacMbMvdLx(int32_t sMvd,int32_t iCtx,int32_t iPredMvd) {
	const int32_t iAbsMvd=ABS(sMvd);
	int32_t iCtxInc=0;
	int32_t iPrefix=MIN(iAbsMvd,9);
	int32_t i=0;
	if(iPredMvd>32)
		iCtxInc+=2;
	else
	if(iPredMvd>2)
		iCtxInc+=1;
	if(iPrefix) {
		if(iPrefix<9) {
			CabacEncodeDecision(iCtx+iCtxInc,1);
			iCtxInc=3;
			for(i=0;i<iPrefix-1;i++) {
			CabacEncodeDecision(iCtx+iCtxInc,1);
			if(i<3)
				iCtxInc++;
			}
			CabacEncodeDecision(iCtx+iCtxInc,0);
			CabacEncodeBypassOne (sMvd<0);
		}else{
			CabacEncodeDecision(iCtx+iCtxInc,1);
			iCtxInc=3;
			for(i=0;i<(9-1);i++) {
			CabacEncodeDecision(iCtx+iCtxInc,1);
			if(i<3)
				iCtxInc++;
			}
			CabacEncodeUeBypass(3,iAbsMvd-9);
			CabacEncodeBypassOne(sMvd<0);
		}
	}else{
		CabacEncodeDecision(iCtx+iCtxInc,0);
	}
}

SMVUnitXY SCabacCtxEnc::CabacMbMvd(const MBWindow& mbWindow,SMVUnitXY sCurMv,SMVUnitXY sPredMv,int16_t i4x4ScanIdx) {
	uint32_t iAbsMvd0,iAbsMvd1;
	SMVUnitXY sMvd;
	SMVUnitXY sMvdLeft;
	SMVUnitXY sMvdTop;
	sMvdLeft.iMvX=sMvdLeft.iMvY=sMvdTop.iMvX=sMvdTop.iMvY=0;
	sMvd.sDeltaMv(sCurMv,sPredMv);
	if((i4x4ScanIdx<4) && mbWindow.HasNeightbor(N_TOP)) { //top row blocks
		sMvdTop.sAssignMv(mbWindow.GetNeighbor(N_TOP)->m_mvd[i4x4ScanIdx+12]);
	}else
	if(i4x4ScanIdx>=4) {
		sMvdTop.sAssignMv(mbWindow.m_mb->m_mvd[i4x4ScanIdx-4]);
	}
	if((!(i4x4ScanIdx&0x03)) && mbWindow.HasNeightbor(N_LEFT)) { //left column blocks
		sMvdLeft.sAssignMv(mbWindow.GetNeighbor(N_LEFT)->m_mvd[i4x4ScanIdx+3]);
	}else
	if(i4x4ScanIdx&0x03) {
		sMvdLeft.sAssignMv(mbWindow.m_mb->m_mvd[i4x4ScanIdx-1]);
	}
	iAbsMvd0=ABS(sMvdLeft.iMvX)+ABS(sMvdTop.iMvX);
	iAbsMvd1=ABS(sMvdLeft.iMvY)+ABS(sMvdTop.iMvY);
	CabacMbMvdLx(sMvd.iMvX,40,iAbsMvd0);
	CabacMbMvdLx(sMvd.iMvY,47,iAbsMvd1);
	return sMvd;
}

void SCabacCtxEnc::CabacSubMbType(MBWindow* pCurMb) {
	for(int32_t i8x8Idx=0;i8x8Idx<4;++i8x8Idx) {
		CabacEncodeDecision(21,1);
	}
}

void SCabacCtxEnc::CabacSubMbMvd(MBWindow* mbWindow,const SMbCache& pMbCache) {
	SMVUnitXY sMvd;
	int32_t i8x8Idx,i4x4ScanIdx;
	for(i8x8Idx=0;i8x8Idx<4;++i8x8Idx) {
		i4x4ScanIdx=g_kuiMbCountScan4Idx[i8x8Idx<<2];
		sMvd=CabacMbMvd(*mbWindow,mbWindow->m_mb->m_mv[i4x4ScanIdx],pMbCache.sMbMvp[i4x4ScanIdx],i4x4ScanIdx);
		mbWindow->m_mb->m_mvd[  i4x4ScanIdx].sAssignMv(sMvd);
		mbWindow->m_mb->m_mvd[1+i4x4ScanIdx].sAssignMv(sMvd);
		mbWindow->m_mb->m_mvd[4+i4x4ScanIdx].sAssignMv(sMvd);
		mbWindow->m_mb->m_mvd[5+i4x4ScanIdx].sAssignMv(sMvd);
	}
}

void SCabacCtxEnc::CabacMbCbp(const MBWindow& mbWindow) {
	int32_t iCbpBlockLuma[4]={ (mbWindow.m_mb->uiCbp)&1,(mbWindow.m_mb->uiCbp>>1)&1,(mbWindow.m_mb->uiCbp>>2)&1,(mbWindow.m_mb->uiCbp>>3)&1};
	int32_t iCbpChroma=mbWindow.m_mb->uiCbp>>4;
	int32_t iCbpBlockLeft[4]={0,0,0,0};
	int32_t iCbpBlockTop[4]={0,0,0,0};
	int32_t iCbpLeftChroma=0;
	int32_t iCbpTopChroma=0;
	int32_t iCbp=0;
	int32_t iCtx=0;
	if(mbWindow.HasNeightbor(N_LEFT)) {
		iCbp=mbWindow.GetNeighbor(N_LEFT)->uiCbp;
		iCbpBlockLeft[0]=!(iCbp&1);
		iCbpBlockLeft[1]=!((iCbp>>1)&1);
		iCbpBlockLeft[2]=!((iCbp>>2)&1);
		iCbpBlockLeft[3]=!((iCbp>>3)&1);
		iCbpLeftChroma=iCbp>>4;
		if(iCbpLeftChroma)
			iCtx+=1;
	}
	if(mbWindow.HasNeightbor(N_TOP)) {
		iCbp=mbWindow.GetNeighbor(N_TOP)->uiCbp;
		iCbpBlockTop[0]=!(iCbp&1);
		iCbpBlockTop[1]=!((iCbp>>1)&1);
		iCbpBlockTop[2]=!((iCbp>>2)&1);
		iCbpBlockTop[3]=!((iCbp>>3)&1);
		iCbpTopChroma=iCbp>>4;
		if(iCbpTopChroma)
			iCtx+=2;
	}
	CabacEncodeDecision(73+iCbpBlockLeft[1]+iCbpBlockTop[2]*2,iCbpBlockLuma[0]);
	CabacEncodeDecision(73+!iCbpBlockLuma[0]+iCbpBlockTop[3]*2,iCbpBlockLuma[1]);
	CabacEncodeDecision(73+iCbpBlockLeft[3]+(!iCbpBlockLuma[0])*2 ,iCbpBlockLuma[2]);
	CabacEncodeDecision(73+!iCbpBlockLuma[2]+(!iCbpBlockLuma[1])*2,iCbpBlockLuma[3]);
	//chroma
	if(iCbpChroma) {
		CabacEncodeDecision(77+iCtx,1);
		CabacEncodeDecision(81+(iCbpLeftChroma>>1)+((iCbpTopChroma>>1)*2),iCbpChroma>1);
	}else{
		CabacEncodeDecision(77+iCtx,0);
	}
}

void SCabacCtxEnc::CabacMbDeltaQp(MBWindow* mbWindow) {
	int32_t iCtx=0;
	if(mbWindow->HasNeightbor(N_LEFT)) {			//was pCurMb->iMbXY
		const MacroBlock* pPrevMb=mbWindow->GetNeighbor(N_LEFT);
		mbWindow->m_mb->iLumaDQp=mbWindow->m_mb->uiLumaQp-pPrevMb->uiLumaQp;
		if(IS_SKIP(pPrevMb->uiMbType) || ((pPrevMb->uiMbType!=MB_TYPE_INTRA16x16) && (!pPrevMb->uiCbp)) || (!pPrevMb->iLumaDQp))
			iCtx=0;
		else
			iCtx=1;
	}
	if(mbWindow->m_mb->iLumaDQp) {
		int32_t iValue=mbWindow->m_mb->iLumaDQp<0 ? (-2*mbWindow->m_mb->iLumaDQp) : (2*mbWindow->m_mb->iLumaDQp-1);
		CabacEncodeDecision(60+iCtx,1);
		if(iValue==1) {
			CabacEncodeDecision(60+2,0);
		}else{
			CabacEncodeDecision(60+2,1);
			iValue--;
			while((--iValue)>0)
			CabacEncodeDecision(60+3,1);
			CabacEncodeDecision(60+3,0);
		}
	}else{
		CabacEncodeDecision(60+iCtx,0);
	}
}

static const uint16_t uiSignificantCoeffFlagOffset[5]={0,15,29,44,47};
static const uint16_t uiLastCoeffFlagOffset[5]={0,15,29,44,47};
static const uint16_t uiCoeffAbsLevelMinus1Offset[5]={0,10,20,30,39};
static const uint16_t uiCodecBlockFlagOffset[5]={0,4,8,12,16};

int16_t SCabacCtxEnc::GetMbCtxCabac(const SMbCache& pMbCache,const MBWindow& mbWindow,ECtxBlockCat eCtxBlockCat,int16_t iIdx) {
	int16_t iNzA=-1,iNzB=-1;
	const int8_t* pNonZeroCoeffCount=pMbCache.iNonZeroCoeffCount;
	int32_t bIntra=IS_INTRA(mbWindow.m_mb->uiMbType);
	int32_t iCtxInc=0;
	switch(eCtxBlockCat) {
		case LUMA_AC:
		case CHROMA_AC:
		case LUMA_4x4:
			iNzA=pNonZeroCoeffCount[iIdx-1];
			iNzB=pNonZeroCoeffCount[iIdx-8];
			break;
		case LUMA_DC:
		case CHROMA_DC:
			if(mbWindow.HasNeightbor(N_LEFT))
				iNzA=mbWindow.GetNeighbor(N_LEFT)->iCbpDc&(1<<iIdx);
			if(mbWindow.HasNeightbor(N_TOP))
				iNzB=mbWindow.GetNeighbor(N_TOP)->iCbpDc&(1<<iIdx);
			break;
		default:
			break;
	}
	if(((iNzA==-1) && bIntra) || (iNzA>0))
		iCtxInc+=1;
	if(((iNzB==-1) && bIntra) || (iNzB>0))
		iCtxInc+=2;
	return 85+uiCodecBlockFlagOffset[eCtxBlockCat]+iCtxInc;
}

void SCabacCtxEnc::WriteBlockResidualCabac(const SMbCache& pMbCache,const MBWindow& mbWindow,ECtxBlockCat eCtxBlockCat,int16_t  iIdx,int16_t iNonZeroCount,const int16_t* pBlock,int16_t iEndIdx) {
	int32_t iCtx=GetMbCtxCabac(pMbCache,mbWindow,eCtxBlockCat,iIdx);
	if(iNonZeroCount) {
		int16_t iLevel[16];
		const int32_t iCtxSig=105+uiSignificantCoeffFlagOffset[eCtxBlockCat];
		const int32_t iCtxLast=166+uiLastCoeffFlagOffset[eCtxBlockCat];
		const int32_t iCtxLevel=227+uiCoeffAbsLevelMinus1Offset[eCtxBlockCat];
		int32_t iNonZeroIdx=0;
		int32_t i=0;
		CabacEncodeDecision(iCtx,1);
		while(1) {
			if(pBlock[i]) {
				iLevel[iNonZeroIdx]=pBlock[i];
				iNonZeroIdx++;
				CabacEncodeDecision(iCtxSig+i,1);
				if(iNonZeroIdx!=iNonZeroCount)
					CabacEncodeDecision(iCtxLast+i,0);
				else {
					CabacEncodeDecision(iCtxLast+i,1);
					break;
				}
			}else
				CabacEncodeDecision(iCtxSig+i,0);
			i++;
			if(i==iEndIdx) {
				iLevel[iNonZeroIdx]=pBlock[i];
				iNonZeroIdx++;
				break;
			}
		}
		int32_t iNumAbsLevelGt1=0;
		int32_t iCtx1=iCtxLevel+1;
		do{
			int32_t iPrefix=0;
			iNonZeroIdx--;
			iPrefix=ABS(iLevel[iNonZeroIdx])-1;
			if(iPrefix) {
				iPrefix=MIN(iPrefix,14);
				iCtx=MIN(iCtxLevel+4,iCtx1);
				CabacEncodeDecision(iCtx,1);
				iNumAbsLevelGt1++;
				iCtx=iCtxLevel+4+MIN(5-(eCtxBlockCat==CHROMA_DC),iNumAbsLevelGt1);
				for(i=1;i<iPrefix;i++)
					CabacEncodeDecision(iCtx,1);
				if(ABS(iLevel[iNonZeroIdx])<15)
					CabacEncodeDecision(iCtx,0);
				else
					CabacEncodeUeBypass(0,ABS(iLevel[iNonZeroIdx])-15);
				iCtx1=iCtxLevel;
			}else{
				iCtx=MIN(iCtxLevel+4,iCtx1);
				CabacEncodeDecision(iCtx,0);
				iCtx1+=iNumAbsLevelGt1==0;
			}
			CabacEncodeBypassOne (iLevel[iNonZeroIdx]<0);
		}while(iNonZeroIdx>0);
	}else{
		CabacEncodeDecision(iCtx,0);
	}
}
void SCabacCtxEnc::WriteMbResidualCabac(const SMbCache& pMbCache,MBWindow* mbWindow,uint8_t* puiLastMbQp) {
	const uint16_t uiMbType=mbWindow->m_mb->uiMbType;
	int16_t i=0;
	const int8_t* pNonZeroCoeffCount=pMbCache.iNonZeroCoeffCount;
	mbWindow->m_mb->iCbpDc=0;
	mbWindow->m_mb->iLumaDQp=0;
	if((mbWindow->m_mb->uiCbp>0) || (uiMbType==MB_TYPE_INTRA16x16)) {
		int32_t iCbpChroma=mbWindow->m_mb->uiCbp>>4;
		int32_t iCbpLuma=mbWindow->m_mb->uiCbp&15;
		mbWindow->m_mb->iLumaDQp=mbWindow->m_mb->uiLumaQp-*puiLastMbQp;
		CabacMbDeltaQp(mbWindow);
		*puiLastMbQp=mbWindow->m_mb->uiLumaQp;
		if(uiMbType==MB_TYPE_INTRA16x16) {
		//Luma DC
			int iNonZeroCount=WelsGetNoneZeroCount_c(pMbCache.m_dct.iLumaI16x16Dc);
			WriteBlockResidualCabac(pMbCache,*mbWindow,LUMA_DC,0,iNonZeroCount,pMbCache.m_dct.iLumaI16x16Dc,15);
			if(iNonZeroCount)
			mbWindow->m_mb->iCbpDc|=1;
		//Luma AC
			if(iCbpLuma) {
				for(i=0;i<16;i++) {
					int32_t iIdx=g_kuiCache48CountScan4Idx[i];
					WriteBlockResidualCabac(pMbCache,*mbWindow,LUMA_AC,iIdx,pNonZeroCoeffCount[iIdx],pMbCache.m_dct.iLumaBlock[i],14);
				}
			}
		}else{
		//Luma AC
			for(i=0;i<16;i++) {
				if(iCbpLuma&(1<<(i>>2))) {
					int32_t iIdx=g_kuiCache48CountScan4Idx[i];
					WriteBlockResidualCabac(pMbCache,*mbWindow,LUMA_4x4,iIdx,pNonZeroCoeffCount[iIdx],pMbCache.m_dct.iLumaBlock[i],15);
				}
			}
		}
		if(iCbpChroma) {
			int32_t iNonZeroCount=0;
		//chroma DC
			iNonZeroCount=WelsCalNonZeroCount2x2Block(pMbCache.m_dct.iChromaDc[0]);
			if(iNonZeroCount)
			mbWindow->m_mb->iCbpDc|=0x2;
			WriteBlockResidualCabac(pMbCache,*mbWindow,CHROMA_DC,1,iNonZeroCount,pMbCache.m_dct.iChromaDc[0],3);
			iNonZeroCount=WelsCalNonZeroCount2x2Block(pMbCache.m_dct.iChromaDc[1]);
			if(iNonZeroCount)
			mbWindow->m_mb->iCbpDc|=0x4;
			WriteBlockResidualCabac(pMbCache,*mbWindow,CHROMA_DC,2,iNonZeroCount,pMbCache.m_dct.iChromaDc[1],3);
			if(iCbpChroma&0x02) {
				const uint8_t* g_kuiCache48CountScan4Idx_16base=&g_kuiCache48CountScan4Idx[16];
			//Cb AC
				for(i=0;i<4;i++) {
					int32_t iIdx=g_kuiCache48CountScan4Idx_16base[i];
					WriteBlockResidualCabac(pMbCache,*mbWindow,CHROMA_AC,iIdx,pNonZeroCoeffCount[iIdx],pMbCache.m_dct.iChromaBlock[i],14);
				}
			//Cr AC
				for(i=0;i<4;i++) {
					int32_t iIdx=24+g_kuiCache48CountScan4Idx_16base[i];
					WriteBlockResidualCabac(pMbCache,*mbWindow,CHROMA_AC,iIdx,pNonZeroCoeffCount[iIdx],pMbCache.m_dct.iChromaBlock[4+i],14);
				}
			}
		}
	}else{
		mbWindow->m_mb->iLumaDQp=0;
		mbWindow->m_mb->uiLumaQp=*puiLastMbQp;
	}
}

int32_t SCabacCtxEnc::SpatialWriteMbSynCabac(ESliceType eSliceType,MBWindow* mbWindow,uint8_t* puiLastMbQp,const SMbCache& pMbCache,uint8_t uiChmaI8x8Mode) {
	int32_t iBsPosSlice=(int32_t)((m_pBufCur-m_pBufStart)<<3)+(m_iLowBitCnt-9);
	const uint16_t uiMbType=mbWindow->m_mb->uiMbType;
	int16_t i=0;
	if(mbWindow->iMbXY>0)
		CabacEncodeTerminate(0);
	if(IS_SKIP(uiMbType)) {
		mbWindow->m_mb->uiLumaQp=*puiLastMbQp;
		MbSkipCabac(mbWindow,eSliceType,1);
	}else{
		if(eSliceType!=I_SLICE)
			MbSkipCabac(mbWindow,eSliceType,0);
		SMVUnitXY sMvd;
		CabacMbType(*mbWindow,pMbCache,eSliceType);
		if(IS_INTRA(uiMbType)) {
			CabacMbIntraChromaPredMode(*mbWindow,uiChmaI8x8Mode);
			sMvd.iMvX=sMvd.iMvY=0;
			for(i=0;i<16;++i) {
				mbWindow->m_mb->m_mvd[i].sAssignMv(sMvd);
			}
		}else
		if(uiMbType==MB_TYPE_16x16) {
			if(eSliceType==I_SLICE) {
				CabacMbRef(pMbCache,0);
			}
			sMvd=CabacMbMvd(*mbWindow,mbWindow->m_mb->m_mv[0],pMbCache.sMbMvp[0],0);
			for(i=0;i<16;++i) {
				mbWindow->m_mb->m_mvd[i].sAssignMv(sMvd);
			}
		}else
		if(uiMbType==MB_TYPE_16x8) {
			if(eSliceType==I_SLICE) {
				CabacMbRef(pMbCache,0);
				CabacMbRef(pMbCache,12);
			}
			sMvd=CabacMbMvd(*mbWindow,mbWindow->m_mb->m_mv[0],pMbCache.sMbMvp[0],0);
			for(i=0;i<8;++i) {
				mbWindow->m_mb->m_mvd[i].sAssignMv(sMvd);
			}
			sMvd=CabacMbMvd(*mbWindow,mbWindow->m_mb->m_mv[8],pMbCache.sMbMvp[1],8);
			for(i=8;i<16;++i) {
				mbWindow->m_mb->m_mvd[i].sAssignMv(sMvd);
			}
		}else
		if(uiMbType==MB_TYPE_8x16) {
			if(eSliceType==I_SLICE) {
				CabacMbRef(pMbCache,0);
				CabacMbRef(pMbCache,2);
			}
			sMvd=CabacMbMvd(*mbWindow,mbWindow->m_mb->m_mv[0],pMbCache.sMbMvp[0],0);
			for(i=0;i<16;i+=4) {
				mbWindow->m_mb->m_mvd[i].sAssignMv(sMvd);
				mbWindow->m_mb->m_mvd[i+1].sAssignMv(sMvd);
			}
			sMvd=CabacMbMvd(*mbWindow,mbWindow->m_mb->m_mv[2],pMbCache.sMbMvp[1],2);
			for(i=0;i<16;i+=4) {
				mbWindow->m_mb->m_mvd[i+2].sAssignMv(sMvd);
				mbWindow->m_mb->m_mvd[i+3].sAssignMv(sMvd);
			}
		}else
		if(uiMbType==MB_TYPE_8x8) {
		//write sub_mb_type
			CabacSubMbType(mbWindow);
			if(eSliceType==I_SLICE) {
				CabacMbRef(pMbCache,0);
				CabacMbRef(pMbCache,2);
				CabacMbRef(pMbCache,12);
				CabacMbRef(pMbCache,14);
			}
		//write sub8x8 mvd
			CabacSubMbMvd(mbWindow,pMbCache);
		}
		if(uiMbType!=MB_TYPE_INTRA16x16) {
			CabacMbCbp(*mbWindow);
		}
		WriteMbResidualCabac(pMbCache,mbWindow,puiLastMbQp);
	}
	if(!IS_INTRA(uiMbType))
		mbWindow->m_mb->uiChromPredMode=0;
	return (int32_t)((m_pBufCur-m_pBufStart)<<3)+(m_iLowBitCnt-9)-iBsPosSlice;
}

void SCabacCtxEnc::CabacEncodeUpdateLowNontrivial_() {
	int32_t iLowBitCnt=m_iLowBitCnt;
	int32_t iRenormCnt=m_iRenormCnt;
	uint64_t uiLow=m_uiLow;
	do{
		uint8_t* pBufCur=m_pBufCur;
		const int32_t kiInc=CABAC_LOW_WIDTH-1-iLowBitCnt;
		uiLow<<=kiInc;
		if(uiLow&uint64_t (1)<<(CABAC_LOW_WIDTH-1))
			PropagateCarry(pBufCur,m_pBufStart);
		if(CABAC_LOW_WIDTH>32) {
			WRITE_BE_32(pBufCur,(uint32_t)(uiLow>>31));
			pBufCur+=4;
		}
		*pBufCur++=(uint8_t)(uiLow>>23);
		*pBufCur++=(uint8_t)(uiLow>>15);
		iRenormCnt-=kiInc;
		iLowBitCnt=15;
		uiLow &=(1u<<iLowBitCnt)-1;
		m_pBufCur=pBufCur;
	}while(iLowBitCnt+iRenormCnt>CABAC_LOW_WIDTH-1);
	m_iLowBitCnt=iLowBitCnt+iRenormCnt;
	m_uiLow=uiLow<<iRenormCnt;
}

void SCabacCtxEnc::CabacEncodeUpdateLow_() {
	if(m_iLowBitCnt+m_iRenormCnt<CABAC_LOW_WIDTH) {
		m_iLowBitCnt+=m_iRenormCnt;
		m_uiLow<<=m_iRenormCnt;
	}else{
		CabacEncodeUpdateLowNontrivial_();
	}
	m_iRenormCnt=0;
}

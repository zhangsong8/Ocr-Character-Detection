#ifndef  NNIE_H
#define NNIE_H

#ifdef __cplusplus
extern "C"{
#endif /*__cplusplus*/

#include "SDKCommon.h"
#include "OSAL_type.h"

typedef enum hiDetection_Net_Type
{
	Detection_Net_YOLOV2      = 1,
    Detection_Net_SSD         = 2,  //*pLength/7 start objects numbers (0-0 1-class 2-confidence 3-6-pos)
    Detection_Net_FasterRCNN  = 3,  //the same with SSD(current support not more than 21 classes)
	Detection_Net_PVANet      = 4,  //the same with fasterRCNN
	Detection_Net_KeyPoint    = 5,  
	Detection_Net_Base        = 6,
	
	Detection_Net_Lane        = 7,  //detect lane
    Detection_Net_YOLOV3      = 8,
	Detection_Net_CenterNet      = 9,
	Detection_Net_CRNN      = 10

}Detection_Net_Type;


typedef struct hiDetection_Param_YOLOV2
{
    unsigned int u32BboxNumEachGrid;
    unsigned int u32ClassNum;
    unsigned int u32GridNumHeight;
    unsigned int u32GridNumWidth;
    unsigned int u32NmsThresh;
    unsigned int u32ConfThresh;
    unsigned int u32MaxRoiNum;
	int          bSameLowNMS;
}Detection_Param_YOLOV2;

typedef struct hiDetection_Param_YOLOV3
{
	unsigned int u32BboxNumEachGrid;
    unsigned int u32ClassNum;
    unsigned int au32GridNumHeight[3];
    unsigned int au32GridNumWidth[3];
    unsigned int u32NmsThresh;
    unsigned int u32ConfThresh;
    unsigned int u32MaxRoiNum;
}Detection_Param_YOLOV3;

typedef struct hiDetection_Param_CRNN
{

	//unsigned int u32OriImHeight;
    //unsigned int u32OriImWidth;
	unsigned int u32OutHeight;
	unsigned int u32OutWidth;
}Detection_Param_CRNN;


typedef struct hiDetection_Param_ResnetSSD
{
    /*----------------- Model Parameters ---------------*/
    unsigned int au32ConvHeight[12];
    unsigned int au32ConvWidth[12];
    unsigned int au32ConvChannel[12];
    /*----------------- PriorBox Parameters ---------------*/
    unsigned int au32PriorBoxWidth[6];
    unsigned int au32PriorBoxHeight[6];
    float af32PriorBoxMinSize[6][1];
    float af32PriorBoxMaxSize[6][1];
    unsigned int u32MinSizeNum;
    unsigned int u32MaxSizeNum[6];
    unsigned int u32OriImHeight;
    unsigned int u32OriImWidth;
    unsigned int au32InputAspectRatioNum[6];
    float af32PriorBoxAspectRatio[6][2];
    float af32PriorBoxStepWidth[6];
    float af32PriorBoxStepHeight[6];
    float f32Offset;
    int   bFlip;
    int   bClip;
    int as32PriorBoxVar[4];
    /*----------------- Softmax Parameters ---------------*/
    unsigned int au32SoftMaxInChn[6];
    unsigned int u32SoftMaxInHeight;
    unsigned int u32ConcatNum;
    unsigned int u32SoftMaxOutWidth;
    unsigned int u32SoftMaxOutHeight;
    unsigned int u32SoftMaxOutChn;
    /*----------------- DetectionOut Parameters ---------------*/
    unsigned int u32ClassNum;
    unsigned int u32TopK;
    unsigned int u32KeepTopK;
    unsigned int u32NmsThresh;
    unsigned int u32ConfThresh;
    unsigned int au32DetectInputChn[6];
    unsigned int au32ConvStride[6];
}Detection_Param_ResnetSSD;

typedef struct TagDetection_Param_Keypoint
{
	unsigned int u32OutnChannel;
	unsigned int u32OutHeight;
	unsigned int u32OutWidth;
	
}Detection_Param_Keypoint;

typedef struct hiDetection_Param_LaneNet
{
	unsigned int u32OutHeight;
	unsigned int u32OutWidth;
	unsigned int u32BinChannel;
	unsigned int u32SegChannel;
}Detection_Param_LaneNet;


typedef struct hiDetection_Param_FasterRCNN
{
	OSAL_UINT32 au32Scales[9];
	OSAL_UINT32 au32Ratios[9];
	OSAL_UINT32 au32ConvHeight[2];
	OSAL_UINT32 au32ConvWidth[2];
	OSAL_UINT32 au32ConvChannel[2];
	OSAL_UINT32 u32ConvStride;
	OSAL_UINT32 u32NumRatioAnchors;
	OSAL_UINT32 u32NumScaleAnchors;
	OSAL_UINT32 u32OriImHeight;
	OSAL_UINT32 u32OriImWidth;
	OSAL_UINT32 u32MinSize;
	OSAL_UINT32 u32SpatialScale;
	OSAL_UINT32 u32NmsThresh;
    OSAL_UINT32 u32FilterThresh;
    OSAL_UINT32 u32NumBeforeNms;
	OSAL_UINT32 u32MaxRoiNum;
	OSAL_UINT32 u32ClassNum;
	OSAL_UINT32 au32ConfThresh[21];
	OSAL_UINT32 u32ValidNmsThresh;
	OSAL_CHAR*  apcRpnDataLayerName[2];

}Detection_Param_FasterRCNN;

typedef struct hiClassify_Param_CNN
{
	OSAL_UINT32 u32TopN;
}Classify_Param_CNN;

#define MAX_NNIEOBJ_NUM 20

typedef struct TagNNIEObjResult
{
    OSAL_INT16   left;        //
    OSAL_INT16   right;       //
    OSAL_INT16   top;         //
    OSAL_INT16   bottom;      //
    OSAL_INT32   char_class;  //
    OSAL_FLOAT32 fConfidence; //
}TNNIEObjInfo;


typedef struct TagNNIEDetectResult
{
    TNNIEObjInfo  m_DetectObj[ MAX_NNIEOBJ_NUM ];
    OSAL_INT32 obj_count;
}NNIEDetectResult,*PNNIEDetectResult;

typedef struct TagCPointInof_{  
	unsigned int x_;  
	unsigned int y_;  
	unsigned int label_;
}CPointInof_;
#define EFFECTIVE_POINT 50
typedef struct TagCLineInfo{  
	CPointInof_ Fristpt_;  
	CPointInof_ Lastpt_;  
	CPointInof_ Centerpt_[EFFECTIVE_POINT];  
	unsigned char nEffectiveNum;  
	unsigned char nLaneMark;  
	unsigned short nPointCout;  
	float fSegmentLength;
}CLineInfo;

#define MAX_LANE_NUM 5
typedef struct TagNNIELaneResult{
	CLineInfo LaneArr[MAX_LANE_NUM];
	unsigned int iLaneNum;
}NNIELaneResult;

#define MAX_OCR_NUM 100
typedef struct TagNNIEOcrCrnnResult{
	int ocrs[MAX_OCR_NUM];
	unsigned int iOcrNum;
}NNIEOcrCrnnResult;



#define MAX_NNIELAYERLOUT_NUM 1000

typedef struct TagNNIELayerOutResult
{
	OSAL_FLOAT32 LayerOutArr[MAX_NNIELAYERLOUT_NUM];
	OSAL_INT32   iLength;
}NNIELayerOutResult,*PNNIELayerOutResult;

typedef struct TagKeyPointOutResult
{
	OSAL_FLOAT32 keypoints[MAX_NNIELAYERLOUT_NUM];
	OSAL_INT32   ikeyLength;

	OSAL_INT32   keyshapes[3];
}TKeyPointOutResult;

//init first and only once.
SDKErrCode NNIE_InitSystem();

//load diffierent model
SDKErrCode NNIE_NetInit(void** pNNIEHandle, char* pModePath, Detection_Net_Type ModeType, void* pParam, int iCoreID, int iMaxInputNum);

//mul step make predict
SDKErrCode NNIE_Detect_init(void* NNIEHandle,int iInputNum);
SDKErrCode NNIE_Detect_fillData(void* NNIEHandle, unsigned char *pImage);
SDKErrCode NNIE_Detect_Forword(void* NNIEHandle);
SDKErrCode NNIE_Detect_GetResult(void* NNIEHandle);
SDKErrCode NNIE_Detect_GetResultOut(void* NNIEHandle, unsigned int width, unsigned int height, NNIEDetectResult* pResult);
SDKErrCode NNIE_Detect_GetKeyPointOut(void* NNIEHandle, unsigned int width, unsigned int height, TKeyPointOutResult* pResult);
SDKErrCode NNIE_Detect_GetLaneNetOut(void* NNIEHandle, unsigned int width, unsigned int height, NNIELaneResult* pResult);
SDKErrCode NNIE_Detect_GetLayerOut(void* NNIEHandle, unsigned int width, unsigned int height, NNIELayerOutResult* pResult);
SDKErrCode NNIE_Detect_GetOcrCrnnOut(void* NNIEHandle,  NNIEOcrCrnnResult* pResult);

//one step make predict
SDKErrCode NNIE_Detect_Process(void* NNIEHandle, unsigned char *pImage, unsigned int width, unsigned int height, NNIEDetectResult* pResult);

//free model and other sources;
SDKErrCode NNIE_NetFree(void** pNNIEHandle);

//exit last and only once.
SDKErrCode NNIE_ExitSystem();

#ifdef __cplusplus
}
#endif /*__cplusplus*/


#endif//NNIE_H



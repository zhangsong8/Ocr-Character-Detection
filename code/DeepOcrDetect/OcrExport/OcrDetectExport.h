/****************************************************************************************
********
********   file name:   LaneDetectExport.h
********   description: export of lane detection
********   version:     V1.0
********   author:      flybird
********   time:        2019-7-29 20:29
********  
*****************************************************************************************/
#ifndef LANEDETECTIONEXPORT_H
#define LANEDETECTIONEXPORT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "OSAL_type.h"
#include "SDKCommon.h"
#include "HiImageType.h"
typedef struct TagPixPoint
{
    OSAL_FLOAT32 x; //
    OSAL_FLOAT32 y;
}PixPoint;

typedef struct TagDetectObjInfo
{
    OSAL_INT16   left;        //
    OSAL_INT16   right;       //
    OSAL_INT16   top;         //
    OSAL_INT16   bottom;      //
    OSAL_INT32   char_class;  //
    OSAL_FLOAT32 fConfidence; //
}TDetectObjInfo;

#define MAX_OBJ_RESULT_NUM 10
typedef struct TagObjectDetectResult
{
    TDetectObjInfo  m_DetectObj[ MAX_OBJ_RESULT_NUM ];
    OSAL_INT32 obj_count;
}ObjectDetectionResult,*PObjectDetectionResult;

typedef struct TagPersonName
{
    OSAL_CHAR Surname[40];   //持照人姓
    OSAL_CHAR name[40];   //持照人名
//    std::string Surname;   //护照类型
//    std::string name;
}PersonName;


#define MAX_OCR_NUM 100
typedef struct TagOutOcrResult{
//	std::string type;   //护照类型
//	std::string num;    //护照号码
//	std::string nation; //护照国籍
    PersonName full_name;   //持照人姓名
//	std::string sex;    //持照人性别
//	std::string birthData;//持照人出生日期
//	std::string validData;//护照有效日期
//	std::string machineData;//护照机器码
//    std::string ocrs_character[2];

    OSAL_CHAR type[10];
    OSAL_CHAR num[20];
    OSAL_CHAR nation[10];
    OSAL_CHAR sex[5];
    OSAL_CHAR birthData[10];
    OSAL_CHAR validData[10];
    OSAL_CHAR machineData[40];
    OSAL_CHAR ocrs_character[2][50];
    unsigned int iOcrNum[2];
}OutOcrResult,*POutOcrResult;



/**************************************************************
funcName  : LaneDetectionAlgorithmVersion
funcs	  : Get LaneDetection Alg version
Param In  :
Prame Out :
return    : point version arr
**************************************************************/
OSAL_PCHAR OcrDetectionAlgorithmVersion();

/**************************************************************
funcName  : LaneDetectionCreate
funcs	  : create LaneDetection Alg
Param In  : modelPath -- model path(detect line), modelPropertyPath -- model path(classify line), iAICode -- AI core ID
Prame Out : pHandle -- LD handle
return    : 0 -- success  other -- fail
**************************************************************/
SDKErrCode OcrDetectionCreate(OSAL_HANDLE* pHandle, OSAL_CHAR *modelDetectionPath, OSAL_CHAR *modelClassfyPath,OSAL_CHAR *labelPath, OSAL_INT32 image_width, OSAL_INT32 image_height,OSAL_INT32 iAICode);

/**************************************************************
funcName  : LaneDetectionFree
funcs	  : free LaneDetection Alg
Param In  : pHandle -- LD handle
Prame Out : 
return    : 0 -- success  other -- fail
**************************************************************/
SDKErrCode OcrDetectionFree(OSAL_HANDLE* pHandle);


/**************************************************************
funcName  : LaneDetectionProcessFrame
funcs	  : alg process
Param In  : pHandle -- LD handle, pImage -- input image(map), pFrame -- input image YVU420, image_width -- image width, image_height -- image height, enum_type -- image type
Prame Out : ppLaneDetectResult -- detect result
return    : 0 -- success  other -- fail
**************************************************************/
SDKErrCode OcrDetectionProcessFrame(OSAL_HANDLE Handle, OSAL_UCHAR* pImage,ALGORITHM_FRAME_INFO_S *pFrame, OSAL_INT32 image_width, OSAL_INT32 image_height, ImageDataType enum_type, POutOcrResult* ppLaneDetectResult);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LANEDETECTIONEXPORT_H */




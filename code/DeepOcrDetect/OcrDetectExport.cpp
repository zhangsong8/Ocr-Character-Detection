#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
//#include <opencv.hpp>

#include "OcrExport/OcrDetectExport.h"

//#define  WIN_DEBUG
//#define OPENCV
#define  HI_3559


#ifdef HI_3559
#include "NNIE/sample_comm_ive.h"
#include "NNIE/nnie.h"
#endif

#ifdef WIN_DEBUG
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
//#include <sys/time.h>

#endif
#include <sys/time.h>

using namespace std;

#define MAX_OCRTASK_NUM     24    //max support task numbers

#define  OCRDETION_VERSION  "PP-0002-V0.0.0.1-D"

static OSAL_INT32 g_iOCRHandleCount = 0;
static OSAL_CHAR   g_chOCRVersion[256];
static const OSAL_CHAR * OCR_BUILD_TIME = __DATE__", " __TIME__;
static std::vector<string> labels_;
//char *crnn_labels[39] = {"%","2","1","3","4","5","6","7","8","9","O","J","I","C","G","A","B","D","E","F","H","Q","0","S","K","L","M","N","P","R","T","Y","U","V","W","X","Z","<","-"};


//#define INPUT_NET_WIDTH  512
//#define INPUT_NET_HEIGHT 288
#define INPUT_NET_WIDTH  480
#define INPUT_NET_HEIGHT 480
#define CRNN_INPUT_NET_WIDTH  1216
//#define CRNN_INPUT_NET_WIDTH  1120
#define CRNN_INPUT_NET_HEIGHT 32




typedef struct TagTaskOcrDetect
{
	OSAL_HANDLE   m_OcrDetectHandle;
	OSAL_HANDLE   m_OcrClassfyHandle;
	//OSAL_INT32    m_LaneDetectSensity;
	OSAL_INT32    m_DetectCount;

	OSAL_PUCHAR   m_pInputYVU;
	OSAL_UCHAR    m_InputBGR[INPUT_NET_WIDTH*INPUT_NET_HEIGHT*3];
	//OSAL_UCHAR    m_InputBGR_CRNN[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT*3];
#ifdef HI_3559
	IVE_SRC_IMAGE_S    stYVU420SPSrc;      //input video frame 
	IVE_SRC_IMAGE_S    stIVEInputBGR;      //ive src image bgr planar
	IVE_DST_IMAGE_S    stIVEResize;        //ive resize image(512x288) 
	IVE_RESIZE_CTRL_S  stResizeCtrl;       //IVE resize parameters
	IVE_CSC_CTRL_S     stCSCCtrl;                        //IVE Color space conversion parameters
#endif	
	OutOcrResult m_DetectResult;

}TTaskOcrDetect,*PTTaskOcrDetect;


//////////////////////////////private function////////////////////////////////////

#ifdef  HI_3559
static OSAL_INT32 IVE_FrameFlushCache(IVE_SRC_IMAGE_S *pIVEFrame);
static OSAL_INT32 IVE_FrameYVUFlushCache(IVE_SRC_IMAGE_S *pIVEFrame);
static void SplitImageToIVEFrame(IVE_DST_IMAGE_S *pIVEFrame,OSAL_UCHAR *pImage,OSAL_INT32 srcImgW, OSAL_INT32 srcImgH);

#endif

/////////////////////////////////////////////////////////////////////////////////

double getCurrentTime()
{
   struct timeval tv;
   gettimeofday(&tv,NULL);
   return tv.tv_sec*1000 + tv.tv_usec/1000.0;
}

/**************************************************************
funcName  : LaneDetectionAlgorithmVersion
funcs	  : Get LaneDetection Alg version
Param In  :
Prame Out :
return    : point version arr
**************************************************************/
OSAL_PCHAR OcrDetectionAlgorithmVersion()
{
	memset(g_chOCRVersion,0,256);

	sprintf(g_chOCRVersion, "%s build on %s", OCRDETION_VERSION,  OCR_BUILD_TIME);

	return g_chOCRVersion;
}

/**************************************************************
funcName  : LaneDetectionCreate
funcs	  : create LaneDetection Alg
Param In  : modelPath -- model path(detect line), modelPropertyPath -- model path(classify line), iAICode -- AI core ID
Prame Out : pHandle -- LD handle
return    : 0 -- success  other -- fail
**************************************************************/
SDKErrCode OcrDetectionCreate(OSAL_HANDLE* pHandle, OSAL_CHAR *modelDetectionPath, OSAL_CHAR *modelClassfyPath,OSAL_CHAR *labelPath,OSAL_INT32 image_width, OSAL_INT32 image_height,OSAL_INT32 iAICode)
{
	TTaskOcrDetect*  pTask = NULL;
	SDKErrCode eRet = SDK_ERR_NONE;
	OSAL_INT32 s32Ret = 0;
	std::string labelPath_ = labelPath;

	/* Load labels. */
    if (labelPath_.size() > 0)
    {
        //std::ifstream labels(labelPath_.c_str());
        std::ifstream labels(labelPath_);
        //CHECK(labels) << "Unable to open labels file " << labelPath_;
        string line;
        int label_count =0;
        while (std::getline(labels, line)){
                    //printf("%s\n",line.c_str());
                    labels_.push_back(std::string(line));
                    label_count++;
                }
        //printf("label_count:%d\n",label_count);
    }
    else
    {
        printf("Error(:),labelPath ls error!\n");
    }
/*
    bool bNNIESystemInit = true;
    if(bNNIESystemInit)
    {
        eRet = NNIE_InitSystem();
        if(eRet != SDK_ERR_NONE)
        {
            return eRet;
        }
        //pTask->bNNIESystemInit = OSAL_TRUE;
        //bNNIESystemInit = OSAL_FALSE;
    }
    else
    {
        //pTask->bNNIESystemInit = OSAL_FALSE;
    }
*/
	if (pHandle == OSAL_NULL || modelDetectionPath == OSAL_NULL)
	{
		return SDK_ERR_PARAM;
	}
	if (g_iOCRHandleCount < MAX_OCRTASK_NUM)
	{
        pTask = new TTaskOcrDetect;
        //pTask->m_DetectResult = {0};
        if (pTask)
        {
            NNIE_NetInit(&pTask->m_OcrDetectHandle, modelDetectionPath , Detection_Net_CenterNet, OSAL_NULL, iAICode, 1);

            if(modelClassfyPath)
            {
                Detection_Param_CRNN  ocrNetParam;
                ocrNetParam.u32OutHeight  = 39;
                ocrNetParam.u32OutWidth   = 305;
//                ocrNetParam.u32OutHeight  = 5990;
//                ocrNetParam.u32OutWidth   = 140;
                NNIE_NetInit(&pTask->m_OcrClassfyHandle, modelClassfyPath, Detection_Net_CRNN, &ocrNetParam, iAICode, 1);
            }
#ifdef HI_3559
            if(1)//if (image_width <= 1920 && image_height <= 1080)
			{
				//input YVU420SP ---> BBB...GGG...RRR...
				s32Ret = SAMPLE_COMM_IVE_CreateImageByCached(&(pTask->stYVU420SPSrc), IVE_IMAGE_TYPE_YUV420SP, image_width, image_height);
				if(HI_SUCCESS != s32Ret)
				{
					printf("Error(%#x),Create stYVU420SPSrc image failed!\n", s32Ret);
				}
//                printf("***%s****%d***%d****\n",__FUNCTION__,image_width,image_height,__LINE__);
				s32Ret = SAMPLE_COMM_IVE_CreateImageByCached(&(pTask->stIVEInputBGR), IVE_IMAGE_TYPE_U8C3_PLANAR, image_width, image_height);
				if(HI_SUCCESS != s32Ret)
				{
					printf("Error(%#x),Create stIVEInputBGR image failed!\n", s32Ret);
				}
//                printf("***%s****%d***%d****\n",__FUNCTION__,INPUT_NET_WIDTH,INPUT_NET_HEIGHT,__LINE__);
                s32Ret = SAMPLE_COMM_IVE_CreateImageByCached(&(pTask->stIVEResize), IVE_IMAGE_TYPE_U8C3_PLANAR, INPUT_NET_WIDTH, INPUT_NET_HEIGHT);
				if(HI_SUCCESS != s32Ret)
				{
					printf("Error(%#x),Create stIVEResize image failed!\n", s32Ret);
				}
				pTask->stCSCCtrl.enMode = IVE_CSC_MODE_VIDEO_BT601_YUV2RGB; //IVE_CSC_MODE_VIDEO_BT709_YUV2RGB; //; 

                pTask->stResizeCtrl.u16Num = 1;
                pTask->stResizeCtrl.stMem.u32Size = pTask->stResizeCtrl.u16Num * sizeof(IVE_IMAGE_FILL_U8C3_PLANAR_S) * 2;
                s32Ret = HI_MPI_SYS_MmzAlloc_Cached(&pTask->stResizeCtrl.stMem.u64PhyAddr,
                    (void**)&pTask->stResizeCtrl.stMem.u64VirAddr,
                    NULL,
                    NULL,
                    pTask->stResizeCtrl.stMem.u32Size);

                printf("\nstResizeCtrl stMem.u64PhyAddr:%p, stMem.u64VirAddr:%p\n", pTask->stResizeCtrl.stMem.u64PhyAddr, pTask->stResizeCtrl.stMem.u64VirAddr);

                pTask->stResizeCtrl.enMode = IVE_RESIZE_MODE_LINEAR; //IVE_RESIZE_MODE_AREA;
			}
#endif
			*pHandle = pTask;
		}
		else
		{
			eRet = SDK_ERR_ALLOC;
		}
	}
	else
	{
		eRet = SDK_ERR_FULLTASK;
	}

	return eRet;
}

/**************************************************************
funcName  : LaneDetectionFree
funcs	  : free LaneDetection Alg
Param In  : pHandle -- LD handle
Prame Out : 
return    : 0 -- success  other -- fail
**************************************************************/
SDKErrCode OcrDetectionFree(OSAL_HANDLE* pHandle)
{
	TTaskOcrDetect* pTask = OSAL_NULL;

	if (pHandle == OSAL_NULL||*pHandle == OSAL_NULL)
	{
		return SDK_ERR_PARAM;
	}
	pTask = (TTaskOcrDetect*)(*pHandle);

#ifdef HI_3559
    IVE_MMZ_FREE(pTask->stYVU420SPSrc.au64PhyAddr[0], pTask->stYVU420SPSrc.au64VirAddr[0]);
    IVE_MMZ_FREE(pTask->stResizeCtrl.stMem.u64PhyAddr, pTask->stResizeCtrl.stMem.u64VirAddr);

    IVE_MMZ_FREE(pTask->stIVEInputBGR.au64PhyAddr[0], pTask->stIVEInputBGR.au64VirAddr[0]);
    IVE_MMZ_FREE(pTask->stIVEInputBGR.au64PhyAddr[1], pTask->stIVEInputBGR.au64VirAddr[1]);
    IVE_MMZ_FREE(pTask->stIVEInputBGR.au64PhyAddr[2], pTask->stIVEInputBGR.au64VirAddr[2]);
	
    IVE_MMZ_FREE(pTask->stIVEResize.au64PhyAddr[0], pTask->stIVEResize.au64VirAddr[0]);
    IVE_MMZ_FREE(pTask->stIVEResize.au64PhyAddr[1], pTask->stIVEResize.au64VirAddr[1]);
    IVE_MMZ_FREE(pTask->stIVEResize.au64PhyAddr[2], pTask->stIVEResize.au64VirAddr[2]);
#endif
    if(OSAL_NULL != pTask->m_OcrDetectHandle)
    {
	NNIE_NetFree(&pTask->m_OcrDetectHandle);	
	pTask->m_OcrDetectHandle = OSAL_NULL;
    }
    if(OSAL_NULL != pTask->m_OcrClassfyHandle)
    {
	NNIE_NetFree(&pTask->m_OcrClassfyHandle);
	pTask->m_OcrClassfyHandle = OSAL_NULL;
    }
	delete pTask;  pTask = NULL;

	return SDK_ERR_NONE;
}

float pointLineDis(float p1x, float p1y, float p2x, float p2y,float px, float py){
    float a = p1y - p2y;
    float b = p2x - p1x;
    float c = p1x *p2y - p2x * p1y;
    return (a*px + b*py + c);
}

#ifdef OPENCV   //opencv
//锟斤拷锟斤拷5锟斤拷原始图copy一锟斤拷map锟斤拷锟斤拷锟斤拷锟斤拷
cv::Mat PPcopyDarwMap(cv::Mat orginImg,std::vector<float> centerXsNew,std::vector<float> centerYsNew,std::vector<int> matchResult)
{
    //锟斤拷锟斤拷全0图片
    int w = orginImg.size().width;
    int h = orginImg.size().height;
    cv::Mat dst = cv::Mat::zeros(cv::Size(w, h), CV_8UC1);
    //循锟斤拷锟斤拷锟斤拷
    for( int i = 0 ; i < centerXsNew.size(); i++)
    {
        float centerX1 = (centerXsNew[i]);
        float centerY1 = (centerYsNew[i]);
        ///float Rr1 = (anchorC1New[i][2]);
        float Rr1 = 12;
        //锟斤拷锟斤拷圆锟斤拷
        cv::circle(dst, cv::Point(centerX1,centerY1), Rr1, cv::Scalar(255), -1, 3);
    }
    //锟斤拷锟斤拷直锟斤拷
    cv::imwrite("./drawCircle.jpg",dst);
    cv::line(dst, cv::Point(centerXsNew[matchResult[0]],centerYsNew[matchResult[0]]), cv::Point(centerXsNew[matchResult[1]],centerYsNew[matchResult[1]]), cv::Scalar(255), 3, 8);  //锟斤拷锟斤拷锟斤拷小锟斤拷泳锟斤拷锟矫匡拷锟斤拷锟�1�7
    cv::line(dst, cv::Point(centerXsNew[matchResult[2]],centerYsNew[matchResult[2]]), cv::Point(centerXsNew[matchResult[3]],centerYsNew[matchResult[3]]), cv::Scalar(255), 3, 8);  //锟斤拷锟斤拷锟斤拷小锟斤拷泳锟斤拷锟矫匡拷锟斤拷锟�1�7
    cv::imwrite("./drawLine.jpg",dst);

    return dst;
}

void PPbigImg(cv::Mat orginImg,cv::Mat mapImg,cv::Mat& orginImgBig,cv::Mat& mapImgBig){
    //锟斤拷锟矫憋拷锟斤拷锟斤拷锟斤拷
    int w = orginImg.size().width;
    int h = orginImg.size().height;
    orginImgBig = cv::Mat::zeros(cv::Size(w*2, h*2), CV_8UC3);
    mapImgBig = cv::Mat::zeros(cv::Size(w*2, h*2), CV_8UC1);
    int w_Start1 = int(w / 2);
    int h_Start1 = int(h / 2);
    //锟斤拷锟矫伙拷锟斤拷锟斤拷锟斤拷锟斤拷锟津并革拷锟斤拷
    cv::Rect roi_rect1 = cv::Rect(w_Start1, h_Start1, orginImg.size().width, orginImg.size().height);
    cv::Rect roi_rect2 = cv::Rect(w_Start1, h_Start1, mapImg.size().width, mapImg.size().height);
    orginImg.copyTo(orginImgBig(roi_rect1));
    mapImg.copyTo(mapImgBig(roi_rect2));
    //imwrite("PPorginImgBig.jpg",orginImgBig);
    //imwrite("PPmapImgBig.jpg",mapImgBig);
    //imwrite("PPmapImg.jpg",mapImg);
}

cv::Mat PProtateImage(cv::Mat img, cv::RotatedRect rect){
    cv::Mat src = img.clone();
    float degree = rect.angle;
    float wh[2] = {rect.size.width,rect.size.height};
    //float wh[2] = {rect.size.width,38};
    float center[2] = {rect.center.x,rect.center.y};

    float wh_[2] = {wh[0],wh[1]};
    if(wh[0]<wh[1]){
        if(degree>0){
            degree = degree-90;}
        else{
            degree = degree+90;}
        wh_[0] = wh[1];
        wh_[1] = wh[0];
    }
    wh_[1] = 38;

    cv::Mat matRotation(2,3,CV_32FC1);
    matRotation = cv::getRotationMatrix2D(cv::Point(center[0],center[1]),degree,1);
    cv::warpAffine(src,src, matRotation,img.size());
    //    cv::imwrite("./affineImg.jpg",src);

    int x1 = center[0]-wh_[0]/2;
    int y1 = center[1]-wh_[1]/2;

    int rect_height = wh_[1];// < (src.size().width - x1) ? rect_height : (src.size().width - x1 - 1);
    int rect_width = wh_[0];// < (src.size().height - y1) ? rect_width : (src.size().height - y1 - 1);
    cv::Rect rect1=cv::Rect(int(x1), int(y1), int(rect_width), int(rect_height));
    cv::Mat seg_img = src(rect1);
    //cv::rectangle(src,rect1,(255,255,255));
    //cv::imwrite("rectangle.jpg",src);
    return seg_img;
}
#endif

/**************************************************************
funcName  : LaneDetectionProcessFrame
funcs	  : alg process
Param In  : pHandle -- LD handle, pImage -- input image, image_width -- image width, image_height -- image height, enum_type -- image type
Prame Out : ppLaneDetectResult -- detect result
return    : 0 -- success  other -- fail
**************************************************************/
SDKErrCode OcrDetectionProcessFrame(OSAL_HANDLE Handle, OSAL_UCHAR* pImage, ALGORITHM_FRAME_INFO_S *pFrame, OSAL_INT32 image_width, OSAL_INT32 image_height, ImageDataType enum_type, POutOcrResult* ppOcrDetectResult)
{
	TTaskOcrDetect* pTask = OSAL_NULL;
	OSAL_PUCHAR pInputImg = OSAL_NULL;
#ifdef HI_3559
	VIDEO_FRAME_INFO_S stFrameInfo;
	IVE_HANDLE ive_resize_handle;
	int s32Ret = 0;
	IVE_HANDLE ive_csc_handle;
	HI_BOOL bBlock  = HI_TRUE;
	HI_BOOL bFinish = HI_FALSE;
#endif
//    double centernet_prestart=getCurrentTime();
#ifdef WIN_DEBUG
	HI_CHAR savename[256] = {0};
#endif
    if (Handle == OSAL_NULL || (pImage == OSAL_NULL && pFrame == OSAL_NULL)|| ppOcrDetectResult == OSAL_NULL)
	{
        printf("error***%s*******%d****\n",__FUNCTION__,__LINE__);
		return SDK_ERR_PARAM;
	}
	pTask = (TTaskOcrDetect*)Handle;

#ifdef HI_3559
	if (pFrame)
	{
		stFrameInfo.u32PoolId = pFrame->u32PoolId;
		stFrameInfo.enModId   = (MOD_ID_E)pFrame->s32ModId;
		stFrameInfo.stVFrame.u32Width      = pFrame->stAlgFrame.u32Width;
		stFrameInfo.stVFrame.u32Height     = pFrame->stAlgFrame.u32Height;
		stFrameInfo.stVFrame.u32Stride[0]  = pFrame->stAlgFrame.u32Stride[0];
		stFrameInfo.stVFrame.u32Stride[1]  = pFrame->stAlgFrame.u32Stride[1];
		stFrameInfo.stVFrame.u32Stride[2]  = pFrame->stAlgFrame.u32Stride[2];
		stFrameInfo.stVFrame.u64PhyAddr[0] = pFrame->stAlgFrame.u64PhyAddr[0];
		stFrameInfo.stVFrame.u64PhyAddr[1] = pFrame->stAlgFrame.u64PhyAddr[1];
		stFrameInfo.stVFrame.u64PhyAddr[2] = pFrame->stAlgFrame.u64PhyAddr[2];
		stFrameInfo.stVFrame.u64VirAddr[0] = pFrame->stAlgFrame.u64VirAddr[0];
		stFrameInfo.stVFrame.u64VirAddr[1] = pFrame->stAlgFrame.u64VirAddr[1];
		stFrameInfo.stVFrame.u64VirAddr[2] = pFrame->stAlgFrame.u64VirAddr[2];
	}
#endif
	if (IMAGE_TYPE_BGR_PACKAGE == enum_type)
	{
        pTask->stIVEInputBGR.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
        pTask->stIVEInputBGR.au32Stride[0] = ((image_width+15)>>4)<<4;
        pTask->stIVEInputBGR.au32Stride[1] = ((image_width+15)>>4)<<4;
        pTask->stIVEInputBGR.au32Stride[2] = ((image_width+15)>>4)<<4;
        pTask->stIVEInputBGR.u32Width      = image_width;
        pTask->stIVEInputBGR.u32Height     = image_height;

        s32Ret = IVE_FrameFlushCache(&pTask->stIVEInputBGR);
        SplitImageToIVEFrame(&pTask->stIVEInputBGR, pImage, image_width, image_height);

        s32Ret = IVE_FrameFlushCache(&pTask->stIVEInputBGR);
        pTask->stIVEResize.u32Width  = INPUT_NET_WIDTH;
        pTask->stIVEResize.u32Height = INPUT_NET_HEIGHT;
        pTask->stIVEResize.au32Stride[0] = ((INPUT_NET_WIDTH+15)>>4)<<4;
        pTask->stIVEResize.au32Stride[1] = ((INPUT_NET_WIDTH+15)>>4)<<4;
        pTask->stIVEResize.au32Stride[2] = ((INPUT_NET_WIDTH+15)>>4)<<4;
		pTask->stIVEResize.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
		s32Ret = IVE_FrameFlushCache(&pTask->stIVEResize);
		s32Ret = HI_MPI_IVE_Resize(&ive_resize_handle, &pTask->stIVEInputBGR, &pTask->stIVEResize, &pTask->stResizeCtrl, HI_TRUE);
		if(HI_SUCCESS != s32Ret)
		{
			printf("Error(%#x),HI_MPI_IVE_Resize failed 1!\n",s32Ret);
		}

		s32Ret = HI_MPI_IVE_Query(ive_resize_handle, &bFinish, bBlock);
		while (HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
		{
			usleep(100);
			s32Ret = HI_MPI_IVE_Query(ive_resize_handle, &bFinish, bBlock);
		}		
		if(HI_SUCCESS != s32Ret)
		{
			printf("Error(%#x),HI_MPI_IVE_Query failed!\n",s32Ret);
		}
		//copy to buf
		for (int i = 0; i < INPUT_NET_HEIGHT; i++)
		{
			memcpy((void *)(pTask->m_InputBGR+ INPUT_NET_WIDTH * i), (void *)(pTask->stIVEResize.au64VirAddr[0] + pTask->stIVEResize.au32Stride[0] * i), INPUT_NET_WIDTH);
			memcpy((void *)(pTask->m_InputBGR + INPUT_NET_WIDTH*INPUT_NET_HEIGHT + INPUT_NET_WIDTH * i), (void *)(pTask->stIVEResize.au64VirAddr[1] + pTask->stIVEResize.au32Stride[1] * i), INPUT_NET_WIDTH);
			memcpy((void *)(pTask->m_InputBGR + 2*INPUT_NET_WIDTH*INPUT_NET_HEIGHT + INPUT_NET_WIDTH * i), (void *)(pTask->stIVEResize.au64VirAddr[2] + pTask->stIVEResize.au32Stride[2] * i), INPUT_NET_WIDTH);
		}		
	}
	else if (IMAGE_TYPE_YVU == enum_type)
	{
		pTask->m_pInputYVU = pImage;
		
#ifdef HI_3559
		if (pFrame == OSAL_NULL)//if (image_width > 1920 || image_height > 1080 || pFrame == OSAL_NULL)
		{//not use ive		
            printf("error***%s*******%d****\n",__FUNCTION__,__LINE__);
			return SDK_ERR_PARAM;
		}
		else
		{
			s32Ret = IVE_FrameFlushCache(&pTask->stIVEInputBGR);
			pTask->stYVU420SPSrc.enType = IVE_IMAGE_TYPE_YUV420SP;
			s32Ret = IVE_FrameYVUFlushCache(&pTask->stYVU420SPSrc);
			if(HI_SUCCESS != s32Ret)
			{
				printf("Error(%#x),IVE_FrameYVUFlushCache failed!\n",s32Ret);
			}

			s32Ret = SAMPLE_COMM_IVE_DmaImage_YVU420SP(&stFrameInfo, &pTask->stYVU420SPSrc, HI_TRUE);
			if(HI_SUCCESS != s32Ret)
			{
				printf("Error(%#x),HI_MPI_IVE_CSC failed!\n",s32Ret);
			}
			s32Ret = HI_MPI_IVE_CSC(&ive_csc_handle, &pTask->stYVU420SPSrc, &pTask->stIVEInputBGR, &pTask->stCSCCtrl, HI_TRUE);
			if(HI_SUCCESS != s32Ret)
			{
				printf("Error(%#x),HI_MPI_IVE_CSC failed!\n",s32Ret);
			}

			s32Ret = HI_MPI_IVE_Query(ive_csc_handle, &bFinish, bBlock);
			while (HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
			{
				usleep(100);
				s32Ret = HI_MPI_IVE_Query(ive_csc_handle, &bFinish, bBlock);
			}
			if(HI_SUCCESS != s32Ret)
			{
				printf("Error(%#x),HI_MPI_IVE_Query failed!\n",s32Ret);
			}
			pTask->stIVEResize.u32Width  = INPUT_NET_WIDTH;
			pTask->stIVEResize.u32Height = INPUT_NET_HEIGHT;
			pTask->stIVEResize.au32Stride[0] = ((INPUT_NET_WIDTH+15)>>4)<<4;
			pTask->stIVEResize.au32Stride[1] = ((INPUT_NET_WIDTH+15)>>4)<<4;
			pTask->stIVEResize.au32Stride[2] = ((INPUT_NET_WIDTH+15)>>4)<<4;
			pTask->stIVEResize.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
			s32Ret = IVE_FrameFlushCache(&pTask->stIVEResize);
			pTask->stIVEInputBGR.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
			s32Ret = IVE_FrameFlushCache(&pTask->stIVEInputBGR);
			s32Ret = HI_MPI_IVE_Resize(&ive_resize_handle, &pTask->stIVEInputBGR, &pTask->stIVEResize, &pTask->stResizeCtrl, HI_TRUE);	
			s32Ret = HI_MPI_IVE_Query(ive_resize_handle, &bFinish, bBlock);
			while (HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
			{
				usleep(100);
				s32Ret = HI_MPI_IVE_Query(ive_resize_handle, &bFinish, bBlock);
			}
			if(HI_SUCCESS != s32Ret)
			{
				printf("Error(%#x),HI_MPI_IVE_Query failed!\n",s32Ret);
			}
			//copy to buf
			for (int i = 0; i < INPUT_NET_HEIGHT; i++)
			{
				memcpy((void *)(pTask->m_InputBGR+ INPUT_NET_WIDTH * i), (void *)(pTask->stIVEResize.au64VirAddr[0] + pTask->stIVEResize.au32Stride[0] * i), INPUT_NET_WIDTH);
				memcpy((void *)(pTask->m_InputBGR + INPUT_NET_WIDTH*INPUT_NET_HEIGHT + INPUT_NET_WIDTH * i), (void *)(pTask->stIVEResize.au64VirAddr[1] + pTask->stIVEResize.au32Stride[1] * i), INPUT_NET_WIDTH);
				memcpy((void *)(pTask->m_InputBGR + 2*INPUT_NET_WIDTH*INPUT_NET_HEIGHT + INPUT_NET_WIDTH * i), (void *)(pTask->stIVEResize.au64VirAddr[2] + pTask->stIVEResize.au32Stride[2] * i), INPUT_NET_WIDTH);
			}	
		}
#else
        printf("error***%s*******%d****\n",__FUNCTION__,__LINE__);
		return SDK_ERR_PARAM;
#endif
	}
#ifdef WIN_DEBUG
	#if 0
		Mat dispImg(INPUT_NET_HEIGHT, INPUT_NET_WIDTH, CV_8UC3);
		for(int i = 0; i < pTask->stIVEResize.u32Height; i++)
		{
			for(int j = 0; j < pTask->stIVEResize.u32Width; j++)
			{
				dispImg.data[i * dispImg.step[0] + 3 * j]	  = *((HI_U8*)pTask->stIVEResize.au64VirAddr[0]+ i * pTask->stIVEResize.au32Stride[0] + j);
				dispImg.data[i * dispImg.step[0] + 3 * j + 1] = *((HI_U8*)pTask->stIVEResize.au64VirAddr[1]+ i * pTask->stIVEResize.au32Stride[1] + j);
				dispImg.data[i * dispImg.step[0] + 3 * j + 2] = *((HI_U8*)pTask->stIVEResize.au64VirAddr[2]+ i * pTask->stIVEResize.au32Stride[2] + j);
			}
		}
		sprintf(savename, "dispImg_%d.jpg",pTask->m_DetectCount++);
		imwrite(savename, dispImg);
	#endif		
#endif	

	*ppOcrDetectResult = &pTask->m_DetectResult;

	NNIEDetectResult nnie_results = {0};
	ObjectDetectionResult obj_results = {0};

	//memset(&nnie_results,  0, sizeof(nnie_results));
	//memset(&obj_results,   0, sizeof(obj_results));
	
	//NNIELaneResult tLaneResult = {0};
//    double centernet_start=getCurrentTime();
//    printf("centernet_prestart each image detect time  ========================  %lfms       \n",centernet_start - centernet_prestart);
	NNIE_Detect_init(pTask->m_OcrDetectHandle,1);
	NNIE_Detect_fillData(pTask->m_OcrDetectHandle, pTask->m_InputBGR);
	NNIE_Detect_Forword(pTask->m_OcrDetectHandle);
	NNIE_Detect_GetResult(pTask->m_OcrDetectHandle);
    //printf("******%s****debug***********%d\n",__FUNCTION__,__LINE__);
    NNIE_Detect_GetResultOut(pTask->m_OcrDetectHandle,INPUT_NET_WIDTH, INPUT_NET_HEIGHT, &nnie_results);
    //printf("***%s****%d***%d****\n",__FUNCTION__,nnie_results.obj_count,__LINE__);//
//    double centernet_end=getCurrentTime();
//    printf("centernet each image detect time  ========================  %lfms       \n",centernet_end-centernet_start);

    for(int i = 0; i < nnie_results.obj_count; ++i)
	{
		obj_results.m_DetectObj[i].char_class = nnie_results.m_DetectObj[i].char_class + 1;
		obj_results.m_DetectObj[i].fConfidence = nnie_results.m_DetectObj[i].fConfidence;
		obj_results.m_DetectObj[i].left   = nnie_results.m_DetectObj[i].left;
		obj_results.m_DetectObj[i].right  = nnie_results.m_DetectObj[i].right;
		obj_results.m_DetectObj[i].top    = nnie_results.m_DetectObj[i].top;
		obj_results.m_DetectObj[i].bottom = nnie_results.m_DetectObj[i].bottom;
	}
	obj_results.obj_count = nnie_results.obj_count;
#if 0
        cv::Mat dispImg(INPUT_NET_HEIGHT, INPUT_NET_WIDTH, CV_8UC3);
        for(int i = 0; i < dispImg.rows; i++)
        {
            for(int j = 0; j < dispImg.cols; j++)
            {
                dispImg.data[i * dispImg.step[0] + 3 * j]	    = *(pTask->m_InputBGR +INPUT_NET_HEIGHT * INPUT_NET_WIDTH * 0 + i * INPUT_NET_WIDTH + j );
                dispImg.data[i * dispImg.step[0] + 3 * j + 1] = *(pTask->m_InputBGR +INPUT_NET_HEIGHT * INPUT_NET_WIDTH * 1 + i * INPUT_NET_WIDTH   + j );
                dispImg.data[i * dispImg.step[0] + 3 * j + 2] = *(pTask->m_InputBGR +INPUT_NET_HEIGHT * INPUT_NET_WIDTH * 2 + i * INPUT_NET_WIDTH   + j  );
            }
        }
        //锟斤拷锟斤拷锟斤拷锟斤拷
        if(obj_results.obj_count > 0)
        {
        for(int i = 0;i < obj_results.obj_count; i++){
                cv::Point x0 = cv::Point(obj_results.m_DetectObj[i].left,obj_results.m_DetectObj[i].top);
                cv::Point y0 = cv::Point(obj_results.m_DetectObj[i].right,obj_results.m_DetectObj[i].bottom);
                printf("--------(%d,%d)---------(%d,%d)-------\n",x0.x,x0.y,y0.x,y0.y);
            cv::rectangle(dispImg, x0, y0, cv::Scalar(0, 0, 255), 3, 8);
            std::stringstream class_confidences;
            std::string class_confidence;
            class_confidences.clear();
            //class_confidences.setf(std::ios::fixed);
            class_confidences<<obj_results.m_DetectObj[i].char_class<<"-";
            class_confidences.setf(std::ios::fixed);
            //class_confidences<< pTrackResult->pCurDetectResult->m_DetectObj[i].fConfidence;

            class_confidences>>class_confidence;
            double font_scale = 3;
            int thickness = 3;
            int lineType = 8;
            //printf("*********************%d\n",__LINE__);

            cv::putText(dispImg,class_confidence,x0,cv::FONT_HERSHEY_PLAIN,font_scale,cv::Scalar(0, 0, 255),thickness,lineType);
        }
        }else
        {
        printf("------------pTrackResult->pCurDetectResult num is zero------------\n");
            //break;
        }

        imwrite("./Img_centernet.jpg", dispImg);
#endif
    std::vector<float> xmin;
    std::vector<float> ymin;
    std::vector<float> xmax;
    std::vector<float> ymax;
    std::vector<float> scores;
    std::vector<float> classes;
    for(int i=0;i<obj_results.obj_count;i++)
    {
        xmin.push_back(obj_results.m_DetectObj[i].left);
        ymin.push_back(obj_results.m_DetectObj[i].top);
        xmax.push_back(obj_results.m_DetectObj[i].right);
        ymax.push_back(obj_results.m_DetectObj[i].bottom);
        scores.push_back(obj_results.m_DetectObj[i].fConfidence);
        classes.push_back(obj_results.m_DetectObj[i].char_class);
    }
/*
	std::vector<std::vector<float>> outputs;
    outputs.push_back(xmin_);
    outputs.push_back(ymin_);
    outputs.push_back(xmax_);
    outputs.push_back(ymax_);
    outputs.push_back(scores_);
    outputs.push_back(classes_);

    //******************PPselectObj******************
	//锟斤拷锟斤拷1锟斤拷锟斤拷筛选一锟轿ｏ拷 锟斤拷锟斤拷outputs锟斤拷锟斤拷锟絭ector anchorC0[centerX0,centerY0,R0]	vector anchorC1[centerX1,centerY1,R0]
    std::vector<float> xmin=outputs[0];
    std::vector<float> ymin=outputs[1];
    std::vector<float> xmax=outputs[2];
    std::vector<float> ymax=outputs[3];
    std::vector<float> scores=outputs[4];
*/
    std::vector<float> centerXs;
    std::vector<float> centerYs;

    if (scores.size() != 4)
    {
//        printf("Error(:), center net output num ls  not equal 4 !\n");// 锟斤拷锟斤拷锟届常
        printf("error***%s*******%d****\n",__FUNCTION__,__LINE__);
        return SDK_ERR_NOINIT;
    }

    for(int i=0;i<4;i++)
    {
        float centerX = ((xmax[i]+xmin[i])/2.0);
        float centerY = ((ymax[i]+ymin[i])/2.0);
        //float centerX = (abs(xmax[i]-xmin[i]) * 4.0);
        //float centerY = (abs(ymax[i]+ymin[i]) * 4.0);
        centerXs.push_back(centerX);
        centerYs.push_back(centerY);
    }
	/**************************************************/
    //锟斤拷锟斤拷2锟斤拷锟竭达拷映锟斤拷锟皆�蛣1�7
    //锟斤拷锟斤拷锟斤拷锟斤拷偏锟斤拷值
    std::vector<float> centerXsNew;
    std::vector<float> centerYsNew;
    std::vector<float> xminNew;
    std::vector<float> yminNew;
    std::vector<float> xmaxNew;
    std::vector<float> ymaxNew;

    float scaleW = 1.0*image_width/INPUT_NET_WIDTH;
    float scaleH = 1.0*image_height/INPUT_NET_HEIGHT;
    //循锟斤拷锟斤拷锟斤拷
    for( int i = 0 ; i < centerXs.size(); i++)
    {
        float centerX1 = (centerXs[i])*scaleW;
        float centerY1 = (centerYs[i])*scaleH;
        //printf("******%s***%d****%f****%f****\n",__FUNCTION__,__LINE__,centerX1,centerY1);
        centerXsNew.push_back(centerX1);
        centerYsNew.push_back(centerY1);

        xminNew.push_back((xmin[i])*scaleW * 1.0);
        xmaxNew.push_back((xmax[i])*scaleW * 1.0);
        yminNew.push_back((ymin[i])*scaleH * 1.0);
        ymaxNew.push_back((ymax[i])*scaleH * 1.0);
    }
	
	//锟截硷拷锟斤拷匹锟斤拷
    std::vector<int> matchResult;
    float tX = centerXsNew[3];
    float tY = centerYsNew[3];
    std::vector<float> dis_All;
    float indexValue = 10000000000000000;
    int index_;
    for (int index_1=0;index_1<3;index_1++)
    {
        float dis_ = (tX-centerXsNew[index_1])*(tX-centerXsNew[index_1])+(tY-centerYsNew[index_1])*(tY-centerYsNew[index_1]);
        dis_All.push_back(dis_);
        if(indexValue>dis_)
        {
            indexValue = dis_;
            index_ = index_1;
        }
    }
    float pax = centerXsNew[3];
    float pay = centerYsNew[3];
    float pcx = centerXsNew[index_];
    float pcy = centerYsNew[index_];
    int indexFur[2] = {0,0};
    int indexTemp = 0;
    for (int indexOther=0;indexOther<3;indexOther++){
        if(indexOther != index_){
            indexFur[indexTemp] = indexOther;
            indexTemp++;
        }
    }
    ////gengju dian zai zhixian shangfang panduan(>0 zai xian de xianfang;<0 zai shangfang)
    float dis1 =0.0, dis2=0.0;
    dis1 = pointLineDis(pax, pay,centerXsNew[indexFur[0]],centerYsNew[indexFur[0]], pcx, pcy);
    dis2 = pointLineDis(pax, pay,centerXsNew[indexFur[0]],centerYsNew[indexFur[0]], centerXsNew[indexFur[1]],centerYsNew[indexFur[1]]);
    //matchResult[0]:first rows left index;matchResult[1]:first rows right index;matchResult[2]:second rows left index; matchResult[3]:second rows right index;
    if(dis1*dis2>0)
    {
        if(centerYsNew[3]>centerYsNew[index_])//centerYsNew[index_]:first rows ; centerYsNew[3]:second rows
        {
            if(centerXsNew[indexFur[1]]>centerXsNew[index_])//centerYsNew[index_]:first rows  left index; centerYsNew[indexFur[1]]:first rows right index
            {
                matchResult.push_back(index_);
                matchResult.push_back(indexFur[1]);
            }else{
                matchResult.push_back(indexFur[1]);
                matchResult.push_back(index_);
            }
            if(centerXsNew[3]>centerXsNew[indexFur[0]])
            {
                matchResult.push_back(indexFur[0]);
                matchResult.push_back(3);
            }else{
                matchResult.push_back(3);
                matchResult.push_back(indexFur[0]);
            }
        }else
        {
            if(centerXsNew[3]>centerXsNew[indexFur[0]])
            {
                matchResult.push_back(indexFur[0]);
                matchResult.push_back(3);
            }else{
                matchResult.push_back(3);
                matchResult.push_back(indexFur[0]);
            }
            if(centerXsNew[indexFur[1]]>centerXsNew[index_])
            {
                matchResult.push_back(index_);
                matchResult.push_back(indexFur[1]);
            }else{
                matchResult.push_back(indexFur[1]);
                matchResult.push_back(index_);
            }
        }
//        matchResult.push_back(3);
//        matchResult.push_back(indexFur[0]);
//        matchResult.push_back(indexFur[1]);
//        matchResult.push_back(index_);
    }
    else
    {
        if(centerYsNew[3]>centerYsNew[index_])//centerYsNew[index_]:first rows ; centerYsNew[3]:second rows
        {
            if(centerXsNew[indexFur[0]]>centerXsNew[index_])//centerYsNew[index_]:first rows  left index; centerYsNew[indexFur[0]]:first rows right index
            {
                matchResult.push_back(index_);
                matchResult.push_back(indexFur[0]);
            }else{
                matchResult.push_back(indexFur[0]);
                matchResult.push_back(index_);
            }
            if(centerXsNew[3]>centerXsNew[indexFur[1]])
            {
                matchResult.push_back(indexFur[1]);
                matchResult.push_back(3);
            }else{
                matchResult.push_back(3);
                matchResult.push_back(indexFur[1]);
            }
        }else
        {
            if(centerXsNew[3]>centerXsNew[indexFur[1]])
            {
                matchResult.push_back(indexFur[1]);
                matchResult.push_back(3);
            }else{
                matchResult.push_back(3);
                matchResult.push_back(indexFur[1]);
            }
            if(centerXsNew[indexFur[0]]>centerXsNew[index_])
            {
                matchResult.push_back(index_);
                matchResult.push_back(indexFur[0]);
            }else{
                matchResult.push_back(indexFur[0]);
                matchResult.push_back(index_);
            }
        }
//        matchResult.push_back(3);
//        matchResult.push_back(indexFur[1]);
//        matchResult.push_back(indexFur[0]);
//        matchResult.push_back(index_);
    }
	if(pTask->m_OcrClassfyHandle)
	{
        #if 1
//C++ cut image	
        //printf("*%s***%f***%f***%f***%f***%d****\n",__FUNCTION__,centerXsNew[matchResult[0]],centerXsNew[matchResult[1]],centerXsNew[matchResult[2]],centerXsNew[matchResult[3]],__LINE__);
        //printf("*%s***%f***%f***%f***%f***%d****\n",__FUNCTION__,centerYsNew[matchResult[0]],centerYsNew[matchResult[1]],centerYsNew[matchResult[2]],centerYsNew[matchResult[3]],__LINE__);
        float x1 = centerXsNew[matchResult[0]];
        float x2 = centerXsNew[matchResult[1]];
        float y1 = centerYsNew[matchResult[0]];
        float y2 = centerYsNew[matchResult[1]];
        //second
        float x3 = centerXsNew[matchResult[2]];
        float x4 = centerXsNew[matchResult[3]];
        float y3 = centerYsNew[matchResult[2]];
        float y4 = centerYsNew[matchResult[3]];

        float k = (y2-y1)/(x2-x1);
        float b = y1 - k * x1;
        float src_cosangle = (x2-x1) / sqrt(pow(y2-y1,2)+pow(x2-x1,2));
        float src_sinangle = (y2-y1) / sqrt(pow(y2-y1,2)+pow(x2-x1,2));
        //printf("*%s***%f***%f***%d****\n",__FUNCTION__,src_cosangle,src_sinangle,__LINE__);
        //second
        float second_k = (y4-y3)/(x4-x3);
        float second_b = y3 - second_k * x3;
        float second_src_cosangle = (x4-x3) / sqrt(pow(y4-y3,2)+pow(x4-x3,2));
        float second_src_sinangle = (y4-y3) / sqrt(pow(y4-y3,2)+pow(x4-x3,2));

        float charactorboxW[4];
        float charactorboxH[4];
        float diagonallength[4];
        charactorboxW[0] = xmaxNew[0] - xminNew[0];
        charactorboxH[0] = ymaxNew[0] - yminNew[0];
        //printf("*%s***%f***%f***%d****\n",__FUNCTION__,charactorboxW[0],charactorboxH[0],__LINE__);
        diagonallength[0] = sqrt(pow(charactorboxW[0],2)+pow(charactorboxH[0],2));
        charactorboxW[1] = xmaxNew[1] - xminNew[1];
        charactorboxH[1] = ymaxNew[1] - yminNew[1];
        diagonallength[1] = sqrt(pow(charactorboxW[1],2)+pow(charactorboxH[1],2));
        float src_cut_height = diagonallength[1] > diagonallength[0]?diagonallength[1]:diagonallength[0] + 2;//锟斤拷取锟侥高讹拷
        float offset_b = src_cut_height/2.0/src_cosangle;
        //second
        charactorboxW[2] = xmaxNew[2] - xminNew[2];
        charactorboxH[2] = ymaxNew[2] - yminNew[2];
        diagonallength[2] = sqrt(pow(charactorboxW[2],2)+pow(charactorboxH[2],2));
        charactorboxW[3] = xmaxNew[3] - xminNew[3];
        charactorboxH[3] = ymaxNew[3] - yminNew[3];
        diagonallength[3] = sqrt(pow(charactorboxW[3],2)+pow(charactorboxH[3],2));
        float second_src_cut_height = diagonallength[3] > diagonallength[2]?diagonallength[3]:diagonallength[2] + 2;//锟斤拷取锟侥高讹拷
        float second_offset_b = second_src_cut_height/2.0/second_src_cosangle;

        float cutx1 = x1 - charactorboxW[0];
        cutx1 = cutx1 > 0?cutx1:0;
        float cuty1 = k * cutx1 + b;
        float cutx2 = x2 + charactorboxW[0];
        cutx2 = cutx2 < image_width ? cutx2 : image_width;
        float cuty2 = k * cutx2 + b;
        float src_cut_width = sqrt(pow(cutx2-cutx1,2)+pow(cuty2 -cuty1,2));//锟斤拷取锟侥匡拷锟斤拷

        //second
        float cutx3 = x3 - charactorboxW[2];
        cutx3 = cutx3 > 0?cutx3:0;
        float cuty3 = second_k * cutx3 + second_b;
        float cutx4 = x4 + charactorboxW[2];
        cutx4 = cutx4 < image_width ? cutx4 : image_width;
        float cuty4 = second_k * cutx4 + second_b;
        float second_src_cut_width = sqrt(pow(cutx4-cutx3,2)+pow(cuty4 -cuty3,2));//锟斤拷取锟侥匡拷锟斤拷


        //calculate vertical line
        float k1 = -1.0/k;
        float b1  = cuty1 - k1* cutx1;
        k = k;
        b = b - offset_b;
        //锟斤拷取锟斤拷始锟斤拷
        float src_reallycutx1 = (b1 - b)/(k - k1);
        float src_reallycuty1 = k1 * src_reallycutx1 + b1;
        //printf("*%s***%f***%f***%d****\n",__FUNCTION__,src_reallycutx1,src_reallycuty1,__LINE__);

        //second
        //calculate vertical line
        float second_k1 = -1.0/second_k;
        float second_b1  = cuty3 - second_k1* cutx3;
        second_k = second_k;
        second_b = second_b - second_offset_b;
        //锟斤拷取锟斤拷始锟斤拷
        float src_reallycutx2= (second_b1 - second_b)/(second_k - second_k1);
        float src_reallycuty2 = second_k1 * src_reallycutx2 + second_b1;
        //printf("*%s***%f***%f***%d****\n",__FUNCTION__,src_reallycutx2,src_reallycuty2,__LINE__);
//        double zhangsongcenternet_to_crnn=getCurrentTime();
//        printf("zhangsongcenternet_to_crnn each image detect time  ========================  %lfms       \n",zhangsongcenternet_to_crnn-centernet_end);


        OSAL_UCHAR *pFrame0 = (OSAL_UCHAR*)pTask->stIVEInputBGR.au64VirAddr[0];
        OSAL_UCHAR *pFrame1 = (OSAL_UCHAR*)pTask->stIVEInputBGR.au64VirAddr[1];
        OSAL_UCHAR *pFrame2 = (OSAL_UCHAR*)pTask->stIVEInputBGR.au64VirAddr[2];
#if 0
        //锟饺憋拷锟斤拷锟斤拷
        memset(pTask->m_InputBGR, 0, CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT*3);
        float crnn_Relative_scaleW1 = 1.0*CRNN_INPUT_NET_WIDTH/src_cut_width;
        float crnn_Relative_scaleH1 = 1.0*CRNN_INPUT_NET_HEIGHT/src_cut_height;
        float crnn_Relative_scale1 = crnn_Relative_scaleW1 < crnn_Relative_scaleH1?crnn_Relative_scaleW1:crnn_Relative_scaleH1;

        for(int i = 0; i < (int)(src_cut_height * crnn_Relative_scale1+0.5f); i++)
        {
            for(int j = 0, src_x = 0.0, src_y = 0.0; j < (int)(src_cut_width * crnn_Relative_scale1+0.5f); j++)
            {
                float templex = (j / crnn_Relative_scale1 * src_cosangle - i / crnn_Relative_scale1 * src_sinangle) ;
                float templey = (j / crnn_Relative_scale1 * src_sinangle + i / crnn_Relative_scale1 * src_cosangle) ;
//                printf("*%s*i:%d*j:%d*%f***%f***%d****\n",__FUNCTION__,i,j,templex,templey,__LINE__);
                src_x = (int)((templex + src_reallycutx1)+0.5) ;
                src_y = (int)((templey + src_reallycuty1)+0.5)  ;
//                printf("*%s***i:%d*j:%d***%d***%d***%d****\n",__FUNCTION__,i,j,src_x,src_y,__LINE__);
                if((src_x < 0) || (src_x >= image_width) || (src_y < 0) || (src_y >= image_height))
                {
                    continue;
                }
                pTask->m_InputBGR[i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame0 + src_y * pTask->stIVEInputBGR.au32Stride[0] + src_x);
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame1 + src_y * pTask->stIVEInputBGR.au32Stride[1] + src_x);;
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT * 2 + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame2 + src_y * pTask->stIVEInputBGR.au32Stride[2] + src_x);;
            }
        }
        #else
        //锟斤拷值锟斤拷锟斤拷
        memset(pTask->m_InputBGR, 0, CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT*3);
        float crnn_Relative_scaleW1 = 1.0*src_cut_width/CRNN_INPUT_NET_WIDTH;
        float crnn_Relative_scaleH1 = 1.0*src_cut_height/CRNN_INPUT_NET_HEIGHT;

        for(int i = 0; i < CRNN_INPUT_NET_HEIGHT; i++)
        {
            for(int j = 0, src_x = 0.0, src_y = 0.0; j < CRNN_INPUT_NET_WIDTH; j++)
            {
                float templex = (j * crnn_Relative_scaleW1 * src_cosangle - i * crnn_Relative_scaleH1 * src_sinangle) ;
                float templey = (j * crnn_Relative_scaleW1 * src_sinangle + i * crnn_Relative_scaleH1 * src_cosangle) ;
//                printf("*%s*i:%d*j:%d*%f***%f***%d****\n",__FUNCTION__,i,j,templex,templey,__LINE__);
                src_x = (int)((templex + src_reallycutx1)+0.5) ;
                src_y = (int)((templey + src_reallycuty1)+0.5)  ;
//                printf("*%s***i:%d*j:%d***%d***%d***%d****\n",__FUNCTION__,i,j,src_x,src_y,__LINE__);
                if((src_x < 0) || (src_x >= image_width) || (src_y < 0) || (src_y >= image_height))
                {
                    continue;
                }
                pTask->m_InputBGR[i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame0 + src_y * pTask->stIVEInputBGR.au32Stride[0] + src_x);
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame1 + src_y * pTask->stIVEInputBGR.au32Stride[1] + src_x);;
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT * 2 + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame2 + src_y * pTask->stIVEInputBGR.au32Stride[2] + src_x);;
            }
        }
        #endif
//        double crnn_start=getCurrentTime();
        NNIEOcrCrnnResult Result1;
        NNIE_Detect_init(pTask->m_OcrClassfyHandle,1);
        NNIE_Detect_fillData(pTask->m_OcrClassfyHandle, pTask->m_InputBGR);
        NNIE_Detect_Forword(pTask->m_OcrClassfyHandle);
        NNIE_Detect_GetResult(pTask->m_OcrClassfyHandle);
        NNIE_Detect_GetOcrCrnnOut(pTask->m_OcrClassfyHandle,  &Result1);
//        double crnn_end=getCurrentTime();
//        printf("crnnnet each image detect time  ========================  %lfms       \n",crnn_end-crnn_start);
        #if 0
        cv::Mat dispImgfirst(CRNN_INPUT_NET_HEIGHT, CRNN_INPUT_NET_WIDTH, CV_8UC3);
        for(int i = 0; i < dispImgfirst.rows; i++)
        {
            for(int j = 0; j < dispImgfirst.cols; j++)
            {
                dispImgfirst.data[i * dispImgfirst.step[0] + 3 * j]	    = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 0 + i * CRNN_INPUT_NET_WIDTH + j );
                dispImgfirst.data[i * dispImgfirst.step[0] + 3 * j + 1] = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 1 + i * CRNN_INPUT_NET_WIDTH   + j );
                dispImgfirst.data[i * dispImgfirst.step[0] + 3 * j + 2] = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 2 + i * CRNN_INPUT_NET_WIDTH   + j  );
            }
        }
        imwrite("./dispImg_cutfirst.jpg", dispImgfirst);
        #endif



#if 0
        //second   锟饺憋拷锟斤拷锟斤拷
        memset(pTask->m_InputBGR, 0, CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT*3);
        float second_crnn_Relative_scaleW1 = 1.0*CRNN_INPUT_NET_WIDTH/second_src_cut_width;
        float second_crnn_Relative_scaleH1 = 1.0*CRNN_INPUT_NET_HEIGHT/second_src_cut_height;
        float second_crnn_Relative_scale1 = second_crnn_Relative_scaleW1 < second_crnn_Relative_scaleH1?second_crnn_Relative_scaleW1:second_crnn_Relative_scaleH1;

        for(int i = 0; i < (int)(second_src_cut_height * second_crnn_Relative_scale1+0.5f); i++)
        {
            for(int j = 0, src_x = 0.0, src_y = 0.0; j < (int)(second_src_cut_width * second_crnn_Relative_scale1+0.5f); j++)
            {
                float templex = (j / second_crnn_Relative_scale1 * second_src_cosangle - i / second_crnn_Relative_scale1 * second_src_sinangle) ;
                float templey = (j / second_crnn_Relative_scale1 * second_src_sinangle + i / second_crnn_Relative_scale1 * second_src_cosangle) ;
//                printf("*%s*i:%d*j:%d*%f***%f***%d****\n",__FUNCTION__,i,j,templex,templey,__LINE__);
                src_x = (int)((templex + src_reallycutx2)+0.5) ;
                src_y = (int)((templey + src_reallycuty2)+0.5)  ;
//                printf("*%s***i:%d*j:%d***%d***%d***%d****\n",__FUNCTION__,i,j,src_x,src_y,__LINE__);
                if((src_x < 0) || (src_x >= image_width) || (src_y < 0) || (src_y >= image_height))
                {
                    continue;
                }
                pTask->m_InputBGR[i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame0 + src_y * pTask->stIVEInputBGR.au32Stride[0] + src_x);
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame1 + src_y * pTask->stIVEInputBGR.au32Stride[1] + src_x);;
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT * 2 + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame2 + src_y * pTask->stIVEInputBGR.au32Stride[2] + src_x);;
            }
        }
        #else
        
        //second   锟斤拷值锟斤拷锟斤拷
        memset(pTask->m_InputBGR, 0, CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT*3);
        float second_crnn_Relative_scaleW1 = 1.0*second_src_cut_width/CRNN_INPUT_NET_WIDTH;
        float second_crnn_Relative_scaleH1 = 1.0*second_src_cut_height/CRNN_INPUT_NET_HEIGHT;

        for(int i = 0; i < CRNN_INPUT_NET_HEIGHT; i++)
        {
            for(int j = 0, src_x = 0.0, src_y = 0.0; j < CRNN_INPUT_NET_WIDTH; j++)
            {
                float templex = (j * second_crnn_Relative_scaleW1 * second_src_cosangle - i * second_crnn_Relative_scaleH1 * second_src_sinangle) ;
                float templey = (j * second_crnn_Relative_scaleW1 * second_src_sinangle + i * second_crnn_Relative_scaleH1 * second_src_cosangle) ;
//                printf("*%s*i:%d*j:%d*%f***%f***%d****\n",__FUNCTION__,i,j,templex,templey,__LINE__);
                src_x = (int)((templex + src_reallycutx2)+0.5) ;
                src_y = (int)((templey + src_reallycuty2)+0.5)  ;
//                printf("*%s***i:%d*j:%d***%d***%d***%d****\n",__FUNCTION__,i,j,src_x,src_y,__LINE__);
                if((src_x < 0) || (src_x >= image_width) || (src_y < 0) || (src_y >= image_height))
                {
                    continue;
                }
                pTask->m_InputBGR[i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame0 + src_y * pTask->stIVEInputBGR.au32Stride[0] + src_x);
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame1 + src_y * pTask->stIVEInputBGR.au32Stride[1] + src_x);;
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT * 2 + i * CRNN_INPUT_NET_WIDTH + j] = *(pFrame2 + src_y * pTask->stIVEInputBGR.au32Stride[2] + src_x);;
            }
        }
        #endif
        NNIEOcrCrnnResult Result2;
        NNIE_Detect_init(pTask->m_OcrClassfyHandle,1);
        NNIE_Detect_fillData(pTask->m_OcrClassfyHandle, pTask->m_InputBGR);
        NNIE_Detect_Forword(pTask->m_OcrClassfyHandle);
        NNIE_Detect_GetResult(pTask->m_OcrClassfyHandle);
        NNIE_Detect_GetOcrCrnnOut(pTask->m_OcrClassfyHandle,  &Result2);

#if 0
        cv::Mat dispImgfirst(CRNN_INPUT_NET_HEIGHT, CRNN_INPUT_NET_WIDTH, CV_8UC3);
        for(int i = 0; i < dispImgfirst.rows; i++)
        {
            for(int j = 0; j < dispImgfirst.cols; j++)
            {
                dispImgfirst.data[i * dispImgfirst.step[0] + 3 * j]	    = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 0 + i * CRNN_INPUT_NET_WIDTH + j );
                dispImgfirst.data[i * dispImgfirst.step[0] + 3 * j + 1] = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 1 + i * CRNN_INPUT_NET_WIDTH   + j );
                dispImgfirst.data[i * dispImgfirst.step[0] + 3 * j + 2] = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 2 + i * CRNN_INPUT_NET_WIDTH   + j  );
            }
        }
        imwrite("./dispImg_cutfirst.jpg", dispImgfirst);
#endif

//opencv cut image	
#else
		cv::Mat orginImg = cv::Mat(image_height, image_width, CV_8UC3);
		

		for(int i = 0; i < image_height; i++)
		{
			for(int j = 0; j < image_width; j++)
			{
				orginImg.at<cv::Vec3b>(i,j)[0] = pImage[3*image_width*i+j*3];
				orginImg.at<cv::Vec3b>(i,j)[1] = pImage[3*image_width*i+j*3 + 1];
				orginImg.at<cv::Vec3b>(i,j)[2] = pImage[3*image_width*i+j*3 + 2];
			}
		}
		//sprintf(savename, "dispImg_%d.jpg",pTask->m_DetectCount++);
		//imwrite(savename, dispImg);
		

		cv::Mat mapImg = PPcopyDarwMap(orginImg,centerXsNew,centerYsNew,matchResult);


	   // cv::imwrite("PPmapImg.jpg",mapImg);
	    //锟斤拷锟斤拷6锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷 锟斤拷锟斤拷angle锟斤拷原始图锟斤拷copy图锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�1�7
	    cv::Mat orginImgBig;
	    cv::Mat mapImgBig;
	    PPbigImg(orginImg,mapImg,orginImgBig,mapImgBig);
	    //cv::imwrite("mapImgBig.jpg",mapImgBig);
	    //cv::imwrite("orginImgBig.jpg",orginImgBig);

	    //锟矫碉拷contours
	    std::vector<std::vector<cv::Point>> contours;
	    std::vector<cv::Vec4i> hierarcy;

	    double thresh = 100;
	    int maxVal = 255;
	    cv::threshold(mapImgBig, mapImgBig, thresh, maxVal, cv::THRESH_BINARY);
	    cv::findContours(mapImgBig, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());
	    cv::RotatedRect box0 = cv::minAreaRect(cv::Mat(contours[0]));  //锟斤拷锟斤拷每锟斤拷锟斤拷锟斤拷锟斤拷小锟斤拷泳锟斤拷锟�1�7
	    cv::RotatedRect box1 = cv::minAreaRect(cv::Mat(contours[1]));  //锟斤拷锟斤拷每锟斤拷锟斤拷锟斤拷锟斤拷小锟斤拷泳锟斤拷锟�1�7 锟节讹拷锟斤拷锟街凤拷
	    if(box0.center.y > box1.center.y)
		{

			cv::RotatedRect boxtmp;
			boxtmp = box0;
			box0 = box1;
            box1 = boxtmp;
		}else{
			
		}
	    cv::Mat imgROI1 = PProtateImage(orginImgBig, box0);// 锟斤拷一锟斤拷锟街凤拷
	    cv::Mat imgROI2 = PProtateImage(orginImgBig, box1);// 锟节讹拷锟斤拷锟街凤拷
	    cv::imwrite("PPimgROI1.jpg",imgROI1);
	    cv::imwrite("PPimgROI2.jpg",imgROI2);
	    std::vector<cv::Mat> imgROI;
	    imgROI.push_back(imgROI2);
	    imgROI.push_back(imgROI1);
//        double centernet_to_crnn=getCurrentTime();
//        printf("centernet_to_crnn each image detect time  ========================  %lfms       \n",centernet_to_crnn-centernet_end);
		/*
		for (int i = 0; i < img_width * img_height * 3; i++)
		{
			p1[i]= (uchar)img.at<Vec3b>(i / (img_width * 3), (i % (img_width * 3)) / 3)[i % 3];
		}*/
		memset(pTask->m_InputBGR, 0, CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT*3);
        float crnn_scaleW1 = 1.0*imgROI1.cols/CRNN_INPUT_NET_WIDTH;
        float crnn_scaleH1 = 1.0*imgROI1.rows/CRNN_INPUT_NET_HEIGHT;
		for(int i = 0; i < CRNN_INPUT_NET_HEIGHT; i++)
		{
			for(int j = 0; j < CRNN_INPUT_NET_WIDTH; j++)
			{
                pTask->m_InputBGR[i * CRNN_INPUT_NET_WIDTH + j] = imgROI1.at<cv::Vec3b>((int)(i*crnn_scaleH1),(int)(j*crnn_scaleW1))[0];
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT + i * CRNN_INPUT_NET_WIDTH + j] = imgROI1.at<cv::Vec3b>((int)(i*crnn_scaleH1),(int)(j*crnn_scaleW1))[1];
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT * 2 + i * CRNN_INPUT_NET_WIDTH + j] = imgROI1.at<cv::Vec3b>((int)(i*crnn_scaleH1),(int)(j*crnn_scaleW1))[2];
			}
		}
#if 1
    char savename[100];
//        cv::Mat dispImg(CRNN_INPUT_NET_HEIGHT, CRNN_INPUT_NET_WIDTH, CV_8UC3);
//        for(int i = 0; i < dispImg.rows; i++)
//        {
//            for(int j = 0; j < dispImg.cols; j++)
//            {
//                dispImg.data[i * dispImg.step[0] + 3 * j]	    = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 0 + i * CRNN_INPUT_NET_WIDTH + j );
//                dispImg.data[i * dispImg.step[0] + 3 * j + 1] = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 1 + i * CRNN_INPUT_NET_WIDTH   + j );
//                dispImg.data[i * dispImg.step[0] + 3 * j + 2] = *(pTask->m_InputBGR +CRNN_INPUT_NET_HEIGHT * CRNN_INPUT_NET_WIDTH * 2 + i * CRNN_INPUT_NET_WIDTH   + j  );
//            }
//        }
        sprintf(savename, "./dispImg_%d.jpg",pTask->m_DetectCount);
//        imwrite(savename, dispImg);
#endif


//        double crnn_start=getCurrentTime();
        NNIEOcrCrnnResult Result1;
		NNIE_Detect_init(pTask->m_OcrClassfyHandle,1);
		NNIE_Detect_fillData(pTask->m_OcrClassfyHandle, pTask->m_InputBGR);
		NNIE_Detect_Forword(pTask->m_OcrClassfyHandle);
		NNIE_Detect_GetResult(pTask->m_OcrClassfyHandle);
        NNIE_Detect_GetOcrCrnnOut(pTask->m_OcrClassfyHandle,  &Result1);
//        double crnn_end=getCurrentTime();
//        printf("crnnnet each image detect time  ========================  %lfms       \n",crnn_end-crnn_start);

		memset(pTask->m_InputBGR, 0, CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT*3);
        float crnn_scaleW2 = 1.0*imgROI2.cols/CRNN_INPUT_NET_WIDTH;
        float crnn_scaleH2 = 1.0*imgROI2.rows/CRNN_INPUT_NET_HEIGHT;
		for(int i = 0; i < CRNN_INPUT_NET_HEIGHT; i++)
		{
			for(int j = 0; j < CRNN_INPUT_NET_WIDTH; j++)
			{
                pTask->m_InputBGR[i * CRNN_INPUT_NET_WIDTH + j] = imgROI2.at<cv::Vec3b>((int)(i*crnn_scaleH2),(int)(j*crnn_scaleW2))[0];
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT + i * CRNN_INPUT_NET_WIDTH + j] = imgROI2.at<cv::Vec3b>((int)(i*crnn_scaleH2),(int)(j*crnn_scaleW2))[1];
                pTask->m_InputBGR[CRNN_INPUT_NET_WIDTH*CRNN_INPUT_NET_HEIGHT * 2 + i * CRNN_INPUT_NET_WIDTH + j] = imgROI2.at<cv::Vec3b>((int)(i*crnn_scaleH2),(int)(j*crnn_scaleW2))[2];
			}
		}
        NNIEOcrCrnnResult Result2;
		NNIE_Detect_init(pTask->m_OcrClassfyHandle,1);
		NNIE_Detect_fillData(pTask->m_OcrClassfyHandle, pTask->m_InputBGR);
		NNIE_Detect_Forword(pTask->m_OcrClassfyHandle);
		NNIE_Detect_GetResult(pTask->m_OcrClassfyHandle);
        NNIE_Detect_GetOcrCrnnOut(pTask->m_OcrClassfyHandle,  &Result2);
#endif
//        if(Result1.iOcrNum != 44)
//		{
//            printf("Error(:), Result1 size ls :%d  not equal 44 !\n",Result1.iOcrNum);// 锟斤拷锟斤拷锟届常
//		}
//        if(Result2.iOcrNum != 44)
//		{
//            printf("Error(:), Result2 size ls :%d   not equal 44 !\n",Result2.iOcrNum);// 锟斤拷锟斤拷锟届常
//		}
//        printf("******************\n");
        pTask->m_DetectResult.iOcrNum[0] = Result1.iOcrNum;
        for(int i = 0 ; i < Result1.iOcrNum; i++)
        {
//            printf("******************\n");iOcrNum
//            pTask->m_DetectResult.ocrs_character[0] += labels_[Result1.ocrs[i]];
            strcpy(pTask->m_DetectResult.ocrs_character[0] + strlen((char *)pTask->m_DetectResult.ocrs_character[0]), labels_[Result1.ocrs[i]].c_str());
            //printf("*%s*", labels_[Result1.ocrs[i]].c_str());
        }
        printf("\n");
        pTask->m_DetectResult.iOcrNum[1] = Result2.iOcrNum;
        for(int i = 0 ; i < Result2.iOcrNum; i++)
        {
            strcpy(pTask->m_DetectResult.ocrs_character[1] + strlen((char *)pTask->m_DetectResult.ocrs_character[1]), labels_[Result2.ocrs[i]].c_str());
            //pTask->m_DetectResult.ocrs_character[1] += labels_[Result2.ocrs[i]];
            //printf("*%s*", labels_[Result2.ocrs[i]].c_str());
        }
        printf("\n");

//        pTask->m_DetectResult.type += labels_[Result1.ocrs[0]];
//        pTask->m_DetectResult.type += labels_[Result1.ocrs[1]];
        strcpy(pTask->m_DetectResult.type + strlen(pTask->m_DetectResult.type), labels_[Result1.ocrs[0]].c_str());
        strcpy(pTask->m_DetectResult.type + strlen(pTask->m_DetectResult.type), labels_[Result1.ocrs[1]].c_str());
        //printf("*%s*\n", pTask->m_DetectResult.type.c_str());
//        pTask->m_DetectResult.nation += labels_[Result1.ocrs[2]];
//        pTask->m_DetectResult.nation += labels_[Result1.ocrs[3]];
//        pTask->m_DetectResult.nation += labels_[Result1.ocrs[4]];
        strcpy(pTask->m_DetectResult.nation + strlen(pTask->m_DetectResult.nation), labels_[Result1.ocrs[2]].c_str());
        strcpy(pTask->m_DetectResult.nation + strlen(pTask->m_DetectResult.nation), labels_[Result1.ocrs[3]].c_str());
        strcpy(pTask->m_DetectResult.nation + strlen(pTask->m_DetectResult.nation), labels_[Result1.ocrs[4]].c_str());
		bool surnameEnd = false;
        bool nameEnd = false;
        for(int i = 5 ; i < Result1.iOcrNum ; i++ )
		{
            if(labels_[Result1.ocrs[i]] != "<" && false == surnameEnd)
			{
//                pTask->m_DetectResult.full_name.Surname += labels_[Result1.ocrs[i]];
                strcpy(pTask->m_DetectResult.full_name.Surname + strlen(pTask->m_DetectResult.full_name.Surname), labels_[Result1.ocrs[i]].c_str());
            }else if(labels_[Result1.ocrs[i]] == "<" && false == surnameEnd)
			{
				surnameEnd = true;
            }else if(labels_[Result1.ocrs[i]] != "<" && true == surnameEnd )
			{
                //pTask->m_DetectResult.full_name.name += labels_[Result1.ocrs[i]];
                strcpy(pTask->m_DetectResult.full_name.name + strlen(pTask->m_DetectResult.full_name.name), labels_[Result1.ocrs[i]].c_str());
                //printf("i:%d*%s*\n", i,labels_[Result1.ocrs[i]].c_str());
			}else
			{
                continue;
			}
		}
        //printf("*%s*\n", pTask->m_DetectResult.full_name.Surname.c_str());
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[0]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[1]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[2]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[3]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[4]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[5]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[6]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[7]];
//        pTask->m_DetectResult.num += labels_[Result2.ocrs[8]];
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[0]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[1]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[2]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[3]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[4]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[5]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[6]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[7]].c_str());
        strcpy(pTask->m_DetectResult.num + strlen(pTask->m_DetectResult.num), labels_[Result2.ocrs[8]].c_str());

//        pTask->m_DetectResult.birthData += labels_[Result2.ocrs[13]];
//        pTask->m_DetectResult.birthData += labels_[Result2.ocrs[14]];
//        pTask->m_DetectResult.birthData += labels_[Result2.ocrs[15]];
//        pTask->m_DetectResult.birthData += labels_[Result2.ocrs[16]];
//        pTask->m_DetectResult.birthData += labels_[Result2.ocrs[17]];
//        pTask->m_DetectResult.birthData += labels_[Result2.ocrs[18]];
        strcpy(pTask->m_DetectResult.birthData + strlen(pTask->m_DetectResult.birthData), labels_[Result2.ocrs[13]].c_str());
        strcpy(pTask->m_DetectResult.birthData + strlen(pTask->m_DetectResult.birthData), labels_[Result2.ocrs[14]].c_str());
        strcpy(pTask->m_DetectResult.birthData + strlen(pTask->m_DetectResult.birthData), labels_[Result2.ocrs[15]].c_str());
        strcpy(pTask->m_DetectResult.birthData + strlen(pTask->m_DetectResult.birthData), labels_[Result2.ocrs[16]].c_str());
        strcpy(pTask->m_DetectResult.birthData + strlen(pTask->m_DetectResult.birthData), labels_[Result2.ocrs[17]].c_str());
        strcpy(pTask->m_DetectResult.birthData + strlen(pTask->m_DetectResult.birthData), labels_[Result2.ocrs[18]].c_str());

//        pTask->m_DetectResult.sex += labels_[Result2.ocrs[20]];
        strcpy(pTask->m_DetectResult.sex + strlen(pTask->m_DetectResult.sex), labels_[Result2.ocrs[20]].c_str());

//        pTask->m_DetectResult.validData += labels_[Result2.ocrs[21]];
//        pTask->m_DetectResult.validData += labels_[Result2.ocrs[22]];
//        pTask->m_DetectResult.validData += labels_[Result2.ocrs[23]];
//        pTask->m_DetectResult.validData += labels_[Result2.ocrs[24]];
//        pTask->m_DetectResult.validData += labels_[Result2.ocrs[25]];
//        pTask->m_DetectResult.validData += labels_[Result2.ocrs[26]];
        strcpy(pTask->m_DetectResult.validData + strlen(pTask->m_DetectResult.validData), labels_[Result2.ocrs[21]].c_str());
        strcpy(pTask->m_DetectResult.validData + strlen(pTask->m_DetectResult.validData), labels_[Result2.ocrs[22]].c_str());
        strcpy(pTask->m_DetectResult.validData + strlen(pTask->m_DetectResult.validData), labels_[Result2.ocrs[23]].c_str());
        strcpy(pTask->m_DetectResult.validData + strlen(pTask->m_DetectResult.validData), labels_[Result2.ocrs[24]].c_str());
        strcpy(pTask->m_DetectResult.validData + strlen(pTask->m_DetectResult.validData), labels_[Result2.ocrs[25]].c_str());
        strcpy(pTask->m_DetectResult.validData + strlen(pTask->m_DetectResult.validData), labels_[Result2.ocrs[26]].c_str());


		
        for(int i = 28 ; i < Result2.iOcrNum ; i++ )
		{
            if(labels_[Result2.ocrs[i]] != "<")
			{
//                pTask->m_DetectResult.machineData += labels_[Result2.ocrs[i]];
                strcpy(pTask->m_DetectResult.machineData + strlen(pTask->m_DetectResult.machineData), labels_[Result2.ocrs[i]].c_str());
			}else{
				break;
			}
		}				


#ifdef WIN_DEBUG
		Mat image_ROI = Mat::zeros(iClassNetH, iClassNetW, CV_8UC3);
		for(int i = 0; i < image_ROI.rows; i++)
		{
			for(int j = 0; j < image_ROI.cols; j++)
			{
                image_ROI.data[i * image_ROI.step[0] + 3 * j]	    = pTask->m_InputBGR[i*iClassNetW+j];
				image_ROI.data[i * image_ROI.step[0] + 3 * j + 1] = pTask->m_InputBGR[iClassNetW*iClassNetH+i*iClassNetW+j];
				image_ROI.data[i * image_ROI.step[0] + 3 * j + 2] = pTask->m_InputBGR[iClassNetW*iClassNetH*2+i*iClassNetW+j];
			}
		} 
		sprintf(savename, "ROIImg_%d_iType%d_Confidence%f_Count%d.jpg",pLineInfo->nLaneMark,pTask->m_DetectResult.LaneArr[i].iLaneType,fMaxConfidence,pTask->m_DetectCount++);
		imwrite(savename, image_ROI);
		
#endif
	}
	else
	{
		; 
	}	

	
#if 0
	s32Ret = IVE_FrameFlushCache(&pTask->stIVEInputBGR);
	Mat dispImg(pTask->stIVEInputBGR.u32Height, pTask->stIVEInputBGR.au32Stride[2], CV_8UC1);
	dispImg.data = (OSAL_UCHAR*)pTask->stIVEInputBGR.au64VirAddr[2];
	sprintf(savename, "display_%d.jpg",pTask->m_DetectCount++);
	imwrite(savename, dispImg); 
#endif

	return SDK_ERR_NONE;

}


//////////////////////////////private function////////////////////////////////////


#ifdef HI_3559
static OSAL_INT32 IVE_FrameFlushCache(IVE_SRC_IMAGE_S *pIVEFrame)
{
	OSAL_INT32 s32Ret = 0;
	s32Ret = HI_MPI_SYS_MmzFlushCache(pIVEFrame->au64PhyAddr[0], 
		(HI_VOID *)pIVEFrame->au64VirAddr[0],
		pIVEFrame->au32Stride[0] * pIVEFrame->u32Height);
	if(HI_SUCCESS != s32Ret)
	{
		printf("Error(%#x),HI_MPI_SYS_MmzFlushCache failed!\n",s32Ret);
		return s32Ret;
	}

	if (pIVEFrame->enType == IVE_IMAGE_TYPE_U8C3_PACKAGE || pIVEFrame->enType == IVE_IMAGE_TYPE_U8C3_PLANAR)
	{
		s32Ret = HI_MPI_SYS_MmzFlushCache(pIVEFrame->au64PhyAddr[1], 
			(HI_VOID *)pIVEFrame->au64VirAddr[1],
			pIVEFrame->au32Stride[1] * pIVEFrame->u32Height);
		if(HI_SUCCESS != s32Ret)
		{
			printf("Error(%#x),HI_MPI_SYS_MmzFlushCache failed!\n",s32Ret);
			return s32Ret;
		}

		s32Ret = HI_MPI_SYS_MmzFlushCache(pIVEFrame->au64PhyAddr[2], 
			(HI_VOID *)pIVEFrame->au64VirAddr[2],
			pIVEFrame->au32Stride[2] * pIVEFrame->u32Height);
		if(HI_SUCCESS != s32Ret)
		{
			printf("Error(%#x),HI_MPI_SYS_MmzFlushCache failed!\n",s32Ret);
			return s32Ret;
		}
	}
	return s32Ret;
}

static OSAL_INT32 IVE_FrameYVUFlushCache(IVE_SRC_IMAGE_S *pIVEFrame)
{
	OSAL_INT32 s32Ret = 0;
	s32Ret = HI_MPI_SYS_MmzFlushCache(pIVEFrame->au64PhyAddr[0], 
		(HI_VOID *)pIVEFrame->au64VirAddr[0],
		pIVEFrame->au32Stride[0] * pIVEFrame->u32Height * 3 >> 1);
	if(HI_SUCCESS != s32Ret)
	{
		printf("Error(%#x),HI_MPI_SYS_MmzFlushCache failed!\n",s32Ret);
	}
	return s32Ret;
}

static void SplitImageToIVEFrame(IVE_DST_IMAGE_S *pIVEFrame,OSAL_UCHAR *pImage,OSAL_INT32 srcImgW, OSAL_INT32 srcImgH)
{
	int x,y;
	OSAL_UCHAR *mat_buff_index;
	OSAL_UCHAR *pFrame0 = (OSAL_UCHAR*)pIVEFrame->au64VirAddr[0];
	OSAL_UCHAR *pFrame1 = (OSAL_UCHAR*)pIVEFrame->au64VirAddr[1];
	OSAL_UCHAR *pFrame2 = (OSAL_UCHAR*)pIVEFrame->au64VirAddr[2];

	mat_buff_index = pImage;
    for(y = 0; y < srcImgH; ++y)
    {
        mat_buff_index   = pImage +  3*y*srcImgW;
        for(x = 0; x < srcImgW; ++x)
        {
            pFrame0[x] = mat_buff_index[3*x];
            pFrame1[x] = mat_buff_index[3*x + 1];
            pFrame2[x] = mat_buff_index[3*x + 2];
        }
		pFrame0 +=  pIVEFrame->au32Stride[0];
		pFrame1 +=  pIVEFrame->au32Stride[1];
		pFrame2 +=  pIVEFrame->au32Stride[2];
    }	
}

#endif

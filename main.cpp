#include <opencv.hpp>
//#include "NNIE/nnie.h"
//#include "OSAL_type.h"
#include "OcrDetectExport.h"
#include "stdio.h"

#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <string>

#include<stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <string>
#include "vector"
#include "sys/types.h"

#include <sys/time.h>


using namespace cv;


#define OCR_NET_PATH "./data/wk/centernet_inst_3519A_for_nnie.wk"
#define OCR_CLASS_PATH "./data/wk/crnn_inst_3519A_for_nnie.wk"
//#define OCR_CLASS_PATH "./data/wk/ocr_zhangsong_crnn_inst.wk"
//#define OCR_LABELS_PATH "./data/wk/label.txt"
#define OCR_LABELS_PATH "./data/wk/char_std_39.txt"
//#define INPUT_IMAGE_PATH "./data/inputimage/passport25.jpg"
#define INPUT_IMAGE_PATH "./data/inputimage/passport23.jpg"
//#define INPUT_IMAGE_PATH "./data/inputimage/passport28.jpg"
//#define 

#if 1
#define INPUT_IMAGE_HEIGHT 480
#define INPUT_IMAGE_WIDTH 480
#else
#define INPUT_IMAGE_HEIGHT 1080
#define INPUT_IMAGE_WIDTH 1920
#endif

double getCurrentTime()
{
   struct timeval tv;
   gettimeofday(&tv,NULL);
   return tv.tv_sec*1000 + tv.tv_usec/1000.0;
}
/*
typedef struct OBJ_INFO_
{
    int x;
    int y;
    int width;
    int height;
    int obj_class;
    float obj_confidence;
}Obj_Info;


static void ipl_into_image(OSAL_UCHAR* pDstData, Mat* src)
{
    unsigned char *data = (unsigned char *)src->data;
    int h = src->rows;
    int w = src->cols;
    int c = src->channels();
    int step = src->cols*src->channels();
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                pDstData[k*w*h + i*w + j] = data[i*step + j*c + k];
            }
        }
    }
}
*/
int main(int argc,char **argv)
{
    /* 浠诲姟鍙ユ焺 */
    OSAL_HANDLE m_OcrHandle = NULL;
    SDKErrCode iret ;
    /*杈撳嚭缁撴灉*/
    //PObjectOut_Result pTrackResult = NULL;
    POutOcrResult Ocr_result = NULL;
    cv::Mat Input = imread(INPUT_IMAGE_PATH);
    /* 杈撳叆鍥惧儚灏哄�� */
    OSAL_INT32 nImageW  = Input.cols;
    OSAL_INT32 nImageH = Input.rows;
    OSAL_UCHAR * pu8VirAddr;
    /* 娴锋€�3559妯″瀷璺�寰� */
    char modefile[256]=OCR_NET_PATH;  //妫€娴嬫ā鍨嬭矾寰�
    char modefile_class[256]=OCR_CLASS_PATH;

    char labelfile[256] = OCR_LABELS_PATH;
    //printf("**********debug***%d****%d****%d\n",__LINE__,nImageW,nImageH);
    /* 绠楁硶鍒涘缓 */
    iret=OcrDetectionCreate(&m_OcrHandle,modefile,modefile_class,labelfile,nImageW,nImageH,1);

    if (0 != iret)
    {
            printf("------------MV_VideoTrackTaskCreate error------------\n");
    }
    //鑾峰彇绠楁硶鐗堟湰
    const char * algorithm_version = OcrDetectionAlgorithmVersion();
    if(algorithm_version)
    {
        printf("Algorithm version:%s\n", algorithm_version);
    }

	double total_time=0.0;

    pu8VirAddr = Input.data;
    ALGORITHM_FRAME_INFO_S stAlgFrameInfo;
#if 0
        cv::Mat srcImg(INPUT_IMAGE_HEIGHT, INPUT_IMAGE_WIDTH, CV_8UC3);
        for(int i = 0; i < srcImg.rows; i++)
        {
            for(int j = 0; j < srcImg.cols; j++)
            {
                srcImg.data[i * srcImg.step[0] + 3 * j]	       = *(pu8VirAddr+i * INPUT_IMAGE_WIDTH * 3 + j * 3 + 0);
                srcImg.data[i * srcImg.step[0] + 3 * j + 1] = *(pu8VirAddr+i * INPUT_IMAGE_WIDTH * 3 + j * 3 + 1);
                srcImg.data[i * srcImg.step[0] + 3 * j + 2] = *(pu8VirAddr+i * INPUT_IMAGE_WIDTH * 3 + j * 3 + 2);
            }
        }
        imwrite("./srcImg_mainsrc.jpg", srcImg);
#endif
	double start=getCurrentTime();
    //printf("****%d******debug******%d*****%d\n",__LINE__,nImageW,nImageH);
    iret=OcrDetectionProcessFrame(m_OcrHandle,pu8VirAddr, &stAlgFrameInfo, nImageW,nImageH,IMAGE_TYPE_BGR_PACKAGE,&Ocr_result);
    //printf("**********debug***********%d\n",__LINE__);
    double end=getCurrentTime();
    printf("each image detect time  ========================  %lfms       \n",end-start);

    total_time=end-start;

    printf("*type:%s*\n", Ocr_result->type);
    printf("*num:%s*\n", Ocr_result->num);
    printf("*nation:%s*\n", Ocr_result->nation);
    printf("*full_name:%s-%s*\n", Ocr_result->full_name.Surname,Ocr_result->full_name.name);
    printf("*sex:%s*\n", Ocr_result->sex);
    printf("*birthData:%s*\n", Ocr_result->birthData);
    printf("*validData:%s*\n", Ocr_result->validData);
    printf("*machineData:%s*\n", Ocr_result->machineData);
    printf("*ocrs_character0:%s*\n", Ocr_result->ocrs_character[0]);
    printf("*ocrs_character1:%s*\n", Ocr_result->ocrs_character[1]);

    printf("average each image detect time  ========================  %lfms  \n",total_time);

    //capture.release();
    printf("------------total video success------------\n");
    /* 绠楁硶閫€鍑� */
    OcrDetectionFree(&m_OcrHandle);
    return 0;
}


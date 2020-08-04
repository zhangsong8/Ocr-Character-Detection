#ifndef SDKCOMMON_H
#define SDKCOMMON_H

/*input data type*/
typedef enum eImageDataType
{
    IMAGE_TYPE_BGR_PACKAGE            =  0,  /* BGR BGR BGR */
    IMAGE_TYPE_YUV                    =  1,  /* YUV */
    IMAGE_TYPE_GRAY                   =  2,  /* GRA Y */
    IMAGE_TYPE_YVU                    =  3,  /* YVU420SP */
    IMAGE_TYPE_BGR_PLANAR             =  4,  /* BBB GGG RRR*/
    IMAGE_TYPE_OTHER                  = -1   /* OTHER */
} ImageDataType;

/*error code*/
typedef enum
{
    SDK_ERR_NONE                      =  0,
    SDK_ERR_NOINIT                    = -1,
    SDK_ERR_PARAM                     = -2,
    SDK_ERR_FULLTASK                  = -3,
    SDK_ERR_ALLOC                     = -4,
    SDK_ERR_NOBUFF                    = -5,
    SDK_ERR_LICENSE                   = -6,
    SDK_ERR_NCS                       = -7,
    SDK_ERR_NCS_GET                   = -8,
    SDK_ERR_NCS_OPEN                  = -9,
    SDK_ERR_NCS_ALLOC                 = -10,
    SDK_ERR_NCS_2                     = -11,
    SDK_ERR_GRAPH                     = -12,
    SDK_ERR_OTHER                     = -999
} SDKErrCode;


#endif // SDKCOMMON_H

#ifndef HIIMAGETYPE_H
#define HIIMAGETYPE_H

typedef struct aiVIDEO_FRAME_S
{
	OSAL_UINT32 u32Width;
	OSAL_UINT32 u32Height;
	OSAL_UINT32 u32Stride[3];
	OSAL_UINT64 u64PhyAddr[3];
	OSAL_UINT64 u64VirAddr[3];
} ALGORITHM_FRAME_S;

typedef struct aiVIDEO_FRAME_INFO_S
{
	ALGORITHM_FRAME_S stAlgFrame;
	OSAL_UINT32       u32PoolId;
	OSAL_INT32        s32ModId;
} ALGORITHM_FRAME_INFO_S; 


#endif // HIIMAGETYPE_H
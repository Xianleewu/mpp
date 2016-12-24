/*
 * Copyright 2016 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __RK_MPI_VENC_H__
#define __RK_MPI_VENC_H__

#include "rk_mpi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _RkMpiVencContext {
	MppCtx						mpiCtx;
	MppApi						*mpi;
	MppCodingType				encType;
	MppEncConfig				encCfg;
} RkMpiVencContext;


MPP_RET RK_MPI_VENC_Create(RkMpiVencContext **context, MppCodingType encType) {
	MPP_RET ret = MPP_OK;

	RkMpiVencContext *pCtx = (RkMpiVencContext *)malloc(sizeof(RkMpiVencContext));
	memset((void*)pCtx, 0, sizeof(RkMpiVencContext));
	pCtx->encType = encType;
	
	ret = mpp_create(&pCtx->mpiCtx, &pCtx->mpi);

	if (MPP_OK != ret) {
        mpp_err("mpp_create failed\n");
        return ret;
    }

	ret = mpp_init(pCtx->mpiCtx, MPP_CTX_ENC, encType);
	if (MPP_OK != ret) {
		mpp_err("mpp_init failed\n");
		return ret;
	}

	*context = pCtx;
	return MPP_OK;
}

MPP_RET RK_MPI_VENC_Destory(RkMpiVencContext *context) {
	if (context->mpiCtx) {
        mpp_destroy(context->mpiCtx);
        context->mpiCtx = NULL;
    }
	return MPP_OK;
}

MPP_RET RK_MPI_VENC_SendFrame(RkMpiVencContext *context, MppBuffer packetOut, MppBuffer frameIn, MppBuffer mv) {
	return MPP_OK;
}

MPP_RET RK_MPI_VENC_GetStream(RkMpiVencContext *context, MppBuffer *packetOut) {
	return MPP_OK;
}

MPP_RET RK_MPI_VENC_EncoderOneFrame(RkMpiVencContext *context, MppBuffer packetOut, MppBuffer frameIn, MppBuffer mv) {
	MPP_RET ret = MPP_OK;
	MppFrame  frame = NULL;
    MppPacket packet = NULL;
	MppTask task = NULL;
	
	ret = mpp_frame_init(&frame);
    if (MPP_OK != ret) {
        mpp_err("mpp_frame_init failed\n");
        return ret;
    }

    mpp_frame_set_width(frame, context->encCfg.width);
    mpp_frame_set_height(frame, context->encCfg.height);
    mpp_frame_set_hor_stride(frame, MPP_ALIGN(context->encCfg.width, 16));
    mpp_frame_set_ver_stride(frame, MPP_ALIGN(context->encCfg.height, 16));

	mpp_frame_set_buffer(frame, frameIn);

    mpp_packet_init_with_buffer(&packet, packetOut);

    do {
        ret = context->mpi->dequeue(context->mpiCtx, MPP_PORT_INPUT, &task);
        if (ret) {
            mpp_err("mpp task input dequeue failed\n");
            return ret;
        }
        if (task == NULL) {
            mpp_log("mpi dequeue from MPP_PORT_INPUT fail, task equal with NULL!");
            msleep(3);
        } else {
            break;
        }
    } while (1);


    mpp_task_meta_set_frame (task, KEY_INPUT_FRAME,  frame);
    mpp_task_meta_set_packet(task, KEY_OUTPUT_PACKET, packet);
    mpp_task_meta_set_buffer(task, KEY_MOTION_INFO, mv);

    ret = context->mpi->enqueue(context->mpiCtx, MPP_PORT_INPUT, task);
    if (ret) {
        mpp_err("mpp task input enqueue failed\n");
        return ret;
    }

    do {
        ret = context->mpi->dequeue(context->mpiCtx, MPP_PORT_OUTPUT, &task);
        if (ret) {
            mpp_err("mpp task output dequeue failed\n");
            return ret;
        }

        if (task) {
            MppFrame packet_out = NULL;

            mpp_task_meta_get_packet(task, KEY_OUTPUT_PACKET, &packet_out);

            mpp_assert(packet_out == packet);

            ret = context->mpi->enqueue(context->mpiCtx, MPP_PORT_OUTPUT, task);
            if (ret) {
                mpp_err("mpp task output enqueue failed\n");
                return ret;
            }
            break;
        }
    } while (1);

	if (packet) {       
        mpp_packet_deinit(&packet);
    }
	if (frame) {
		mpp_frame_deinit(&frame);
	}
	return MPP_OK;
}

//pps and sps info
MPP_RET RK_MPI_VENC_GetExtraInfo(RkMpiVencContext *context, MppPacket *packet) {
	MPP_RET ret = MPP_OK;
	
	ret = context->mpi->control(context->mpiCtx, MPP_ENC_GET_EXTRA_INFO, packet);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc get extra info failed\n");
        return ret;
    }

	return MPP_OK;
}

MPP_RET RK_MPI_VENC_GetCfg(RkMpiVencContext *context, MppEncConfig *cfg) {
	MPP_RET ret = MPP_OK;
	
	ret = context->mpi->control(context->mpiCtx, MPP_ENC_GET_CFG, cfg);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc get config info failed\n");
        return ret;
    }

	return MPP_OK;
}

MPP_RET RK_MPI_VENC_SetCfg(RkMpiVencContext *context, MppEncConfig *cfg) {
	MPP_RET ret = MPP_OK;
	
	ret = context->mpi->control(context->mpiCtx, MPP_ENC_SET_CFG, cfg);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc set config info failed\n");
        return ret;
    }

	return MPP_OK;
}

MPP_RET RK_MPI_VENC_SetSeiMode(RkMpiVencContext *context, MppEncSeiMode *seiMode) {
	MPP_RET ret = MPP_OK;
	
	ret = context->mpi->control(context->mpiCtx, MPP_ENC_SET_SEI_CFG, seiMode);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc set sei failed\n");
        return ret;
    }

	return MPP_OK;
}

MPP_RET RK_MPI_VENC_SetOsdPlt(RkMpiVencContext *context, MppEncOSDPlt *osdPlt) {
	MPP_RET ret = MPP_OK;
	ret = context->mpi->control(context->mpiCtx, MPP_ENC_SET_OSD_PLT_CFG, osdPlt);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc set osd plt failed\n");
        return ret;
    }
	return MPP_OK;
}

MPP_RET RK_MPI_VENC_SetOsdData(RkMpiVencContext *context, MppEncOSDData *osdData) {
	MPP_RET ret = MPP_OK;
	ret = context->mpi->control(context->mpiCtx, MPP_ENC_SET_OSD_DATA_CFG, osdData);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc set osd plt failed\n");
        return ret;
    }
	return MPP_OK;
}

MPP_RET RK_MPI_VENC_RequestIDRInst(RkMpiVencContext *context) {
	MPP_RET ret = MPP_OK;
	ret = context->mpi->control(context->mpiCtx, MPP_ENC_SET_IDR_FRAME, NULL);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc set IDR failed\n");
        return ret;
    }
	return MPP_OK;
}


#if 0

MPP_RET RK_MPI_VENC_SetFrmRate();
MPP_RET RK_MPI_VENC_GetFrmRate();

MPP_RET RK_MPI_VENC_SetBitRate();
MPP_RET RK_MPI_VENC_GetBitRate();

MPP_RET RK_MPI_VENC_GetRcPara();
MPP_RET RK_MPI_VENC_SetRcPara();

MPP_RET RK_MPI_VENC_SetH264eRefMode();
MPP_RET RK_MPI_VENC_GetH264eRefMode();

MPP_RET RK_MPI_VENC_SetH264eRefParam();
MPP_RET RK_MPI_VENC_GetH264eRefParam();

MPP_RET RK_MPI_VENC_SetRoiCfg();
MPP_RET RK_MPI_VENC_GetRoiCfg();

MPP_RET RK_MPI_VENC_SetH264SliceSplit();
MPP_RET RK_MPI_VENC_GetH264SliceSplit();

MPP_RET RK_MPI_VENC_SetH264InterPred();
MPP_RET RK_MPI_VENC_GetH264InterPred();

MPP_RET RK_MPI_VENC_SetH264IntraPred();
MPP_RET RK_MPI_VENC_GetH264IntraPred();

MPP_RET RK_MPI_VENC_SetH264Trans();
MPP_RET RK_MPI_VENC_GetH264Trans();

MPP_RET RK_MPI_VENC_SetH264Entropy();
MPP_RET RK_MPI_VENC_GetH264Entropy();

MPP_RET RK_MPI_VENC_SetH264Poc();
MPP_RET RK_MPI_VENC_GetH264Poc();

MPP_RET RK_MPI_VENC_SetH264Dblk();
MPP_RET RK_MPI_VENC_GetH264Dblk();

MPP_RET RK_MPI_VENC_SetH264Vui();
MPP_RET RK_MPI_VENC_GetH264Vui();

MPP_RET RK_MPI_VENC_SetColor2Grey();
MPP_RET RK_MPI_VENC_GetColor2Grey();

MPP_RET RK_MPI_VENC_SetColor2GreyConf();
MPP_RET RK_MPI_VENC_GetColor2GreyConf();

MPP_RET RK_MPI_VENC_SetCrop();
MPP_RET RK_MPI_VENC_GetCrop();

MPP_RET RK_MPI_VENC_SetRcPriority();
MPP_RET RK_MPI_VENC_GetRcPriority();

MPP_RET RK_MPI_VENC_SetLostFrameStrategy();
MPP_RET RK_MPI_VENC_GetLostFrameStrategy();
#endif

#ifdef __cplusplus
}
#endif

#endif

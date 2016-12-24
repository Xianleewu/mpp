/*
mediastreamer2 x264 plugin
Copyright (C) 2006-2010 Belledonne Communications SARL (simon.morlat@linphone.org)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#include "mediastreamer2/msfilter.h"
#include "mediastreamer2/msticker.h"
#include "mediastreamer2/msvideo.h"
#include "mediastreamer2/rfc3984.h"

#include <mpp/mpp_buffer.h>
#include <mpp/mpp_err.h>
#include <mpp/mpp_frame.h>
#include <mpp/mpp_meta.h>
#include <mpp/mpp_packet.h>
#include <mpp/mpp_task.h>
#include <mpp/rk_mpi.h>
#include <mpp/rk_mpi_cmd.h>
#include <mpp/rk_type.h>
#include <mpp/vpu_api.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef _MSC_VER
#include <stdint.h>
#endif

#ifndef VERSION
#define VERSION "1.4.1"
#endif


#define RC_MARGIN 10000 /*bits per sec*/
#define SPECIAL_HIGHRES_BUILD_CRF 28


typedef struct _EncData {
	MppCtx	ctx;
	MppApi	*mpi;

	char	*buf;

	/* input and output */
    MppCodingType   type;
    MppEncConfig    enc_cfg;
	MppBufferGroup	mpp_grp;
	MppBufferGroup	pkt_grp;
	MppPacket		packet;
	size_t			packet_size
	MppFrame		frame;

	int mode;
	uint64_t framenum;
	Rfc3984Context *packer;
	int keyframe_int;
	VideoStarter starter;
	MSVideoConfiguration vconf;
	bool_t generate_keyframe;
}EncData;


/* the goal of this small object is to tell when to send I frames at startup:
at 2 and 4 seconds*/
typedef struct VideoStarter{
	uint64_t next_time;
	int i_frame_count;
}VideoStarter;

static void video_starter_init(VideoStarter *vs){
	vs->next_time=0;
	vs->i_frame_count=0;
}

static void video_starter_first_frame(VideoStarter *vs, uint64_t curtime){
	vs->next_time=curtime+2000;
}

static bool_t video_starter_need_i_frame(VideoStarter *vs, uint64_t curtime){
	if (vs->next_time==0) return FALSE;
	if (curtime>=vs->next_time){
		vs->i_frame_count++;
		if (vs->i_frame_count==1){
			vs->next_time+=2000;
		}else{
			vs->next_time=0;
		}
		return TRUE;
	}
	return FALSE;
}

static void enc_init(MSFilter *f){
	EncData *d=ms_new(EncData,1);

    memset(d, NULL, sizeof(EncData));

	d->mpi = NULL;
	d->mpp_cfg = NULL;
	d->frm_grp = NULL;
	d->pkt_grp = NULL;
	d->frame = NULL;
	d->packet = NULL;

	d->frm_buf[MPI_ENC_IO_COUNT] = { NULL };
	d->pkt_buf[MPI_ENC_IO_COUNT] = { NULL };
	d->md_buf[MPI_ENC_IO_COUNT] = { NULL };
	d->osd_idx_buf[MPI_ENC_IO_COUNT] = { NULL };
	d->osd_plt = NULL;
	d->sei_mode = MPP_ENC_SEI_MODE_ONE_SEQ;
	d->hor_stride = MPP_ALIGN(width, 16);
	d->ver_stride = MPP_ALIGN(height, 16);
	d->fmt = formate;
	d->type = MPP_VIDEO_CodingAVC;

	d->keyframe_int=10; /*10 seconds */
	d->mode=0;
	d->framenum=0;
	d->generate_keyframe=FALSE;
	d->packer=NULL;
    /* some default parameters */
	d->vconf.bitrate_limit = 0;
    d->vconf.fps = 30.0;
    d->vconf.mincpu = 1;
    d->vconf.required_bitrate = 38400;
    d->vconf.vsize = (MSVideoSize) {640, 480};

	f->data=d;
}

static void enc_uninit(MSFilter *f){
	EncData *d=(EncData*)f->data;
	ms_free(d);
}

static void apply_bitrate(MSFilter *f){
	EncData *d=(EncData*)f->data;
	float bitrate;

	bitrate=(float)d->vconf.required_bitrate*0.92;
	if (bitrate>RC_MARGIN)
		bitrate-=RC_MARGIN;

    //TODO: re config mpp bitrate here
}

static void enc_preprocess(MSFilter *f){
	EncData *d=(EncData*)f->data;
    int i;
    int ret = -1;

	mpp_buffer_group_get_internal(&d->frm_grp, MPP_BUFFER_TYPE_ION);

	mpp_buffer_group_get_internal(&d->pkt_grp, MPP_BUFFER_TYPE_ION);

	for (i = 0; i < MPI_ENC_IO_COUNT; i++) {
        ret = mpp_buffer_get(d->frm_grp, &d->frm_buf[i], d->frame_size);
        if (ret) {
            ms_error("failed to get buffer for input frame ret %d\n", ret);
            //goto MPP_TEST_OUT;
        }

        ret = mpp_buffer_get(d->frm_grp, &->osd_idx_buf[i], d->osd_idx_size);
        if (ret) {
            ms_error("failed to get buffer for osd idx buf ret %d\n", ret);
            //goto MPP_TEST_OUT;
        }

        ret = mpp_buffer_get(d->pkt_grp, &d->pkt_buf[i], d->packet_size);
        if (ret) {
            ms_error("failed to get buffer for input frame ret %d\n", ret);
            //goto MPP_TEST_OUT;
        }

        ret = mpp_buffer_get(d->pkt_grp, &d->md_buf[i], d->mdinfo_size);
        if (ret) {
            ms_error("failed to get buffer for motion detection info ret %d\n", ret);
            //goto MPP_TEST_OUT;
        }
    }

	ret = mpp_create(&d->ctx, d->mpi);

    if(ret) {
        ms_error("creat mpp encode context failed");
    }

	ret = mpp_init(d->ctx, MPP_CTX_ENC, d->mode);

    if(ret) {
        ms_error("mpp encode contex init failed");
    }

	memset(&d->mpp_cfg, 0, sizeof(d->mpp_cfg));
    d->mpp_cfg.size        = sizeof(d->mpp_cfg);
    d->mpp_cfg.width       = width;
    d->mpp_cfg.height      = height;
    d->mpp_cfg.hor_stride  = hor_stride;
    d->mpp_cfg.ver_stride  = ver_stride;
    d->mpp_cfg.format      = fmt;
    d->mpp_cfg.rc_mode     = 0;
    d->mpp_cfg.skip_cnt    = 0;
    d->mpp_cfg.fps_in      = 30;
    d->mpp_cfg.fps_out     = 30;
    d->mpp_cfg.bps         = width * height * 2 * mpp_cfg.fps_in;
    d->mpp_cfg.qp          = (type == MPP_VIDEO_CodingMJPEG) ? (10) : (24);
    d->mpp_cfg.gop         = 60;

    d->mpp_cfg.profile     = 100;
    d->mpp_cfg.level       = 41;
    d->mpp_cfg.cabac_en    = 1;

    ret = mpi->control(d->ctx, MPP_ENC_SET_SEI_CFG, &d->sei_mode);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc set sei cfg failed\n");
        goto MPP_TEST_OUT;
    }

    ret = mpi->control(d->ctx, MPP_ENC_SET_CFG, &d->mpp_cfg);
    if (MPP_OK != ret) {
        mpp_err("mpi control enc set cfg failed\n");
        goto MPP_TEST_OUT;
    }

	d->packer=rfc3984_new();
	rfc3984_set_mode(d->packer,d->mode);
	rfc3984_enable_stap_a(d->packer,FALSE);

	apply_bitrate(f);

	params->rc.i_lookahead=0;
	/*enable this by config ?*/
	/*
	 params.i_keyint_max = (int)d->fps*d->keyframe_int;
	 params.i_keyint_min = (int)d->fps;
	 */

	d->framenum=0;
	video_starter_init(&d->starter);
}

static void x264_nals_to_msgb(x264_nal_t *xnals, int num_nals, MSQueue * nalus){
	int i;
	mblk_t *m;
	/*int bytes;*/
	for (i=0;i<num_nals;++i){
		m=allocb(xnals[i].i_payload+10,0);

		memcpy(m->b_wptr,xnals[i].p_payload+4,xnals[i].i_payload-4);
		m->b_wptr+=xnals[i].i_payload-4;
		if (xnals[i].i_type==7) {
			ms_message("A SPS is being sent.");
		}else if (xnals[i].i_type==8) {
			ms_message("A PPS is being sent.");
		}
		ms_queue_put(nalus,m);
	}
}

static void enc_process(MSFilter *f){
	EncData *d=(EncData*)f->data;
	uint32_t ts=f->ticker->time*90LL;
	mblk_t *im;
	MSPicture pic;
	MSQueue nalus;

	if (d->enc==NULL){
		ms_queue_flush(f->inputs[0]);
		return;
	}

	ms_queue_init(&nalus);
	while((im=ms_queue_get(f->inputs[0]))!=NULL){
		if (ms_yuv_buf_init_from_mblk(&pic,im)==0){
			x264_picture_t xpic;
			x264_picture_t oxpic;
			x264_nal_t *xnals=NULL;
			int num_nals=0;

			memset(&xpic, 0, sizeof(xpic));
			memset(&oxpic, 0, sizeof(oxpic));

			/*send I frame 2 seconds and 4 seconds after the beginning */
			if (video_starter_need_i_frame(&d->starter,f->ticker->time))
				d->generate_keyframe=TRUE;

			if (d->generate_keyframe){
				xpic.i_type=X264_TYPE_IDR;
				d->generate_keyframe=FALSE;
			}else xpic.i_type=X264_TYPE_AUTO;
			xpic.i_qpplus1=0;
			xpic.i_pts=d->framenum;
			xpic.param=NULL;
			xpic.img.i_csp=X264_CSP_I420;
			xpic.img.i_plane=3;
			xpic.img.i_stride[0]=pic.strides[0];
			xpic.img.i_stride[1]=pic.strides[1];
			xpic.img.i_stride[2]=pic.strides[2];
			xpic.img.i_stride[3]=0;
			xpic.img.plane[0]=pic.planes[0];
			xpic.img.plane[1]=pic.planes[1];
			xpic.img.plane[2]=pic.planes[2];
			xpic.img.plane[3]=0;

			if (x264_encoder_encode(d->enc,&xnals,&num_nals,&xpic,&oxpic)>=0){
				x264_nals_to_msgb(xnals,num_nals,&nalus);
				/*if (num_nals == 0)	ms_message("Delayed frames info: current=%d max=%d\n",
					x264_encoder_delayed_frames(d->enc),
					x264_encoder_maximum_delayed_frames(d->enc));
				*/
				rfc3984_pack(d->packer,&nalus,f->outputs[0],ts);
				if (d->framenum==0)
					video_starter_first_frame(&d->starter,f->ticker->time);
				d->framenum++;
			}else{
				ms_error("x264_encoder_encode() error.");
			}
		}
		freemsg(im);
	}
}

static void enc_postprocess(MSFilter *f){
	EncData *d=(EncData*)f->data;
	rfc3984_destroy(d->packer);
	d->packer=NULL;
	if (d->enc!=NULL){
		x264_encoder_close(d->enc);
		d->enc=NULL;
	}
}

static int enc_get_br(MSFilter *f, void*arg){
	EncData *d=(EncData*)f->data;
	*(int*)arg=d->vconf.required_bitrate;
	return 0;
}

static int enc_set_configuration(MSFilter *f, void *arg) {
	EncData *d = (EncData *)f->data;
	const MSVideoConfiguration *vconf = (const MSVideoConfiguration *)arg;
	if (vconf != &d->vconf) memcpy(&d->vconf, vconf, sizeof(MSVideoConfiguration));

	if (d->vconf.required_bitrate > d->vconf.bitrate_limit)
		d->vconf.required_bitrate = d->vconf.bitrate_limit;
	if (d->enc) {
		ms_filter_lock(f);
		apply_bitrate(f);
		if (x264_encoder_reconfig(d->enc, &d->params) != 0) {
			ms_error("x264_encoder_reconfig() failed.");
		}
		ms_filter_unlock(f);
		return 0;
	}

	ms_message("Video configuration set: bitrate=%dbits/s, fps=%f, vsize=%dx%d", d->vconf.required_bitrate, d->vconf.fps, d->vconf.vsize.width, d->vconf.vsize.height);
	return 0;
}

static int enc_set_br(MSFilter *f, void *arg) {
	EncData *d = (EncData *)f->data;
	int br = *(int *)arg;
	if (d->enc != NULL) {
		/* Encoding is already ongoing, do not change video size, only bitrate. */
		d->vconf.required_bitrate = br;
		enc_set_configuration(f,&d->vconf);
	} else {
		MSVideoConfiguration best_vconf = ms_video_find_best_configuration_for_bitrate(d->vconf_list, br, ms_get_cpu_count());
		enc_set_configuration(f, &best_vconf);
	}
	return 0;
}

static int enc_set_fps(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	d->vconf.fps=*(float*)arg;
	enc_set_configuration(f, &d->vconf);
	return 0;
}

static int enc_get_fps(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	*(float*)arg=d->vconf.fps;
	return 0;
}

static int enc_get_vsize(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	*(MSVideoSize*)arg=d->vconf.vsize;
	return 0;
}

static int enc_set_vsize(MSFilter *f, void *arg){
	MSVideoConfiguration best_vconf;
	EncData *d = (EncData *)f->data;
	MSVideoSize *vs = (MSVideoSize *)arg;
	best_vconf = ms_video_find_best_configuration_for_size(d->vconf_list, *vs, ms_get_cpu_count());
	d->vconf.vsize = *vs;
	d->vconf.fps = best_vconf.fps;
	d->vconf.bitrate_limit = best_vconf.bitrate_limit;
	enc_set_configuration(f, &d->vconf);
	return 0;
}

static int enc_add_fmtp(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	const char *fmtp=(const char *)arg;
	char value[12];
	if (fmtp_get_value(fmtp,"packetization-mode",value,sizeof(value))){
		d->mode=atoi(value);
		ms_message("packetization-mode set to %i",d->mode);
	}
	return 0;
}

static int enc_req_vfu(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	d->generate_keyframe=TRUE;
	return 0;
}

static int enc_get_configuration_list(MSFilter *f, void *data) {
	EncData *d = (EncData *)f->data;
	const MSVideoConfiguration **vconf_list = (const MSVideoConfiguration **)data;
	*vconf_list = d->vconf_list;
	return 0;
}


static MSFilterMethod enc_methods[] = {
	{ MS_FILTER_SET_FPS,                       enc_set_fps                },
	{ MS_FILTER_SET_BITRATE,                   enc_set_br                 },
	{ MS_FILTER_GET_BITRATE,                   enc_get_br                 },
	{ MS_FILTER_GET_FPS,                       enc_get_fps                },
	{ MS_FILTER_GET_VIDEO_SIZE,                enc_get_vsize              },
	{ MS_FILTER_SET_VIDEO_SIZE,                enc_set_vsize              },
	{ MS_FILTER_ADD_FMTP,                      enc_add_fmtp               },
	{ MS_FILTER_REQ_VFU,                       enc_req_vfu                },
	{ MS_VIDEO_ENCODER_REQ_VFU,                enc_req_vfu                },
	{ MS_VIDEO_ENCODER_GET_CONFIGURATION_LIST, enc_get_configuration_list },
	{ MS_VIDEO_ENCODER_SET_CONFIGURATION,      enc_set_configuration      },
	{ 0,                                       NULL                       }
};

#ifndef _MSC_VER

static MSFilterDesc x264_enc_desc={
	.id=MS_FILTER_PLUGIN_ID,
	.name="MSX264Enc",
	.text="A H264 encoder based on x264 project",
	.category=MS_FILTER_ENCODER,
	.enc_fmt="H264",
	.ninputs=1,
	.noutputs=1,
	.init=enc_init,
	.preprocess=enc_preprocess,
	.process=enc_process,
	.postprocess=enc_postprocess,
	.uninit=enc_uninit,
	.methods=enc_methods
};

#else

static MSFilterDesc x264_enc_desc={
	MS_FILTER_PLUGIN_ID,
	"MSX264Enc",
	"A H264 encoder based on x264 project",
	MS_FILTER_ENCODER,
	"H264",
	1,
	1,
	enc_init,
	enc_preprocess,
	enc_process,
	enc_postprocess,
	enc_uninit,
	enc_methods
};

#endif

MS2_PUBLIC void libmsx264_init(void){
	ms_filter_register(&x264_enc_desc);
	ms_message("msx264-" VERSION " plugin registered.");
}


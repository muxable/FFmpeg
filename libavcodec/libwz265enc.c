/*
 * libwz265 encoder
 *
 * Copyright (c) 2013-2014 Derek Buitenhuis
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <float.h>
#include "wz265/wz265enc.h"
//#include "wzauth_env.h"

#include "libavutil/internal.h"
#include "libavutil/common.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avcodec.h"
#include "internal.h"
#include "packet_internal.h"

typedef struct libwz265Context
{
  const AVClass *class;

  void *encoder;
  WZ265EncConfig *params;

  float crf;
  int aq_mode;
  float aq_strength;
  float aq_smooth;
  float psy_rd;
  int psy;
  char *preset;
  char *tune;
  double duration;
  char *wz265_opts;
  int vui_time;
} libwz265Context;

static int is_keyframe(NAL_UNIT_TYPE naltype)
{
  switch (naltype)
  {
  case NAL_UNIT_TYPE_BLA_W_LP:
  case NAL_UNIT_TYPE_BLA_W_RADL:
  case NAL_UNIT_TYPE_BLA_N_LP:
  case NAL_UNIT_TYPE_IDR_W_RADL:
  case NAL_UNIT_TYPE_IDR_N_LP:
  case NAL_UNIT_TYPE_CRA_NUT:
    return 1;
  default:
    return 0;
  }
}

static av_cold int libwz265_encode_close(AVCodecContext *avctx)
{
  libwz265Context *ctx = avctx->priv_data;

  av_freep(&avctx->extradata);
  if (ctx->params)
  {
    av_free(ctx->params);
  }

  if (ctx->encoder)
    wz265_encoder_close(ctx->encoder);
  return 0;
}

static av_cold int libwz265_encode_init(AVCodecContext *avctx)
{
  libwz265Context *ctx = avctx->priv_data;
  WZ265Nal *nal;
  char sar[12];
  int sar_num, sar_den;
  int nnal;
  int ret = 0;
  if (avctx->strict_std_compliance > FF_COMPLIANCE_EXPERIMENTAL &&
      !av_pix_fmt_desc_get(avctx->pix_fmt)->log2_chroma_w)
  {
    av_log(avctx, AV_LOG_ERROR,
           "4:2:2 and 4:4:4 support is not fully defined for HEVC yet. "
           "Set -strict experimental to encode anyway.\n");
    return AVERROR(ENOSYS);
  }

  ctx->params = (WZ265EncConfig *)(av_mallocz(sizeof(WZ265EncConfig)));
  if (!ctx->params)
  {
    av_log(avctx, AV_LOG_ERROR, "Could not allocate wz265 param structure.\n");
    return AVERROR(ENOMEM);
  }

  if (wz265_param_default_preset(ctx->params, ctx->preset, ctx->tune) < 0)
  {
    av_log(avctx, AV_LOG_ERROR, "Invalid preset or tune.\n");
    return AVERROR(EINVAL);
  }
  // ctx->params->pAuth = avctx->opaque;
  ctx->params->threads = avctx->thread_count;
  ctx->params->frameRate =
      avctx->time_base.den / (double)(avctx->time_base.num * avctx->ticks_per_frame);
  ctx->params->picWidth = avctx->width;
  ctx->params->picHeight = avctx->height;
  // ctx->params->bEnablePsnr     = !!(avctx->flags & CODEC_FLAG_PSNR);

  switch (avctx->pix_fmt)
  {
  case AV_PIX_FMT_YUV420P:
  case AV_PIX_FMT_YUVJ420P:
    ctx->params->inputCsp = WZ265_CSP_I420;
    ctx->params->inputBitDepth = 8;
    break;
  case AV_PIX_FMT_YUV420P9LE:
  case AV_PIX_FMT_YUV420P10LE:
    ctx->params->inputCsp = WZ265_CSP_P010;
    ctx->params->inputBitDepth = 10;
    break;
  default:
    break;
  }

  if (avctx->sample_aspect_ratio.num > 0 && avctx->sample_aspect_ratio.den > 0)
  {
    av_reduce(&sar_num, &sar_den, avctx->sample_aspect_ratio.num, avctx->sample_aspect_ratio.den,
              65535);
    snprintf(sar, sizeof(sar), "%d:%d", sar_num, sar_den);
    // to do , not support yet
    /*
    if (wz265_param_parse(ctx->params, "sar", sar) == wz265_PARAM_BAD_VALUE) {
    av_log(avctx, AV_LOG_ERROR, "Invalid SAR: %d:%d.\n", sar_num, sar_den);
    return AVERROR_INVALIDDATA;
    }
    */
  }

  if (ctx->crf >= 0)
  {
    ctx->params->crf = ctx->crf;
    ctx->params->rc = WZ265_RC_CRF; // crf
  }
  if (avctx->bit_rate > 0)
  {
    ctx->params->bitrateInkbps = avctx->bit_rate / 1000;
    ctx->params->rc = WZ265_RC_ABR; // ABR
  }
  ctx->params->vbv_buffer_size = avctx->rc_buffer_size / 1000;
  ctx->params->vbv_max_rate = avctx->rc_max_rate / 1000;

  if (avctx->gop_size >= 0)
    ctx->params->keyint_max = avctx->gop_size;
  if (avctx->keyint_min >= 0)
    ctx->params->keyint_min = avctx->keyint_min;
  if (avctx->max_b_frames >= 0)
    ctx->params->bframes = avctx->max_b_frames;
  if (avctx->scenechange_threshold >= 0)
    ctx->params->scenecut = avctx->scenechange_threshold;

  if (avctx->qmin >= 0)
    ctx->params->qpmin = avctx->qmin;
  if (avctx->qmax >= 0)
    ctx->params->qpmax = avctx->qmax;
  if (avctx->refs >= 0)
    ctx->params->refnum = avctx->refs;
  if (avctx->trellis >= 0)
    ctx->params->rdoq = avctx->trellis;
  if (avctx->me_range >= 0)
    ctx->params->searchrange = avctx->me_range;
  if (avctx->me_range >= 0)
    ctx->params->searchrange = avctx->me_range;
  if (avctx->noise_reduction >= 0)
    ctx->params->noiseReduction = avctx->noise_reduction;

  ctx->params->calcPsnr = !!(avctx->flags & AV_CODEC_FLAG_PSNR);
  ctx->params->opengop = !(avctx->flags & AV_CODEC_FLAG_CLOSED_GOP);

  ctx->params->bHeaderBeforeKeyframe = 1;

//  if (ctx->params->threads > 1)
//  {
//    ctx->params->enWavefront = 1;
//    ctx->params->enFrameParallel = 1;
//  }
  if (ctx->vui_time)
  {
    ctx->params->vui_parameters_present_flag = 1;
    ctx->params->vui.vui_timing_info_present_flag = 1;
    ctx->params->vui.vui_num_units_in_tick = avctx->time_base.num * avctx->ticks_per_frame;
    ctx->params->vui.vui_time_scale = avctx->time_base.den;
  }

  if (avctx->pix_fmt == AV_PIX_FMT_YUVJ420P || avctx->color_range == AVCOL_RANGE_JPEG)
  {
    ctx->params->vui_parameters_present_flag = 1;
    ctx->params->vui.video_signal_type_present_flag = 1;
    ctx->params->vui.video_format = 5;
    ctx->params->vui.video_full_range_flag = 1;
  }

  if ((avctx->color_primaries <= AVCOL_PRI_SMPTE432 &&
       avctx->color_primaries != AVCOL_PRI_UNSPECIFIED) ||
      (avctx->color_trc <= AVCOL_TRC_ARIB_STD_B67 && avctx->color_trc != AVCOL_TRC_UNSPECIFIED) ||
      (avctx->colorspace <= AVCOL_SPC_ICTCP && avctx->colorspace != AVCOL_SPC_UNSPECIFIED))
  {
    ctx->params->vui_parameters_present_flag = 1;
    ctx->params->vui.video_signal_type_present_flag = 1;
    ctx->params->vui.video_format = 5;
    ctx->params->vui.video_full_range_flag =
        avctx->pix_fmt == AV_PIX_FMT_YUVJ420P || avctx->color_range == AVCOL_RANGE_JPEG;

    ctx->params->vui.colour_description_present_flag = 1;
    ctx->params->vui.colour_primaries = avctx->color_primaries;
    ctx->params->vui.transfer_characteristics = avctx->color_trc;
    ctx->params->vui.matrix_coeffs = avctx->colorspace;
  }
  if (ctx->aq_mode >= 0)
    ctx->params->iAqMode = ctx->aq_mode;
  if (ctx->aq_strength >= 0)
    ctx->params->fAqStrength = ctx->aq_strength;
  if (ctx->aq_smooth >= 0)
    ctx->params->fAqSmooth = ctx->aq_smooth;
  if (ctx->psy_rd >= 0)
    ctx->params->psyRd = ctx->psy_rd;
  if (ctx->psy >= 0)
    ctx->params->psyRdUV = ctx->psy;
  if (ctx->duration >= 0)
    ctx->params->duration = ctx->duration;

  if (ctx->wz265_opts)
  {
    AVDictionary *dict = NULL;
    AVDictionaryEntry *en = NULL;

    av_log(avctx, AV_LOG_INFO, "WZ265Config %s \n", ctx->wz265_opts);

    if (!av_dict_parse_string(&dict, ctx->wz265_opts, "=", ":", 0))
    {
      while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX)))
      {
        int parse_ret = WZ265ConfigParse(ctx->params, en->key, en->value);

        switch (parse_ret)
        {
        case WZ265_PARAM_BAD_NAME:
          av_log(avctx, AV_LOG_WARNING, "Unknown option: %s.\n", en->key);
          break;
        case WZ265_PARAM_BAD_VALUE:
          av_log(avctx, AV_LOG_WARNING, "Invalid value for %s: %s.\n", en->key, en->value);
          break;
        default:
          break;
        }
      }
      av_dict_free(&dict);
    }
  }

  av_log(avctx, AV_LOG_INFO, "WZ265Config %s %s.\n", ctx->preset, ctx->tune);
  av_log(avctx, AV_LOG_INFO, "input colors space %d bitdepth %d.\n", ctx->params->inputCsp, ctx->params->inputBitDepth);
  av_log(avctx, AV_LOG_INFO, "resolution %dx%d.\n", ctx->params->picWidth, ctx->params->picHeight);
  av_log(avctx, AV_LOG_INFO, "frameRate %f .\n", ctx->params->frameRate);
  av_log(avctx, AV_LOG_INFO, "rc %d .\n", ctx->params->rc);
  av_log(avctx, AV_LOG_INFO, "qp %d .\n", ctx->params->qp);
  av_log(avctx, AV_LOG_INFO, "crf %.1f .\n", ctx->params->crf);
  av_log(avctx, AV_LOG_INFO, "bitrateInkbps %d .\n", ctx->params->bitrateInkbps);
  av_log(avctx, AV_LOG_INFO, "vbv-maxrate %d .\n", ctx->params->vbv_max_rate);
  av_log(avctx, AV_LOG_INFO, "vbv-bufsize %d .\n", ctx->params->vbv_buffer_size);
  av_log(avctx, AV_LOG_INFO, "keyint %d .\n", ctx->params->keyint_max);
  av_log(avctx, AV_LOG_INFO, "qpmin %d .\n", ctx->params->qpmin);
  av_log(avctx, AV_LOG_INFO, "qpmax %d .\n", ctx->params->qpmax);
  av_log(avctx, AV_LOG_INFO, "enWavefront %d .\n", ctx->params->enWavefront);
  av_log(avctx, AV_LOG_INFO, "enFrameParallel %d .\n", ctx->params->enFrameParallel);
  av_log(avctx, AV_LOG_INFO, "threads %d .\n", ctx->params->threads);
  av_log(avctx, AV_LOG_INFO, "logLevel %d .\n", ctx->params->logLevel);
  av_log(avctx, AV_LOG_INFO, "bframes %d .\n", ctx->params->bframes);
  av_log(avctx, AV_LOG_INFO, "rc-lookahead %d .\n", ctx->params->lookahead);
  av_log(avctx, AV_LOG_INFO, "aq-mode %d .\n", ctx->params->iAqMode);
  av_log(avctx, AV_LOG_INFO, "aq-strength %5.1f .\n", ctx->params->fAqStrength);
  av_log(avctx, AV_LOG_INFO, "aq-weight %5.1f .\n", ctx->params->fAqWeight);
  av_log(avctx, AV_LOG_INFO, "psy-rd %5.1f .\n", ctx->params->psyRd);
  av_log(avctx, AV_LOG_INFO, "open-gop %d .\n", ctx->params->opengop);
  av_log(avctx, AV_LOG_INFO, "shortload %5.1f .\n", ctx->params->shortLoadingForPlayer);

  // set bit_rate for metadata
  avctx->bit_rate = (ctx->params->bitrateInkbps * 1000);

  if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)
    ctx->params->bHeaderBeforeKeyframe = 0;

  ctx->encoder = wz265_encoder_open(ctx->params, &ret);
  if (!ctx->encoder)
  {
    av_log(avctx, AV_LOG_ERROR, "Cannot open libwz265 encoder. %d\n", ret);
    libwz265_encode_close(avctx);
    return AVERROR_INVALIDDATA;
  }

  if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)
  {
    avctx->extradata_size = wz265_encode_headers(ctx->encoder, &nal, &nnal);
    av_log(avctx, AV_LOG_INFO, "wz265_encode_headers %d.\n", avctx->extradata_size);
    if (avctx->extradata_size <= 0)
    {
      av_log(avctx, AV_LOG_ERROR, "Cannot encode headers.\n");
      libwz265_encode_close(avctx);
      return AVERROR_INVALIDDATA;
    }

    avctx->extradata = av_malloc(avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!avctx->extradata)
    {
      av_log(avctx, AV_LOG_ERROR, "Cannot allocate HEVC header of size %d.\n",
             avctx->extradata_size);
      libwz265_encode_close(avctx);
      return AVERROR(ENOMEM);
    }

    memcpy(avctx->extradata, nal[0].pPayload, avctx->extradata_size);
  }

  return 0;
}

static int libwz265_encode_frame(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *pic,
                                 int *got_packet)
{
  libwz265Context *ctx = avctx->priv_data;
  WZ265YUV yuv;
  WZ265Picture wz265pic;
  WZ265Picture wz265pic_out;
  WZ265Nal *nal;
  uint8_t *dst;
  int payload_size = 0;
  int nnal = 0;
  int ret = 0;
  int i;

  memset(&wz265pic, 0, sizeof(wz265pic));
  wz265pic.yuv = &yuv;
  memset(&wz265pic_out, 0, sizeof(wz265pic_out));
  // wz265_picture_init(ctx->params, &wz265pic);

  if (pic)
  {
    wz265pic.yuv->iWidth = ctx->params->picWidth;
    wz265pic.yuv->iHeight = ctx->params->picHeight;
    wz265pic.yuv->iBitDepth = ctx->params->inputBitDepth;
    const int pixel_stride_shift = (ctx->params->inputBitDepth > 8) ? 1 : 0;
    for (i = 0; i < 3; i++)
    {
      wz265pic.yuv->pData[i] = pic->data[i];
      wz265pic.yuv->iStride[i] = pic->linesize[i] >> pixel_stride_shift;
    }
    wz265pic.pts = pic->pts;
    if (pic->opaque_ref)
    {
      wz265pic.roi = pic->opaque_ref->data;
    }

    if (ctx->params->bitrateInkbps != avctx->bit_rate / 1000)
    {
      ctx->params->bitrateInkbps = avctx->bit_rate / 1000;
      wz265_encoder_reconfig(ctx->encoder, ctx->params);
    }

    if (pic->pict_type == AV_PICTURE_TYPE_I)
    {
      wz265_keyframe_request(ctx->encoder);
    }
  }

  ret = wz265_encoder_frame(ctx->encoder, &nal, &nnal, pic ? &wz265pic : NULL, &wz265pic_out);

  if (ret < 0)
    return AVERROR_EXTERNAL;

  // if (!nnal) return 0;
  if (nnal)
  {
    for (i = 0; i < nnal; i++)
      payload_size += nal[i].iSize;

    ret = ff_alloc_packet2(avctx, pkt, payload_size, 0);
    if (ret < 0)
    {
      av_log(avctx, AV_LOG_ERROR, "Error getting output packet.\n");
      return ret;
    }
    dst = pkt->data;

    for (i = 0; i < nnal; i++)
    {
      memcpy(dst, nal[i].pPayload, nal[i].iSize);
      dst += nal[i].iSize;

      if (is_keyframe(nal[i].naltype))
        pkt->flags |= AV_PKT_FLAG_KEY;
    }
    pkt->pts = wz265pic_out.pts;
    pkt->dts = wz265pic_out.dts;

    *got_packet = 1;
  }

  return 0;
}

// TODO: AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV444P,
static const enum AVPixelFormat wz265_csp_eight[] = {AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P,
                                                     AV_PIX_FMT_NONE};

static const enum AVPixelFormat wz265_csp_all[] = {
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUV420P9LE,
    AV_PIX_FMT_YUV420P10LE,
    AV_PIX_FMT_NONE};

static av_cold void libwz265_encode_init_csp(AVCodec *codec) { codec->pix_fmts = wz265_csp_all; }

// clang-format off
#define OFFSET(x) offsetof(libwz265Context, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
  { "aq-mode",       "AQ method",                                       OFFSET(aq_mode),       AV_OPT_TYPE_INT,    {.i64 = -1}, -1, INT_MAX, VE, "aq_mode" },
  { "aq-strength",   "AQ strength. Reduces blocking and blurring in flat and textured areas.", OFFSET(aq_strength), AV_OPT_TYPE_FLOAT, {.dbl = -1}, -1, FLT_MAX, VE },
  { "aq-smooth",     "AQ smooth. ", OFFSET(aq_smooth), AV_OPT_TYPE_FLOAT, {.dbl = -1}, -1, FLT_MAX, VE  },
  { "psy",           "Use psychovisual optimizations.",                 OFFSET(psy),           AV_OPT_TYPE_BOOL,   {.i64 = -1}, -1, 1, VE  },
  { "psy-rd",        "Strength of psychovisual optimization, in <psy-rd>:<psy-trellis> format.", OFFSET(psy_rd), AV_OPT_TYPE_FLOAT,  {.dbl = -1}, -1, FLT_MAX, VE },
  { "crf", "set the wz265 crf", OFFSET(crf), AV_OPT_TYPE_FLOAT, { .dbl = -1 }, -1, FLT_MAX, VE },
  { "preset", "set the wz265 preset", OFFSET(preset), AV_OPT_TYPE_STRING, { .str = "medium" }, 0, 0, VE },
  { "tune", "set the wz265 tune parameter", OFFSET(tune), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE },
  { "vui_time", "write timeinfo to vui", OFFSET(vui_time), AV_OPT_TYPE_INT, { 0 }, INT_MIN, INT_MAX, VE },
  { "duration",       "video duration",                             OFFSET(duration), AV_OPT_TYPE_DOUBLE,  {.dbl  = -1.0 }, -1.0, DBL_MAX, VE },
  { "wz265-params", "set the wz265 configuration using a :-separated list of key=value parameters", OFFSET(wz265_opts), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE },
  { NULL }
};
// clang-format on

static const AVClass class = {
    .class_name = "libwz265",
    .item_name = av_default_item_name,
    .option = options,
    .version = LIBAVUTIL_VERSION_INT,
};

static const AVCodecDefault wz265_defaults[] = {
    {"b", "0"},
    {"bf", "-1"},
    {"g", "-1"},
    {"keyint_min", "-1"},
    {"sc_threshold", "-1"},
    {"qmin", "-1"},
    {"qmax", "-1"},
    {"refs", "-1"},
    {"trellis", "-1"},
    {"me_range", "-1"},
    {"nr", "-1"},

    {NULL},
};

AVCodec ff_libwz265_encoder = {
    .name = "libwz265",
    .long_name = NULL_IF_CONFIG_SMALL("libwz265 H.265 / HEVC"),
    .type = AVMEDIA_TYPE_VIDEO,
    .id = AV_CODEC_ID_HEVC,
    .init = libwz265_encode_init,
    .init_static_data = libwz265_encode_init_csp,
    .encode2 = libwz265_encode_frame,
    .close = libwz265_encode_close,
    .priv_data_size = sizeof(libwz265Context),
    .priv_class = &class,
    .defaults = wz265_defaults,
    .capabilities = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AUTO_THREADS,
};

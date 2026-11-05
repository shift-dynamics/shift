#include "shift/gui/video_encoder.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <stdexcept>
#include <string>

/* FFmpeg's message for an error code */
static std::string ErrorString(int error) {
  char text[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(error, text, sizeof(text));
  return text;
}

static void Check(int error, const char* what) {
  if(error < 0) {
    throw std::runtime_error(std::string(what) + ": " + ErrorString(error));
  }
}

/* the encoder for a container: H.264 where it takes it, else its default,
else MPEG-4 part 2 */
static const AVCodec* ChooseEncoder(const AVOutputFormat* format) {
  const AVCodec* h264 = avcodec_find_encoder_by_name("libx264");
  if(h264 && avformat_query_codec(format, AV_CODEC_ID_H264,
                                  FF_COMPLIANCE_NORMAL) == 1) {
    return h264;
  }
  if(format->video_codec != AV_CODEC_ID_NONE) {
    if(const AVCodec* codec = avcodec_find_encoder(format->video_codec)) {
      return codec;
    }
  }
  return avcodec_find_encoder(AV_CODEC_ID_MPEG4);
}

/* the encoder's pixel format: 4:2:0 YUV if it takes it, else its first */
static AVPixelFormat ChoosePixelFormat(const AVCodecContext* context,
                                       const AVCodec* codec) {
  const void* configs = nullptr;
  int count = 0;
  if(avcodec_get_supported_config(context, codec, AV_CODEC_CONFIG_PIX_FORMAT,
                                  0, &configs, &count) < 0 ||
     !configs || count == 0) {
    return AV_PIX_FMT_YUV420P;
  }
  const auto* formats = static_cast<const AVPixelFormat*>(configs);
  for(int i = 0; i < count; i++) {
    if(formats[i] == AV_PIX_FMT_YUV420P) {
      return formats[i];
    }
  }
  return formats[0];
}

shift::gui::VideoEncoder::~VideoEncoder() {
  Release();
}

void shift::gui::VideoEncoder::Open(const std::string& filename, int width,
                                    int height, int frame_rate) {
  Release();
  if(width <= 0 || height <= 0 || width % 2 || height % 2) {
    throw std::runtime_error("the video size must be even and positive, not " +
                             std::to_string(width) + " x " +
                             std::to_string(height));
  }
  if(frame_rate <= 0) {
    throw std::runtime_error("the frame rate must be positive");
  }
  width_ = width;
  height_ = height;
  next_pts_ = 0;
  av_log_set_level(AV_LOG_WARNING); // not the encoders' statistics
  try {
    Check(avformat_alloc_output_context2(&format_, nullptr, nullptr,
                                         filename.c_str()),
          ("no video format for " + filename).c_str());
    const AVCodec* codec = ChooseEncoder(format_->oformat);
    if(!codec) {
      throw std::runtime_error("no video encoder for " + filename);
    }
    encoder_name_ = codec->name;

    codec_ = avcodec_alloc_context3(codec);
    if(!codec_) {
      throw std::runtime_error("out of memory");
    }
    codec_->width = width;
    codec_->height = height;
    codec_->time_base = AVRational{1, frame_rate};
    codec_->framerate = AVRational{frame_rate, 1};
    codec_->gop_size = frame_rate; // a key frame each second, for seeking
    codec_->pix_fmt = ChoosePixelFormat(codec_, codec);
    codec_->color_range = AVCOL_RANGE_MPEG;
    codec_->colorspace = AVCOL_SPC_BT709;
    codec_->color_primaries = AVCOL_PRI_BT709;
    codec_->color_trc = AVCOL_TRC_BT709;
    if(encoder_name_ == "libx264") {
      av_opt_set(codec_->priv_data, "preset", "medium", 0);
      av_opt_set(codec_->priv_data, "crf", "18", 0); // visually lossless
    } else if(encoder_name_ == "libvpx-vp9") {
      codec_->bit_rate = 0; // constant quality
      av_opt_set(codec_->priv_data, "crf", "30", 0);
    } else {
      /* about 0.2 bits per pixel, 12 Mbit/s at 1080p and 30 frames/s */
      codec_->bit_rate = static_cast<int64_t>(0.2 * width * height * frame_rate);
    }
    if(format_->oformat->flags & AVFMT_GLOBALHEADER) {
      codec_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }
    Check(avcodec_open2(codec_, codec, nullptr),
          ("could not open the " + encoder_name_ + " encoder").c_str());

    stream_ = avformat_new_stream(format_, nullptr);
    if(!stream_) {
      throw std::runtime_error("out of memory");
    }
    stream_->time_base = codec_->time_base;
    stream_->avg_frame_rate = codec_->framerate;
    Check(avcodec_parameters_from_context(stream_->codecpar, codec_),
          "could not set the stream parameters");

    if(!(format_->oformat->flags & AVFMT_NOFILE)) {
      Check(avio_open(&format_->pb, filename.c_str(), AVIO_FLAG_WRITE),
            ("could not create " + filename).c_str());
    }
    Check(avformat_write_header(format_, nullptr),
          "could not write the video header");
    header_written_ = true;

    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if(!frame_ || !packet_) {
      throw std::runtime_error("out of memory");
    }
    frame_->format = codec_->pix_fmt;
    frame_->width = width;
    frame_->height = height;
    Check(av_frame_get_buffer(frame_, 0), "could not allocate a frame");

    scale_ = sws_getContext(width, height, AV_PIX_FMT_RGBA, width, height,
                            codec_->pix_fmt, SWS_BICUBIC, nullptr, nullptr,
                            nullptr);
    if(!scale_) {
      throw std::runtime_error("could not convert RGBA to the encoder's "
                               "pixel format");
    }
    /* RGB to BT.709 YUV, limited range */
    const int* coefficients = sws_getCoefficients(SWS_CS_ITU709);
    sws_setColorspaceDetails(scale_, coefficients, 1, coefficients, 0, 0,
                             1 << 16, 1 << 16);
  } catch(...) {
    Release();
    throw;
  }
}

void shift::gui::VideoEncoder::Write(const uint8_t* rgba, int stride) {
  if(!codec_) {
    throw std::runtime_error("the video is not open");
  }
  /* the encoder may still hold the previous frame's buffer */
  Check(av_frame_make_writable(frame_), "could not write a frame");
  const uint8_t* source[1] = {rgba};
  const int source_stride[1] = {stride};
  sws_scale(scale_, source, source_stride, 0, height_, frame_->data,
            frame_->linesize);
  frame_->pts = next_pts_++;
  Encode(frame_);
}

void shift::gui::VideoEncoder::Encode(AVFrame* frame) {
  Check(avcodec_send_frame(codec_, frame), "could not encode a frame");
  while(true) {
    const int result = avcodec_receive_packet(codec_, packet_);
    if(result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
      return;
    }
    Check(result, "could not encode a frame");
    av_packet_rescale_ts(packet_, codec_->time_base, stream_->time_base);
    packet_->stream_index = stream_->index;
    Check(av_interleaved_write_frame(format_, packet_),
          "could not write the video");
  }
}

void shift::gui::VideoEncoder::Close() {
  if(!codec_) {
    return;
  }
  try {
    Encode(nullptr);
    Check(av_write_trailer(format_), "could not finish the video");
  } catch(...) {
    Release();
    throw;
  }
  header_written_ = false;
  Release();
}

void shift::gui::VideoEncoder::Release() {
  sws_freeContext(scale_);
  scale_ = nullptr;
  av_packet_free(&packet_);
  av_frame_free(&frame_);
  avcodec_free_context(&codec_);
  if(format_) {
    if(header_written_) {
      av_write_trailer(format_); // a file cut short is still playable
      header_written_ = false;
    }
    if(!(format_->oformat->flags & AVFMT_NOFILE)) {
      avio_closep(&format_->pb);
    }
    avformat_free_context(format_);
    format_ = nullptr;
  }
  stream_ = nullptr;
}

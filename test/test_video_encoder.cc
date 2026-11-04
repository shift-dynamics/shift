#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include "shift/gui/video_encoder.h"

using shift::gui::VideoEncoder;

/* the frames of a video decoded to RGBA, top row first */
static std::vector<std::vector<uint8_t>> DecodeVideo(const std::string& filename,
                                                     int& width, int& height) {
  std::vector<std::vector<uint8_t>> frames;
  AVFormatContext* format = nullptr;
  if(avformat_open_input(&format, filename.c_str(), nullptr, nullptr) < 0) {
    throw std::runtime_error("could not open " + filename);
  }
  avformat_find_stream_info(format, nullptr);
  const int index =
    av_find_best_stream(format, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if(index < 0) {
    avformat_close_input(&format);
    throw std::runtime_error("no video stream");
  }
  const AVCodecParameters* parameters = format->streams[index]->codecpar;
  const AVCodec* codec = avcodec_find_decoder(parameters->codec_id);
  AVCodecContext* context = avcodec_alloc_context3(codec);
  avcodec_parameters_to_context(context, parameters);
  avcodec_open2(context, codec, nullptr);
  width = context->width;
  height = context->height;
  AVPacket* packet = av_packet_alloc();
  AVFrame* frame = av_frame_alloc();
  SwsContext* scale = nullptr;
  auto receive = [&]() {
    while(avcodec_receive_frame(context, frame) == 0) {
      scale = sws_getCachedContext(scale, frame->width, frame->height,
                                   static_cast<AVPixelFormat>(frame->format),
                                   frame->width, frame->height, AV_PIX_FMT_RGBA,
                                   SWS_BICUBIC, nullptr, nullptr, nullptr);
      const int* coefficients = sws_getCoefficients(SWS_CS_ITU709);
      sws_setColorspaceDetails(scale, coefficients, 0, coefficients, 1, 0,
                               1 << 16, 1 << 16);
      std::vector<uint8_t> rgba(size_t(frame->width) * frame->height * 4);
      uint8_t* destination[1] = {rgba.data()};
      const int stride[1] = {frame->width * 4};
      sws_scale(scale, frame->data, frame->linesize, 0, frame->height,
                destination, stride);
      frames.push_back(std::move(rgba));
    }
  };
  while(av_read_frame(format, packet) >= 0) {
    if(packet->stream_index == index) {
      avcodec_send_packet(context, packet);
      receive();
    }
    av_packet_unref(packet);
  }
  avcodec_send_packet(context, nullptr);
  receive();
  sws_freeContext(scale);
  av_frame_free(&frame);
  av_packet_free(&packet);
  avcodec_free_context(&context);
  avformat_close_input(&format);
  return frames;
}

/* frames of one colour each: red, green, blue, ... */
static void SolidFrame(int i, int width, int height, std::vector<uint8_t>& rgba) {
  static const uint8_t colours[3][3] = {{200, 30, 30}, {30, 200, 30}, {30, 30, 200}};
  rgba.resize(size_t(width) * height * 4);
  for(size_t p = 0; p < rgba.size(); p += 4) {
    rgba[p + 0] = colours[i % 3][0];
    rgba[p + 1] = colours[i % 3][1];
    rgba[p + 2] = colours[i % 3][2];
    rgba[p + 3] = 255;
  }
}

static void WriteAndCheck(const std::string& extension) {
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    ("shift_test_video_" + std::to_string(::getpid()) + extension);
  const int width = 64, height = 48, count = 12;
  VideoEncoder encoder;
  encoder.Open(path.string(), width, height, 24);
  std::vector<uint8_t> rgba;
  for(int i = 0; i < count; i++) {
    SolidFrame(i, width, height, rgba);
    encoder.Write(rgba.data(), 4 * width);
  }
  encoder.Close();

  int decoded_width = 0, decoded_height = 0;
  const auto frames = DecodeVideo(path.string(), decoded_width, decoded_height);
  std::filesystem::remove(path);
  EXPECT_EQ(decoded_width, width);
  EXPECT_EQ(decoded_height, height);
  ASSERT_EQ(frames.size(), size_t(count)) << encoder.get_encoder_name();
  for(int i = 0; i < count; i++) {
    SolidFrame(i, width, height, rgba);
    const size_t centre = (size_t(height / 2) * width + width / 2) * 4;
    for(int c = 0; c < 3; c++) {
      EXPECT_NEAR(frames[i][centre + c], rgba[centre + c], 12)
        << "frame " << i << " channel " << c << " " << encoder.get_encoder_name();
    }
  }
}

TEST(VideoEncoder, Mp4) {
  WriteAndCheck(".mp4");
}

TEST(VideoEncoder, Matroska) {
  WriteAndCheck(".mkv");
}

TEST(VideoEncoder, OddSizeThrows) {
  VideoEncoder encoder;
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() / "shift_test_video_odd.mp4";
  EXPECT_THROW(encoder.Open(path.string(), 63, 48, 30), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path));
}

#ifndef SHIFT_GUI_VIDEO_ENCODER_H_
#define SHIFT_GUI_VIDEO_ENCODER_H_

#include <cstdint>
#include <string>

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct AVStream;
struct SwsContext;

namespace shift {
namespace gui {

/*!
@brief Writes RGBA images to a video file with the FFmpeg libraries
(libavformat, libavcodec, libswscale), in process

The container follows the file name's extension (.mp4, .mkv, .mov, .webm,
.avi, ...). The encoder is H.264 (libx264) where the container takes it,
otherwise the container's default video encoder, or MPEG-4 part 2, which
FFmpeg always has. Frames are converted to the encoder's pixel format,
4:2:0 YUV where it has it, which needs an even width and height.

    VideoEncoder encoder;
    encoder.Open("run.mp4", 1920, 1080, 30);
    for(...) encoder.Write(rgba, 4 * 1920);
    encoder.Close();

Errors throw std::runtime_error, with FFmpeg's message.
*/
class VideoEncoder {
public:
  VideoEncoder() = default;
  ~VideoEncoder();
  VideoEncoder(const VideoEncoder&) = delete;
  VideoEncoder& operator=(const VideoEncoder&) = delete;

  /*! \brief create the file and set up the encoder, frame_rate frames per
   * second */
  void Open(const std::string& filename, int width, int height,
            int frame_rate);

  /*! \brief encode the next frame, width x height RGBA pixels (8 bits per
   * channel), top row first, stride bytes between rows */
  void Write(const uint8_t* rgba, int stride);

  /*! \brief encode the frames still buffered and finish the file */
  void Close();

  /*! \brief name of the encoder in use, e.g. libx264 */
  const std::string& get_encoder_name() const {
    return encoder_name_;
  }

private:
  /*! \brief send a frame to the encoder, nullptr to flush it, and write the
   * packets it returns */
  void Encode(AVFrame* frame);
  void Release();

  AVFormatContext* format_ = nullptr;
  AVCodecContext* codec_ = nullptr;
  AVStream* stream_ = nullptr;
  AVFrame* frame_ = nullptr;
  AVPacket* packet_ = nullptr;
  SwsContext* scale_ = nullptr;
  int width_ = 0;
  int height_ = 0;
  int64_t next_pts_ = 0;
  bool header_written_ = false;
  std::string encoder_name_;
};

} // namespace gui
} // namespace shift

#endif

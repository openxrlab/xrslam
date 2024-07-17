#include "libsensors.h"
#include <cstdint>
#include <memory>
#include <vector>

#if defined(__APPLE__) && __APPLE__
#include "TargetConditionals.h"
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
#define LIBSENSORS_IOS
#endif
#endif

#if defined(ANDROID) || defined(__ANDROID__)
#define LIBSENSORS_ANDROID
#endif

#if !defined(LIBSENSORS_IOS) && !defined(LIBSENSORS_ANDROID)
#define LIBSENSORS_ENABLE_FFMPEG
#endif

#if defined(LIBSENSORS_ENABLE_FFMPEG)
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
}
#endif

namespace libsensors {

class Sensors::SensorsImpl {
    Sensors* sensors;

  public:
    SensorsImpl(Sensors* sensors) :
        sensors(sensors) {
    }

    ~SensorsImpl() {
#if defined(LIBSENSORS_ENABLE_FFMPEG)
        if (context) {
            deinit_ffmpeg();
        }
#endif
    }

    void parse_data(const void* bytes, size_t size) {
        buffer.insert(buffer.end(), (const unsigned char*)bytes, ((const unsigned char*)bytes) + size);

        while (true) {
            size_t consumed = 0;
            std::uint8_t type;
            double timestamp;

            if (!advance(type, consumed)) goto end_parse;
            if (!advance(timestamp, consumed)) goto end_parse;

            switch (type) {
            case 0x00: // image
            {
                std::uint32_t width, height;
                if (!advance(width, consumed)) goto end_parse;
                if (!advance(height, consumed)) goto end_parse;
                // if (!try_advance_size(width * height, consumed)) goto end_parse;
                // std::vector<std::uint8_t> pixels;
                // advance_size(width * height, consumed, pixels);
                // sensors->on_image(timestamp, width, height, pixels.data());

                if (!try_advance_size(width * height * sizeof(std::uint32_t), consumed)) goto end_parse;
                std::vector<std::uint32_t> pixels;
                advance_size(width * height * sizeof(std::uint32_t), consumed, pixels);
                sensors->on_image(timestamp, width, height, reinterpret_cast<std::uint8_t*>(pixels.data()));

            } break;
            case 0x01: // gyroscope
            {
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_gyroscope(timestamp, x, y, z);
            } break;
            case 0x02: // accelerometer
            {
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_accelerometer(timestamp, x, y, z);
            } break;
            case 0x03: // magnetometer
            {
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_magnetometer(timestamp, x, y, z);
            } break;
            case 0x04: // altimeter
            {
                double pressure, elevation;
                if (!advance(pressure, consumed)) goto end_parse;
                if (!advance(elevation, consumed)) goto end_parse;
                sensors->on_altimeter(timestamp, pressure, elevation);
            } break;
            case 0x05: // gps
            {
                double lon, lat, alt, hacc, vacc;
                if (!advance(lon, consumed)) goto end_parse;
                if (!advance(lat, consumed)) goto end_parse;
                if (!advance(alt, consumed)) goto end_parse;
                if (!advance(hacc, consumed)) goto end_parse;
                if (!advance(vacc, consumed)) goto end_parse;
                sensors->on_gps(timestamp, lon, lat, alt, hacc, vacc);
            } break;
            case 0x08: // h264
            {
                size_t header_consumed = consumed;

                std::uint32_t sps_len, pps_len;
                if (!advance(sps_len, consumed)) goto end_parse;
                if (sps_len > 0) {
                    if (!try_advance_size(sps_len, consumed)) goto end_parse;
                    consumed += sps_len;
                }
                if (!advance(pps_len, consumed)) goto end_parse;
                if (pps_len > 0) {
                    if (!try_advance_size(pps_len, consumed)) goto end_parse;
                    consumed += pps_len;
                }
                while (true) {
                    std::uint32_t nal_len;
                    if (!advance(nal_len, consumed)) goto end_parse;
                    if (nal_len == 0) break;
                    if (!try_advance_size(nal_len, consumed)) goto end_parse;
                    consumed += nal_len;
                }

                // begin true decoding
                static const std::uint8_t annexb[] = {0x00, 0x00, 0x01};
                consumed = header_consumed;
                std::vector<std::vector<std::uint8_t>> payloads;
                advance(sps_len, consumed);
                if (sps_len > 0) {
                    std::vector<std::uint8_t> sps_buf;
                    advance_size(sps_len, consumed, sps_buf);
                    sps_buf.insert(sps_buf.begin(), annexb, annexb + 3);
                    payloads.emplace_back(std::move(sps_buf));
                }
                advance(pps_len, consumed);
                if (pps_len > 0) {
                    std::vector<std::uint8_t> pps_buf;
                    advance_size(pps_len, consumed, pps_buf);
                    pps_buf.insert(pps_buf.begin(), annexb, annexb + 3);
                    payloads.emplace_back(std::move(pps_buf));
                }
                while (true) {
                    std::uint32_t nal_len;
                    advance(nal_len, consumed);
                    if (nal_len == 0) break;
                    std::vector<std::uint8_t> nal_buf;
                    advance_size(nal_len, consumed, nal_buf);
                    nal_buf.insert(nal_buf.begin(), annexb, annexb + 3);
                    payloads.emplace_back(std::move(nal_buf));
                }

                for (const std::vector<std::uint8_t>& payload : payloads) {
                    h264_decode_payload(timestamp, payload.data(), payload.size());
                }
            } break;
            case 0x12: { // gravity reported by device
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_gravity(timestamp, x, y, z);
            } break;
            default: {
                sensors->on_error("unknown data type.");
            } break;
            }
            if (consumed > 0) {
                buffer.erase(buffer.begin(), buffer.begin() + consumed);
            }
        }

    end_parse:
        return;
    }

  private:
    bool try_advance_size(size_t size, size_t consumed) const {
        if (buffer.size() >= consumed + size) {
            return true;
        } else {
            return false;
        }
    }

    template <typename T>
    bool advance(T& value, size_t& consumed) const {
        if (buffer.size() >= consumed + sizeof(value)) {
            value = *(const T*)(buffer.data() + consumed);
            consumed += sizeof(value);
            return true;
        } else {
            return false;
        }
    }

    bool advance_size(size_t size, size_t& consumed, std::vector<std::uint8_t>& buf) const {
        if (buffer.size() >= consumed + size) {
            buf.resize(size);
            memcpy(buf.data(), buffer.data() + consumed, size);
            consumed += size;
            return true;
        } else {
            sensors->on_error("fatal error: buffer overrun.");
            exit(EXIT_FAILURE);
            return false;
        }
    }

    // BGRA Image
    bool advance_size(std::size_t size, size_t& consumed, std::vector<std::uint32_t>& buf) const {
        if (buffer.size() >= consumed + size) {
            const std::uint32_t* data_ptr = reinterpret_cast<const std::uint32_t*>(buffer.data() + consumed);
            buf.assign(data_ptr, data_ptr + size / sizeof(std::uint32_t));
            consumed += size;
            return true;
        } else {
            sensors->on_error("fatal error: buffer overrun.");
            exit(EXIT_FAILURE);
            return false;
        }
    }

#if defined(LIBSENSORS_ENABLE_FFMPEG)
    void init_ffmpeg() {
        ::av_log_set_level(AV_LOG_QUIET);
        codec = const_cast<AVCodec *>(::avcodec_find_decoder(AV_CODEC_ID_H264));
        if (!codec) {
            sensors->on_error("cannot find h264 decoder.");
            goto on_error;
        }

        context = ::avcodec_alloc_context3(codec);
        if (!context) {
            sensors->on_error("cannot create h264 decoder context.");
            goto on_error;
        }

        if (codec->capabilities & AV_CODEC_ID_TRUEHD) {
            context->flags |= AV_CODEC_ID_TRUEHD;
        }

        if (::avcodec_open2(context, codec, NULL) < 0) {
            sensors->on_error("cannot open h264 decoder.");
            goto on_error;
        }

        parser = ::av_parser_init(AV_CODEC_ID_H264);
        if (!parser) {
            sensors->on_error("cannot init h264 parser.");
            goto on_error;
        }

        picture = ::av_frame_alloc();
        if (!picture) {
            sensors->on_error("cannot allocate frame buffer.");
            goto on_error;
        }

        packet = ::av_packet_alloc();
        if (!packet) {
            sensors->on_error("cannot allocate packet buffer.");
            goto on_error;
        }

        return;
    on_error:
        deinit_ffmpeg();
    }

    void deinit_ffmpeg() {
        if (packet) {
            ::av_packet_free(&packet);
            packet = nullptr;
        }
        if (picture) {
            ::av_free(picture);
            picture = nullptr;
        }
        if (parser) {
            ::av_parser_close(parser);
            parser = nullptr;
        }
        if (context) {
            ::avcodec_close(context);
            ::av_free(context);
            context = nullptr;
        }
    }

    void h264_decode_payload(double timestamp, const void* payload, size_t payload_size) {
        if (!context) {
            init_ffmpeg();
            if (!context) {
                sensors->on_error("cannot init libsensors.");
            }
        }
        std::vector<uint8_t> payload_buf((uint8_t*)payload, (uint8_t*)payload + payload_size);
        payload_buf.resize(payload_buf.size() + AV_INPUT_BUFFER_PADDING_SIZE);
        int pos = 0;
        while (payload_size > 0) {
            current_frame_time = timestamp;
            int len = ::av_parser_parse2(parser, context, &packet->data, &packet->size, &payload_buf[pos], (int)payload_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
            if (len < 0) {
                sensors->on_error("internal error: av_parser_parse2");
                return;
            }
            pos += len;
            payload_size -= len;
            if (packet->size > 0) {
                h264_decode_frame();
            }
        }
    }

    void h264_decode_frame() {
        int ret = ::avcodec_send_packet(context, packet);
        if (ret < 0) {
            sensors->on_error("internal error: avcodec_send_packet");
            return;
        }
        while (ret >= 0) {
            ret = ::avcodec_receive_frame(context, picture);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return;
            } else if (ret < 0) {
                sensors->on_error("internal error: avcodec_receive_frame");
            }
            h264_output_yuv(current_frame_time, picture->width, picture->height, (char*)picture->data[0], (char*)picture->data[1], (char*)picture->data[2], picture->linesize[0], picture->linesize[1], picture->linesize[2]);
        }
    }

    void h264_output_yuv(double timestamp, int width, int height, const char* Y, const char* U, const char* V, int stride_y, int stride_u, int stride_v) {
        std::vector<unsigned char> bytes(width * height);
        for (int i = 0; i < height; ++i) {
            memcpy(bytes.data() + i * width, Y + i * stride_y, width);
        }
        sensors->on_image(timestamp, width, height, bytes.data());
    }

    double current_frame_time;
    AVCodec* codec = nullptr;
    AVCodecContext* context = nullptr;
    AVCodecParserContext* parser = nullptr;
    AVFrame* picture = nullptr;
    AVPacket* packet = nullptr;
#else
    void h264_decode_payload(double timestamp, const void* payload, size_t payload_size) {
        sensors->on_error("H264 decoding is not supported on iOS.");
    }
#endif

    std::vector<unsigned char> buffer;
};

Sensors::Sensors() {
    pimpl = std::make_unique<SensorsImpl>(this);
}

Sensors::~Sensors() = default;

void Sensors::parse_data(const void* bytes, size_t size) {
    pimpl->parse_data(bytes, size);
}

} // namespace libsensors

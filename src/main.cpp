#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "camera/camera.h"
#include "camera/photography_settings.h"
#include "camera/device_discovery.h"
#include "camera/ins_types.h"

#include "stream/stream_delegate.h"
#include "stream/stream_types.h"

#include <atomic>
#include <signal.h>
#include <thread>
#include <regex>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

std::atomic<bool> shutdown_requested(false);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        shutdown_requested = true;
    }
}

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    AVCodec* codec;
    AVCodecContext* codecCtx;
    AVFrame* avFrame;
    AVPacket* pkt;
    struct SwsContext* img_convert_ctx;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

public:
    TestStreamDelegate(rclcpp::Node::SharedPtr node) {
        image_pub = node->create_publisher<sensor_msgs::msg::Image>("insta_image_yuv", 60);

        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec) {
            RCLCPP_ERROR(node->get_logger(), "Codec not found");
            exit(1);
        }

        codecCtx = avcodec_alloc_context3(codec);
        codecCtx->flags2 |= AV_CODEC_FLAG2_FAST;
        if (!codecCtx) {
            RCLCPP_ERROR(node->get_logger(), "Could not allocate video codec context");
            exit(1);
        }

        if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
            RCLCPP_ERROR(node->get_logger(), "Could not open codec");
            exit(1);
        }

        avFrame = av_frame_alloc();
        pkt = av_packet_alloc();
    }

    ~TestStreamDelegate() {
        av_frame_free(&avFrame);
        av_packet_free(&pkt);
        avcodec_free_context(&codecCtx);
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        RCLCPP_INFO(rclcpp::get_logger("TestStreamDelegate"), "on audio data");
    }

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        if (stream_index == 0) {
            pkt->data = const_cast<uint8_t*>(data);
            pkt->size = size;

            if (avcodec_send_packet(codecCtx, pkt) == 0) {
                while (avcodec_receive_frame(codecCtx, avFrame) == 0) {
                    int width = avFrame->width;
                    int height = avFrame->height;
                    int chromaHeight = height / 2;
                    int chromaWidth = width / 2;

                    cv::Mat yuv(height + chromaHeight, width, CV_8UC1);

                    memcpy(yuv.data, avFrame->data[0], width * height);
                    memcpy(yuv.data + width * height, avFrame->data[1], chromaWidth * chromaHeight);
                    memcpy(yuv.data + width * height + chromaWidth * chromaHeight, avFrame->data[2], chromaWidth * chromaHeight);

                    sensor_msgs::msg::Image msg;
                    msg.header.stamp = rclcpp::Clock().now();
                    msg.header.frame_id = "camera_frame";
                    msg.height = yuv.rows;
                    msg.width = yuv.cols;
                    msg.encoding = "8UC1";
                    msg.is_bigendian = false;
                    msg.step = yuv.cols * yuv.elemSize();
                    msg.data.assign(yuv.datastart, yuv.dataend);

                    image_pub->publish(msg);
                }
            }
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        // Implement Gyro data handling if needed
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {
        // Implement Exposure data handling if needed
    }
};

class CameraWrapper : public rclcpp::Node {
public:
    CameraWrapper() : Node("insta360_camera_node") {
        RCLCPP_ERROR(this->get_logger(), "Opened Camera");
    }

    ~CameraWrapper() {
        RCLCPP_ERROR(this->get_logger(), "Closing Camera");
        this->cam->Close();
    }

    int run_camera() {
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        for (int i = 0; i < list.size(); ++i) {
            auto desc = list[i];
            RCLCPP_INFO(this->get_logger(), "serial: %s camera type: %d lens type: %d",
                        desc.serial_number.c_str(), int(desc.camera_type), int(desc.lens_type));
        }

        if (list.empty()) {
            RCLCPP_ERROR(this->get_logger(), "no device found.");
            return -1;
        }

        this->cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!this->cam->Open()) {
            RCLCPP_ERROR(this->get_logger(), "failed to open camera");
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "http base url: %s", this->cam->GetHttpBaseUrl().c_str());

        std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>(shared_from_this());
        this->cam->SetStreamDelegate(delegate);

        discovery.FreeDeviceDescriptors(list);

        RCLCPP_INFO(this->get_logger(), "Successfully opened camera...");

        auto start = time(NULL);
        this->cam->SyncLocalTimeToCamera(start);

        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_1152_1152P30;
        param.video_bitrate = 1024 * 1024 / 100;
        param.enable_audio = false;
        param.using_lrv = false;

        do {

        } while (!this->cam->StartLiveStreaming(param));
        RCLCPP_INFO(this->get_logger(), "successfully started live stream");

        return 0;
    }

private:
    std::shared_ptr<ins_camera::Camera> cam;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    signal(SIGINT, signal_handler);

    {
        auto camera_node = std::make_shared<CameraWrapper>();
        if (camera_node->run_camera() == 0) {
            rclcpp::spin(camera_node);
        }
    }

    rclcpp::shutdown();
    return 0;
}

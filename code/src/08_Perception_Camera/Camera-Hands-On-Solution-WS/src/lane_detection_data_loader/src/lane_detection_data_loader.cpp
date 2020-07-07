/*  
Copyright 2020 Gjorgji Nikolovski

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
*/

#include <chrono>
#include <memory>
#include "lane_detection_data_loader/tinylib.h"
#include "lane_msgs/msg/lane_marking.hpp"
#include "lane_msgs/msg/lane_marking_array.hpp"
#include "lane_msgs/msg/lane_marking_array_both.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#define H264_INBUF_SIZE 16384 /* number of bytes we read per chunk */

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <fstream>

extern "C"
{
#include "libswscale/swscale.h"
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
}



std::ifstream infile(ament_index_cpp::get_package_share_directory("lane_detection_data_loader")+"/resources/laneMarks.csv");

class lane_detection_data_loader : public rclcpp::Node
{

public:
    lane_detection_data_loader()
        : Node("lane_detection_data_loader"), codec(NULL), codec_context(NULL), parser(NULL), fp(NULL), frame_timeout(0), frame_delay(0)
    {

        pub = create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
        pub_markings = create_publisher<lane_msgs::msg::LaneMarkingArrayBoth>("/lane_markings_left_right", 10);
        avcodec_register_all();
    }

    ~lane_detection_data_loader()
    {
        if (parser)
        {
            av_parser_close(parser);
            parser = NULL;
        }

        if (codec_context)
        {
            avcodec_close(codec_context);
            av_free(codec_context);
            codec_context = NULL;
        }

        if (picture)
        {
            av_free(picture);
            picture = NULL;
        }

        if (fp)
        {
            fclose(fp);
            fp = NULL;
        }

        frame = 0;
        frame_timeout = 0;
    }

    bool load(std::string filepath, float fps)
    {

        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec)
        {
            printf("Error: cannot find the h264 codec: %s\n", filepath.c_str());
            return false;
        }

        codec_context = avcodec_alloc_context3(codec);

        if (codec->capabilities & AV_CODEC_CAP_TRUNCATED)
        {
            codec_context->flags |= AV_CODEC_FLAG_TRUNCATED;
        }

        if (avcodec_open2(codec_context, codec, NULL) < 0)
        {
            printf("Error: could not open codec.\n");
            return false;
        }

        fp = fopen(filepath.c_str(), "rb");

        if (!fp)
        {
            printf("Error: cannot open: %s\n", filepath.c_str());
            return false;
        }

        picture = av_frame_alloc();
        parser = av_parser_init(AV_CODEC_ID_H264);

        if (!parser)
        {
            printf("Erorr: cannot create H264 parser.\n");
            return false;
        }

        if (fps > 0.0001f)
        {
            frame_delay = (1.0f / fps) * 1000ull * 1000ull * 1000ull;
            frame_timeout = rx_hrtime() + frame_delay;
        }

        m_convert_ctx = sws_getContext(m_width, m_height, AV_PIX_FMT_YUV420P, m_width, m_height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);

        //load lane markings
        leftLane.reserve(30000);
        rightLane.reserve(30000);
        int frame_nr;
        double coorx, coory;
        char d, dd;
        char lane;
        lane_msgs::msg::LaneMarking lm;
        while ((infile >> frame_nr >> d >> coorx >> dd >> coory >> d >> lane) && (d == ';') && (dd == '&'))
        {
            if (lane == 'l') {
                lm.u = coorx;
                lm.v = coory;
                leftLane[frame_nr].markings.push_back(lm);
            }
            else if (lane == 'r')
            {
                lm.u = coorx;
                lm.v = coory;
                rightLane[frame_nr].markings.push_back(lm);
            }
            
        }


        RCLCPP_INFO(this->get_logger(), "Data succesfully loaded! Publishing ...");
        return true;
    }

    bool readFrame()
    {

        uint64_t now = rx_hrtime();
        if (now < frame_timeout)
        {
            return false;
        }

        bool needs_more = false;

        while (!update(needs_more))
        {
            if (needs_more)
            {
                readBuffer();
                read_attempts++;
            }
            if (read_attempts > 10)
                rclcpp::shutdown();
        }
        read_attempts = 0;
        // it may take some 'reads' before we can set the fps
        if (frame_timeout == 0 && frame_delay == 0)
        {
            double fps = av_q2d(codec_context->time_base);
            if (fps > 0.0)
            {
                frame_delay = fps * 1000ull * 1000ull * 1000ull;
            }
        }

        if (frame_delay > 0)
        {
            frame_timeout = rx_hrtime() + frame_delay;
        }

        return true;
    }

    void decodeFrame(uint8_t *data, int size)
    {

        int got_picture = 0;
        int len = 0;

        AVPacket *pkt = av_packet_alloc();

        pkt->data = data;
        pkt->size = size;

        len = avcodec_decode_video2(codec_context, picture, &got_picture, pkt);
        if (len < 0)
        {
            printf("Error while decoding a frame.\n");
        }

        if (got_picture == 0)
        {
            return;
        }
        auto img = std::make_unique<sensor_msgs::msg::Image>();
        m_stride = 3 * m_width;

        img->data.resize(m_height * m_width * 3);

        uint8_t *buf_out[8] = {&(img->data[0]), NULL, NULL, NULL, NULL, NULL, NULL, NULL};
        sws_scale(m_convert_ctx, (const uint8_t *const *)picture->data, picture->linesize, 0, m_height, buf_out, &m_stride);
        img->width = m_width;
        img->height = m_height;
        img->step = m_stride;
        img->encoding = sensor_msgs::image_encodings::BGR8;
        img->header.stamp = this->now();
        img->header.frame_id = "video";
        pub->publish(std::move(img));

        combined_lanes.markings_left = leftLane[(int)frame].markings;
        combined_lanes.markings_right = rightLane[(int)frame].markings;
        pub_markings->publish(combined_lanes);
        std::cout << frame << std::endl;
        frame+= 1.2;
    }

    int readBuffer()
    {

        int bytes_read = (int)fread(inbuf, 1, H264_INBUF_SIZE, fp);

        if (bytes_read)
        {
            std::copy(inbuf, inbuf + bytes_read, std::back_inserter(buffer));
        }

        return bytes_read;
    }

    bool update(bool &needsMoreBytes)
    {

        needsMoreBytes = false;

        if (!fp)
        {
            printf("Cannot update .. file not opened...\n");
            return false;
        }

        if (buffer.size() == 0)
        {
            needsMoreBytes = true;
            return false;
        }

        uint8_t *data = NULL;
        int size = 0;
        int len = av_parser_parse2(parser, codec_context, &data, &size,
                                   &buffer[0], buffer.size(), 0, 0, AV_NOPTS_VALUE);

        if (size == 0 && len >= 0)
        {
            needsMoreBytes = true;
            return false;
        }

        if (len)
        {
            decodeFrame(&buffer[0], size);
            buffer.erase(buffer.begin(), buffer.begin() + len);
            return true;
        }

        return false;
    }

    uint64_t rx_hrtime()
    {

        static clock_t fast_clock_id = -1;
        struct timespec t;
        clock_t clock_id;

        if (fast_clock_id == -1)
        {
            if (clock_getres(CLOCK_MONOTONIC_COARSE, &t) == 0 && t.tv_nsec <= 1 * 1000 * 1000LLU)
            {
                fast_clock_id = CLOCK_MONOTONIC_COARSE;
            }
            else
            {
                fast_clock_id = CLOCK_MONOTONIC;
            }
        }

        clock_id = CLOCK_MONOTONIC;
        if (clock_gettime(clock_id, &t))
        {
            return 0;
        }

        return t.tv_sec * (uint64_t)1e9 + t.tv_nsec;
    }

    AVCodec *codec;                                                /* the AVCodec* which represents the H264 decoder */
    AVCodecContext *codec_context;                                 /* the context; keeps generic state */
    AVCodecParserContext *parser;                                  /* parser that is used to decode the h264 bitstream */
    AVFrame *picture;                                              /* will contain a decoded picture */
    uint8_t inbuf[H264_INBUF_SIZE + AV_INPUT_BUFFER_PADDING_SIZE]; /* used to read chunks from the file */
    FILE *fp;                                                      /* file pointer to the file from which we read the h264 data */
    float frame = 25;                                                     /* the number of decoded frames */
    uint64_t frame_timeout;                                        /* timeout when we need to parse a new frame */
    uint64_t frame_delay;                                          /* delay between frames (in ns) */
    std::vector<uint8_t> buffer;
    std::vector<lane_msgs::msg::LaneMarkingArray> leftLane;
    std::vector<lane_msgs::msg::LaneMarkingArray> rightLane;
    lane_msgs::msg::LaneMarkingArrayBoth combined_lanes;
    struct SwsContext *m_convert_ctx;
    int read_attempts = 0;
    int m_width = 1920;
    int m_height = 1080;
    int m_stride;
    

private:
    // Image publishing stuff
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub;
    std::shared_ptr<rclcpp::Publisher<lane_msgs::msg::LaneMarkingArrayBoth>> pub_markings;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lane_detection_data_loader>();
    rclcpp::Rate loop_rate(1/100.0);
    node->load(ament_index_cpp::get_package_share_directory("lane_detection_data_loader")+"/resources/laneVideo.h264", 25.0);
    node->readBuffer();
    while (rclcpp::ok())
    {
        node->readFrame();
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}

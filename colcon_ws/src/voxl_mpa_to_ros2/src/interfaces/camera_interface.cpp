/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <modal_pipe.h>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <fstream>

#include "voxl_mpa_to_ros2/utils/camera_helpers.h"
#include "voxl_mpa_to_ros2/interfaces/camera_interface.h"

static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta,
                            char* frame,
                            void* context);

CameraInterface::CameraInterface(
    rclcpp::Node::SharedPtr nh,
    const char *    name) :
    GenericInterface(nh, name)
{
    // Generic interface name
    ginterface_name = name;
    
    if (!strncmp(name, "tracking_down_misp_grey", strlen("tracking_down_misp_grey"))) {
      _frame_id = "tracking_down";
    }

    else if (!strncmp(name, "tracking_down_misp_norm", strlen("tracking_down_misp_norm"))) {
      _frame_id = "tracking_down";
    }

    else if (!strncmp(name, "tracking_front_misp_grey", strlen("tracking_front_misp_grey"))) {
      _frame_id = "tracking_front";
    }

    else if (!strncmp(name, "tracking_front_misp_norm", strlen("tracking_front_misp_norm"))) {
      _frame_id = "tracking_front";
    }

    else if (!strncmp(name, "tracking_rear_misp_grey", strlen("tracking_rear_misp_grey"))) {
      _frame_id = "tracking_rear";
    }

    else if (!strncmp(name, "tracking_rear_misp_norm", strlen("tracking_rear_misp_norm"))) {
      _frame_id = "tracking_rear";
    }

    else {
      _frame_id = name;
    }

    m_imageMsg.header.frame_id = _frame_id;
    m_imageMsg.is_bigendian    = false;
    
    ginterface_name = name;

    pipe_client_set_camera_helper_cb(m_channel, _frame_cb, this);

    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_CAMERA_HELPER | CLIENT_FLAG_START_PAUSED, 0)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

    try {
        // Construct the file path using the provided name
        std::string file_path = std::string("/run/mpa/") + name + "/info";

        // Read the JSON file
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open the file: " + file_path);
        }

        // Parse the JSON file
        nlohmann::json json_data;
        file >> json_data;

        frame_format = json_data["int_format"];
    } catch (const std::exception &e) {
        frame_format = 0;
    }

    this->setPublishingState(false);
}

void CameraInterface::AdvertiseTopics(){


    image_transport::ImageTransport it(m_rosNodeHandle);

    if (frame_format == IMAGE_FORMAT_H265 || frame_format == IMAGE_FORMAT_H264) {
      m_rosCompressedPublisher_ = m_rosNodeHandle->create_publisher<sensor_msgs::msg::CompressedImage>(m_pipeName, 1);
    }
    else {
      m_rosImagePublisher = it.advertise(m_pipeName, 1);
    }
    std::string pipeName = std::string(m_pipeName);
    std::string cameraInfoTopic = pipeName + "/camera_info";

    if(m_rosCameraInfoPublisher == nullptr){
        m_rosCameraInfoPublisher = m_rosNodeHandle->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoTopic, 1);
    }

    // Parsing yaml
    std::string cv_intrinsics_path = "/data/modalai/opencv_" + pipeName + "_intrinsics.yml";
    if(access(cv_intrinsics_path.c_str(), F_OK) == 0){
        YAML::Node config = YAML::LoadFile(cv_intrinsics_path);

        // Transform Frame id
        m_cameraInfo.header.frame_id = _frame_id;
        
        // Getting static values from yaml
        m_cameraInfo.width = config["width"].as<uint32_t>();
        m_cameraInfo.height = config["height"].as<uint32_t>();
        m_cameraInfo.distortion_model = config["distortion_model"].as<std::string>();

        // Getting rotation info
        m_cameraInfo.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

        // Getting distortion data
        auto distortion_data = config["D"]["data"].as<std::vector<double>>();
        m_cameraInfo.d.assign(distortion_data.begin(), distortion_data.end());

        // Load intrinsic camera matrix
        auto camera_matrix_data = config["M"]["data"].as<std::vector<double>>();
        for (size_t i = 0; i < camera_matrix_data.size(); ++i) {
            m_cameraInfo.k[i] = camera_matrix_data[i];
        }

        // Assuming zero for projection matrix P (3x4 zero matrix)
        m_cameraInfo.p = {camera_matrix_data[0], camera_matrix_data[1], camera_matrix_data[2], 0.0,
                    camera_matrix_data[3], camera_matrix_data[4], camera_matrix_data[5], 0.0,
                    camera_matrix_data[6], camera_matrix_data[7], camera_matrix_data[8], 0.0};

    }
    this->setPublishingState(true);

    m_state = ST_AD;
}

void CameraInterface::StopAdvertising(){
    if (frame_format == IMAGE_FORMAT_H265 || frame_format == IMAGE_FORMAT_H264) {
        m_rosCompressedPublisher_.reset();
    }
    else {
        m_rosImagePublisher.shutdown();

    }
    this->setPublishingState(false);
    m_state = ST_CLEAN;
}

int CameraInterface::GetNumClients(){
    if (frame_format == IMAGE_FORMAT_H265 || frame_format == IMAGE_FORMAT_H264) {
        return m_rosCompressedPublisher_->get_subscription_count();
    }
    else {
        return m_rosImagePublisher.getNumSubscribers() + m_rosCameraInfoPublisher->get_subscription_count();
    }
}

// helper callback whenever a frame arrives
static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta,
                            char* frame,
                            void* context)
{

    CameraInterface *interface = (CameraInterface *) context;

    if(interface->GetState() != ST_RUNNING) return;

    image_transport::Publisher& publisher = interface->GetPublisher();
    sensor_msgs::msg::Image& img = interface->GetImageMsg();
    sensor_msgs::msg::CameraInfo& camera_info = interface->GetCameraInfo();
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher = interface->GetCameraInfoPublisher();

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& compressed_publisher = interface->GetCompressedPublisher();
    sensor_msgs::msg::CompressedImage& compressedImage = interface->GetCompressedImageMsg();

    img.header.stamp = _clock_monotonic_to_ros_time(interface->getNodeHandle(), meta.timestamp_ns);
    img.width    = meta.width;
    img.height   = meta.height;

    camera_info.header.stamp = img.header.stamp;
    // camera_info.header.frame_id = std::to_string(meta.frame_id);

    if(interface->getPublishingState()){
        camera_info_publisher->publish(camera_info);
    }


    if(meta.format == IMAGE_FORMAT_NV21 || meta.format == IMAGE_FORMAT_NV12){

        img.header.frame_id = interface->ginterface_name;
        img.is_bigendian = false;
        img.step = meta.width * GetStepSize(IMAGE_FORMAT_YUV422);
        img.encoding = GetRosFormat(IMAGE_FORMAT_YUV422);

        int dataSize = img.step * img.height; 
        img.data.resize(dataSize);

        char *uv = &(frame[dataSize/2]);

        if(meta.format == IMAGE_FORMAT_NV12){

            for(int i = 0; i < meta.height; i+=2)
            {
                for(int j = 0; j < meta.width*2;j+=2){
			        // Use size_t for unsigned indices
            		size_t img_index_0 = (i * meta.width * 2) + (j * 2) + 0;
            		size_t img_index_1 = (i * meta.width * 2) + (j * 2) + 1;
            		size_t img_index_2 = (i * meta.width * 2) + (j * 2) + 2;
            		size_t img_index_3 = (i * meta.width * 2) + (j * 2) + 3;
            		size_t uv_index_0 = ((i/2) * meta.width) + j; // don't subsample chroma here
            		size_t uv_index_1 = uv_index_0 + 1;

            		// Calculate size of img.data and uv
            		size_t uv_size = meta.width * meta.height; //Combined UV size, UVsize = Width/2 * Height * 2 (2-bytes interleaved UV) in YUV422 NV12
            		size_t img_data_size = img.data.size() + uv_size;

            		// Check if indices are within bounds
            		if (img_index_0 < img_data_size && img_index_1 < img_data_size &&
                		img_index_2 < img_data_size && img_index_3 < img_data_size &&
                		uv_index_0 < uv_size && uv_index_1 < uv_size) {

                		// Safely assign values
                		img.data[img_index_0] = uv[uv_index_0]; 
                		img.data[img_index_1] = frame[(i * meta.width) + j]; 
                		img.data[img_index_2] = uv[uv_index_1]; 
                		img.data[img_index_3] = frame[(i * meta.width) + j + 1]; 
                		img.data[((i+1) * meta.width * 2) + (j * 2) + 0] = uv[uv_index_0]; 
                		img.data[((i+1) * meta.width * 2) + (j * 2) + 1] = frame[((i+1) * meta.width) + j]; 
                		img.data[((i+1) * meta.width * 2) + (j * 2) + 2] = uv[uv_index_1]; 
                		img.data[((i+1) * meta.width * 2) + (j * 2) + 3] = frame[((i+1) * meta.width) + j + 1]; 
            		} else {
                		// Log or handle the out-of-bounds access
                		std::cerr << "Index out of bounds: img_index=" << img_index_0
                          		<< ", uv_index_0=" << uv_index_0 << ", uv_index_1:" << uv_index_1 << ", uv_size:" << uv_size << std::endl;
            		}


                }
            }
        } else {

            for(int i = 0; i < meta.height; i+=2)
            {
                for(int j = 0; j < meta.width*2;j+=2){

                    img.data[(i * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j + 1];
                    img.data[(i * meta.width * 2) + (j * 2) + 1] = frame[(i * meta.width) + j];
                    img.data[(i * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j];
                    img.data[(i * meta.width * 2) + (j * 2) + 3] = frame[(i * meta.width) + j + 1];

                    img.data[((i+1) * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j + 1];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 1] = frame[((i+1) * meta.width) + j];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 3] = frame[((i+1) * meta.width) + j + 1];

                }
            }

        }

        publisher.publish(img);

    } else if(meta.format == IMAGE_FORMAT_YUV422_UYVY) {

        img.header.frame_id = interface->ginterface_name;
        img.is_bigendian = false;
        img.step = meta.width * GetStepSize(IMAGE_FORMAT_YUV422);
        img.encoding = GetRosFormat(IMAGE_FORMAT_YUV422);

        int dataSize = img.step * img.height;
        img.data.resize(dataSize);

        for (int i = 0; i < meta.height; ++i)
        {
            for (int j = 0; j < meta.width; j += 2){

                int uyvy_index = i * meta.width * 2 + j * 2;
                int yuv_index = i * meta.width * 2 + j * 2;

                // Copy UYVY data to YUV422 format (YUYV)
                img.data[yuv_index] = frame[uyvy_index + 1];     // Y1
                img.data[yuv_index + 1] = frame[uyvy_index];     // U
                img.data[yuv_index + 2] = frame[uyvy_index + 2]; // Y2
                img.data[yuv_index + 3] = frame[uyvy_index + 3]; // V

            }
        }

        publisher.publish(img);

    } else if(meta.format == IMAGE_FORMAT_RAW8) {

        img.header.frame_id = interface->ginterface_name;
        img.is_bigendian = false;

        img.step     = meta.width * GetStepSize(meta.format);
       	img.encoding = GetRosFormat(meta.format);

        int raw_dataSize = img.step * img.height;

        img.data.resize(raw_dataSize);

        memcpy(&(img.data[0]), frame, raw_dataSize);

        publisher.publish(img);

    } else if (meta.format == IMAGE_FORMAT_H264) {
        // Fill out the image msg header
        compressedImage.header.frame_id = interface->ginterface_name;
        compressedImage.header.stamp.nanosec = meta.timestamp_ns;

        // Fill out image data
        compressedImage.format = GetRosFormat(IMAGE_FORMAT_H264);
        int h264_dataSize = meta.size_bytes;

        compressedImage.data.resize(h264_dataSize);

        memcpy(&(compressedImage.data[0]), frame, h264_dataSize);

        compressed_publisher->publish(compressedImage);


    } else if (meta.format == IMAGE_FORMAT_H265) {
        // Fill out the image msg header
        compressedImage.header.frame_id = interface->ginterface_name;
        compressedImage.header.stamp.nanosec = meta.timestamp_ns;

        // Fill out image data
        compressedImage.format = GetRosFormat(IMAGE_FORMAT_H265);
        int dataSize = meta.size_bytes;

        compressedImage.data.resize(dataSize);

        memcpy(&(compressedImage.data[0]), frame, dataSize);

        compressed_publisher->publish(compressedImage);

    } else {

        img.header.frame_id = interface->ginterface_name;
        img.is_bigendian = false;
        img.step     = meta.width * GetStepSize(meta.format);
        img.encoding = GetRosFormat(meta.format);

        int dataSize = img.step * img.height;

        img.data.resize(dataSize);

        memcpy(&(img.data[0]), frame, dataSize);

        publisher.publish(img);

    }
}

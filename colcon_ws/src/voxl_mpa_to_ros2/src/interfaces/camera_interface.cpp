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

#include "voxl_mpa_to_ros2/utils/camera_helpers.h"
#include "voxl_mpa_to_ros2/interfaces/camera_interface.h"

static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta,
                            char* frame,
                            void* context);

CameraInterface::CameraInterface(
    rclcpp::Node::SharedPtr nh,
    const char * name) :
    GenericInterface(nh, name)
{
    // Generic interface name
    ginterface_name = name;

    // Pipe name to determine if encoded
    pipeName = m_pipeName;

    pipe_client_set_camera_helper_cb(m_channel, _frame_cb, this);

    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_CAMERA_HELPER | CLIENT_FLAG_START_PAUSED, 0)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

}

void CameraInterface::AdvertiseTopics(){

    image_transport::ImageTransport it(m_rosNodeHandle);

    if (pipeName.find("encoded") != std::string::npos) {

      m_rosCompressedPublisher_ = m_rosNodeHandle->create_publisher<sensor_msgs::msg::CompressedImage>(m_pipeName, 1);
    }

    else {
      m_rosImagePublisher = it.advertise(m_pipeName, 1);
    }

    m_state = ST_AD;

}

void CameraInterface::StopAdvertising(){

    if (pipeName.find("encoded") != std::string::npos) {
      m_rosCompressedPublisher_.reset();
    }
    else {
      m_rosImagePublisher.shutdown();
    }
    m_state = ST_CLEAN;
}

int CameraInterface::GetNumClients(){

    if (pipeName.find("encoded") != std::string::npos) {
      return m_rosCompressedPublisher_->get_subscription_count();
    }
    else {
      return m_rosImagePublisher.getNumSubscribers();
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


    // Fill out the image message based on image format from metadata
    switch (meta.format) {
        case IMAGE_FORMAT_NV12:
          {
            sensor_msgs::msg::Image& nv12_img = interface->GetImageMsg();

            // Fill out the image msg header
            nv12_img.header.frame_id = interface->ginterface_name;
            nv12_img.is_bigendian = false;
            nv12_img.header.stamp.nanosec = meta.timestamp_ns;

            // Fill out image dimensions
            nv12_img.width    = meta.width;
            nv12_img.height   = meta.height;

            nv12_img.step = meta.width * GetStepSize(IMAGE_FORMAT_YUV422);
            nv12_img.encoding = GetRosFormat(IMAGE_FORMAT_YUV422);

            int nv12_dataSize = nv12_img.step * nv12_img.height;
            nv12_img.data.resize(nv12_dataSize);

            char *uv = &(frame[nv12_dataSize/2]);

            for(int i = 0; i < meta.height; i+=2)
            {
                for(int j = 0; j < meta.width*2;j+=2){

                    nv12_img.data[(i * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j];
                    nv12_img.data[(i * meta.width * 2) + (j * 2) + 1] = frame[(i * meta.width) + j];
                    nv12_img.data[(i * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j + 1];
                    nv12_img.data[(i * meta.width * 2) + (j * 2) + 3] = frame[(i * meta.width) + j + 1];

                    nv12_img.data[((i+1) * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j];
                    nv12_img.data[((i+1) * meta.width * 2) + (j * 2) + 1] = frame[((i+1) * meta.width) + j];
                    nv12_img.data[((i+1) * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j + 1];
                    nv12_img.data[((i+1) * meta.width * 2) + (j * 2) + 3] = frame[((i+1) * meta.width) + j + 1];

                }
            }

            publisher.publish(nv12_img);
            break;
          }
        case IMAGE_FORMAT_NV21:
          {
            sensor_msgs::msg::Image& nv21_img = interface->GetImageMsg();

            // Fill out the image msg header
            nv21_img.header.frame_id = interface->ginterface_name;
            nv21_img.is_bigendian = false;
            nv21_img.header.stamp.nanosec = meta.timestamp_ns;

            // Fill out image dimensions
            nv21_img.width    = meta.width;
            nv21_img.height   = meta.height;

            nv21_img.step = meta.width * GetStepSize(IMAGE_FORMAT_YUV422);
            nv21_img.encoding = GetRosFormat(IMAGE_FORMAT_YUV422);

            int nv21_dataSize = nv21_img.step * nv21_img.height;
            nv21_img.data.resize(nv21_dataSize);

            char *nv21_uv = &(frame[nv21_dataSize/2]);

            for(int i = 0; i < meta.height; i+=2)
            {
                for(int j = 0; j < meta.width*2;j+=2){

                    nv21_img.data[(i * meta.width * 2) + (j * 2) + 0] = nv21_uv[((i/2) * meta.width) + j + 1];
                    nv21_img.data[(i * meta.width * 2) + (j * 2) + 1] = frame[(i * meta.width) + j];
                    nv21_img.data[(i * meta.width * 2) + (j * 2) + 2] = nv21_uv[((i/2) * meta.width) + j];
                    nv21_img.data[(i * meta.width * 2) + (j * 2) + 3] = frame[(i * meta.width) + j + 1];

                    nv21_img.data[((i+1) * meta.width * 2) + (j * 2) + 0] = nv21_uv[((i/2) * meta.width) + j + 1];
                    nv21_img.data[((i+1) * meta.width * 2) + (j * 2) + 1] = frame[((i+1) * meta.width) + j];
                    nv21_img.data[((i+1) * meta.width * 2) + (j * 2) + 2] = nv21_uv[((i/2) * meta.width) + j];
                    nv21_img.data[((i+1) * meta.width * 2) + (j * 2) + 3] = frame[((i+1) * meta.width) + j + 1];

                }
            }

            publisher.publish(nv21_img);
            break;
          }
        case IMAGE_FORMAT_RAW8:
           {
            sensor_msgs::msg::Image& raw_img = interface->GetImageMsg();

            // Fill out the image msg header
            raw_img.header.frame_id = interface->ginterface_name;
            raw_img.is_bigendian = false;
            raw_img.header.stamp.nanosec = meta.timestamp_ns;

            // Fill out image dimensions

	        raw_img.step     = meta.width * GetStepSize(meta.format);
       		raw_img.encoding = GetRosFormat(meta.format);

            int raw_dataSize = raw_img.step * raw_img.height;

            raw_img.data.resize(raw_dataSize);

            memcpy(&(raw_img.data[0]), frame, raw_dataSize);

            publisher.publish(raw_img);
            
            break;
           }
        // Encoded image formats
        default:
           {
            auto c_img = interface->GetCompressedImageMsg();

            // Fill out the image msg header
            c_img.header.frame_id = interface->ginterface_name;
            c_img.header.stamp.nanosec = meta.timestamp_ns;

            // Fill out image data
            c_img.format = GetRosFormat(meta.format);
            int dataSize = meta.size_bytes;

            c_img.data.resize(dataSize);

            memcpy(&(c_img.data[0]), frame, dataSize);

            interface->m_rosCompressedPublisher_->publish(c_img);

            break;
          }
    }

}

/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
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
#include "voxl_mpa_to_ros2/interfaces/imu_interface.hpp"
#include "voxl_mpa_to_ros2/utils/common_utils.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <px4_ros_com/frame_transforms.h>

static void _helper_cb(
    __attribute__((unused))int ch, 
                           char* data, 
                           int bytes, 
                           void* context);

IMUInterface::IMUInterface(
    rclcpp::Node::SharedPtr nh,
    const char *    name) :
    GenericInterface(nh, name)
{
    std::string imu_frame;
    if (!nh->has_parameter("imu_frame")) {
        imu_frame = nh->declare_parameter("imu_frame", "uav1/IMU");
    } else {
        nh->get_parameter("imu_frame",imu_frame);
    }

    pipe_client_set_simple_helper_cb(m_channel, _helper_cb, this);

    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER | CLIENT_FLAG_START_PAUSED,
                IMU_RECOMMENDED_READ_BUF_SIZE)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

}

void IMUInterface::AdvertiseTopics(){

    char topicName[64];

    sprintf(topicName, "%s", m_pipeName);
    imu_pub_ = m_rosNodeHandle->create_publisher<sensor_msgs::msg::Imu>
        (topicName, rclcpp::SensorDataQoS());

    m_state = ST_AD;

}

void IMUInterface::StopAdvertising(){

    imu_pub_.reset();

    m_state = ST_CLEAN;

}

int IMUInterface::GetNumClients(){
    return imu_pub_->get_subscription_count();
}

// called when the simple helper has data for us
static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
{

    // validate that the data makes sense
    int n_packets;
    imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);
    if(data_array == NULL) return;

    //std::cout << "got data!!" << std::endl;
    IMUInterface *interface = (IMUInterface *) context;
    if(interface->GetState() != ST_RUNNING) return;
    sensor_msgs::msg::Imu& imu = interface->GetImuMsg();

    // make a new data struct to hold the average
    imu_data_t avg;
    memset(&avg,0,sizeof(avg));

    //publish all the samples
    for(int i=0;i<n_packets;i++){

        Eigen::Vector3d gyro, accel;

            // rotate to ENU
            auto gyro_frd = Eigen::Vector3d(
                data_array[i].gyro_rad[0], 
                data_array[i].gyro_rad[1], 
                data_array[i].gyro_rad[2]);
            gyro = px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame(gyro_frd);

            auto accel_frd = Eigen::Vector3d(
                data_array[i].accl_ms2[0], 
                data_array[i].accl_ms2[1], 
                data_array[i].accl_ms2[2]);
            accel = px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame(accel_frd);

        imu.header.stamp = _clock_monotonic_to_ros_time(
            interface->getNodeHandle(),data_array[i].timestamp_ns);
        imu.angular_velocity.x = gyro[0];
        imu.angular_velocity.y = gyro[1];
        imu.angular_velocity.z = gyro[2];
        imu.linear_acceleration.x = accel[0];
        imu.linear_acceleration.y = accel[1];
        imu.linear_acceleration.z = accel[2];

        interface->imu_pub_->publish(imu);

    }


    return;
}

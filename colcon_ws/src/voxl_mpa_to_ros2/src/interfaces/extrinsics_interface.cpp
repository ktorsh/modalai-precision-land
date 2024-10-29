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
#include "voxl_mpa_to_ros2/interfaces/extrinsics_interface.h"

ExtrinsicsInterface::ExtrinsicsInterface(
    rclcpp::Node::SharedPtr nh,
    const char *    name) :
    GenericInterface(nh, name)
{
    br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_rosNodeHandle);
    m_timer = nh->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {this->ReadandPublishConfig(); });
}

void ExtrinsicsInterface::ReadandPublishConfig(){
    std::ifstream config_doc("/etc/modalai/extrinsics.conf", std::ifstream::binary);
    if (!config_doc.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("Extrinsic_file.convert"), "Unable to open configuration file");
        return;
    }

    Json::Value root;
    config_doc >> root;

    const Json::Value extrinsics = root["extrinsics"];
    
    for (const auto& extrinsic : extrinsics) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.frame_id = extrinsic["parent"].asString();
        transform.child_frame_id = extrinsic["child"].asString();
        
        Json::Value T = extrinsic["T_child_wrt_parent"];
        transform.transform.translation.x = T[0].asFloat();
        transform.transform.translation.y = T[1].asFloat();
        transform.transform.translation.z = T[2].asFloat();

        Json::Value RPY = extrinsic["RPY_parent_to_child"];
        double roll = RPY[0].asFloat();
        double pitch = RPY[1].asFloat();
        double yaw = RPY[2].asFloat();

        tf2::Quaternion q;
        q.setRPY(roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        transforms_.push_back(transform);

        transform.header.stamp = m_rosNodeHandle->get_clock()->now();

        br_->sendTransform(transform);
    }
}

void ExtrinsicsInterface::AdvertiseTopics(){
    return;
}

void ExtrinsicsInterface::StopAdvertising(){
    return;
}

int ExtrinsicsInterface::GetNumClients(){
    return 0;
}

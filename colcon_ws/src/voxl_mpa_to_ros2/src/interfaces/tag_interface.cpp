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
 #include "voxl_mpa_to_ros2/interfaces/tag_interface.hpp"
 #include "voxl_mpa_to_ros2/utils/common_utils.h"
 #include <Eigen/Eigen>
 #include <Eigen/Geometry>
 #include <px4_ros_com/frame_transforms.h>
 
 static void _helper_cb(
     __attribute__((unused))int ch, 
                            char* data, 
                            int bytes, 
                            void* context);
 
 TagInterface::TagInterface(
     rclcpp::Node::SharedPtr nh,
     const char *    name) :
     GenericInterface(nh, name)
 {
 
     pipe_client_set_simple_helper_cb(m_channel, _helper_cb, this);
 
     if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                 EN_PIPE_CLIENT_SIMPLE_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                 TAG_DETECTION_RECOMMENDED_READ_BUF_SIZE)){
         pipe_client_close(m_channel);//Make sure we unclaim the channel
         throw -1;
     }
 
 }
 
 void TagInterface::AdvertiseTopics(){
 
    char topicName[64];
 
    sprintf(topicName, "%s", m_pipeName);
     pose_pub_ = m_rosNodeHandle->create_publisher<geometry_msgs::msg::PoseStamped>
         (topicName, rclcpp::SensorDataQoS());
     //sprintf(topicName, "%s/odom", m_pipeName);
     //odom_pub_ = m_rosNodeHandle->create_publisher<nav_msgs::msg::Odometry>
     //    (topicName, rclcpp::SensorDataQoS());
 
     m_state = ST_AD;
 
 }
 
 void TagInterface::StopAdvertising(){
 
     pose_pub_.reset();
     //odom_pub_.reset();
 
     m_state = ST_CLEAN;
 
 }
 
 int TagInterface::GetNumClients(){
     //return pose_pub_->get_subscription_count() + odom_pub_->get_subscription_count();
     return pose_pub_->get_subscription_count();
 }
 
 // called when the simple helper has data for us
 static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
 {
     if(bytes<=0){
         printf("No bytes through pipe");
         return;
     }
     int n_packets;
     // check for 4dof pose packets
 
    tag_detection_t* pose_array = pipe_validate_tag_detection_t(data, bytes, &n_packets);
      if(pose_array==NULL){
          printf("ERROR, failed to validate apriltag packets\n");
          return;
      }
 
     TagInterface *interface = (TagInterface *) context;
     if(interface->GetState() != ST_RUNNING) return;
     
     geometry_msgs::msg::PoseStamped& poseMsg = interface->GetPoseMsg();
     //nav_msgs::msg::Odometry& odomMsg = interface->GetOdomMsg();
 
     for(int i=0;i<n_packets;i++){
        tag_detection_t data = pose_array[i];
 
        poseMsg.header.stamp = _clock_monotonic_to_ros_time(interface->getNodeHandle(), data.timestamp_ns);
        //odomMsg.header.stamp = _clock_monotonic_to_ros_time(interface->getNodeHandle(), data.timestamp_ns);
        poseMsg.header.frame_id = std::to_string(data.id);

        // extract quaternion from {imu w.r.t vio} rotation matrix
        tf2::Matrix3x3 R(
             data.R_tag_to_cam[0][0],
             data.R_tag_to_cam[0][1],
             data.R_tag_to_cam[0][2],
             data.R_tag_to_cam[1][0],
             data.R_tag_to_cam[1][1],
             data.R_tag_to_cam[1][2],
             data.R_tag_to_cam[2][0],
             data.R_tag_to_cam[2][1],
             data.R_tag_to_cam[2][2]);
        tf2::Quaternion q;
        R.getRotation(q);
 
        poseMsg.pose.position.x = data.T_tag_wrt_cam[0];
        poseMsg.pose.position.y = data.T_tag_wrt_cam[1];
        poseMsg.pose.position.z = data.T_tag_wrt_cam[2];
        poseMsg.pose.orientation.x = q.getX();
        poseMsg.pose.orientation.y = q.getY();
        poseMsg.pose.orientation.z = q.getZ();
        poseMsg.pose.orientation.w = q.getW();
        interface->pose_pub_->publish(poseMsg);
 
         //odomMsg.pose.pose = poseMsg.pose;
         //odomMsg.twist.twist.linear.x = data.v_child_wrt_parent[0];
        //  odomMsg.twist.twist.linear.y = data.v_child_wrt_parent[1];
        //  odomMsg.twist.twist.linear.z = data.v_child_wrt_parent[2];
        //  odomMsg.twist.twist.angular.x = data.w_child_wrt_child[0];
        //  odomMsg.twist.twist.angular.y = data.w_child_wrt_child[1];
        //  odomMsg.twist.twist.angular.z = data.w_child_wrt_child[2];
 
        //  interface->odom_pub_->publish(odomMsg);
     }

     return;
 }
 
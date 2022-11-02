/**
 * @file transform_odom.h
 * @brief 
 * @author Clemens Hofbauer
 * @date November 02, 2022: created
 * @copyright Copyright (C) 2022 Austrian Institute of Technologies GmbH. All rights reserved.
 */

#ifndef POINTCLOUD_TRANSFORM_H_
#define POINTCLOUD_TRANSFORM_H_

/*************** DEFINES ***********************************************************************/
#define ENABLE_TIME_MEASUREMENTS false

/*************** INCLUDES **********************************************************************/
// STL
#include <iostream>
#include <chrono>
#include <functional>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <std_msgs/msg/header.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "rclcpp/graph_listener.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

// Messages
#include <sensor_msgs/msg/point_cloud2.hpp>




/*************** CONSTANTS *********************************************************************/

/*************** USING DIRECTIVES **************************************************************/

/*************** TYPEDEFS **********************************************************************/

/***********************************************************************************************/
/*************** CLASS DECLARATION *************************************************************/
/***********************************************************************************************/

/*! \class ElevationMapping
 *
 * \brief 
 *
 */
class TFOdom : public rclcpp::Node
{

public:


  /**
   * \brief Constructor.
   *
   *
   **/
  TFOdom();

  /**
   * \brief Destructor.
   *
   **/
  ~TFOdom();


  /**
   * \brief Callback for odometry message.
   **/
  void OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr&);

  /******************************************************************************************************************************************/
  /******************************************************************************************************************************************/

private:
    /**
   * \brief Reads parameters.
   **/
  void ReadParam();

  /**
   * \brief Initialize node.
   **/
  void Init();

  /******************************************************************************************************************************************/

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_viso_odometry_;
  std::string tpc_sub_viso_odom_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_; 
  std::string tpc_pub_odometry_;
  std::string transformed_odom_frame_;
  nav_msgs::msg::Odometry transformedOdom_;

  // QoS profile for subscriber message definition
  rmw_qos_profile_t custom_qos_profile_;

   // TF Broadcaster
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  tf2::Quaternion q_;
  geometry_msgs::msg::TransformStamped t_;


  // Debug Mode
  bool debugMode_;

  };

#endif //  POINTCLOUD_TRANSFORM_H_

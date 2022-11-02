/**
 * @file trnasform_odom.cpp
 * @brief 
 * @author Clemens Hofbauer
 * @date November 02, 2022: created
 * @copyright Copyright (C) 2022 Austrian Institute of Technologies GmbH. All rights reserved.
 */

/*************** DEFINES ***********************************************************************/

/*************** INCLUDES **********************************************************************/
#include <../include/transform_odom.h>
/**************************************************************************************************
 *
 */
TFOdom::TFOdom() : Node("pc_undistortion")
{
  ReadParam();
  Init();
}

/**************************************************************************************************
 *
 */
TFOdom::~TFOdom()
{
}

/**************************************************************************************************
 *
 */
void TFOdom::ReadParam()
{
  try
  {
    // define parameter descriptor for description of each parameter
    rcl_interfaces::msg::ParameterDescriptor param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    param_desc.description = ""; //TODO
    tpc_sub_viso_odom_ = this->declare_parameter<std::string>("sub_odom_topic", "/odometry", param_desc);
   
    param_desc.description = ""; //TODO
    tpc_pub_odometry_= this->declare_parameter<std::string>("pub_odom_topic", "/tf_odometry", param_desc);

    param_desc.description = ""; //TODO
    transformed_odom_frame_= this->declare_parameter<std::string>("transformed_odom_frame", "world", param_desc);


  }
  catch(const std::exception& e)
  {
    std::cerr << "Error during param handling!\n" << e.what() << "\n";
    exit(0);
  }
}

/**************************************************************************************************
 *
 */
void TFOdom::Init()
{
  // QoS with fail safe
  std::vector<rclcpp::TopicEndpointInfo> topic_info;
  for (int i = 0; i < 10; i++)
  {
    topic_info  = get_publishers_info_by_topic(tpc_sub_viso_odom_);
    if (topic_info.size() != 0){
      custom_qos_profile_.reliability = topic_info.at(0).qos_profile().get_rmw_qos_profile().reliability;
      custom_qos_profile_.durability = topic_info.at(0).qos_profile().get_rmw_qos_profile().durability;
      break;
    }
    RCLCPP_WARN(this->get_logger(), "Automatic QoS detection failed, trying again in 1 s");  
    sleep(1);
    if (i == 9){
      RCLCPP_ERROR(this->get_logger(), "Couldnt apply automatic QoS");
      std::exit(0);
    }

  }
  custom_qos_profile_.depth = 10;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), custom_qos_profile_);

  // Init subscribers
  sub_viso_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(tpc_sub_viso_odom_, qos, std::bind(&TFOdom::OdometryCallback, this, std::placeholders::_1));

  // Init publisher
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(tpc_pub_odometry_ , 10);

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  RCLCPP_INFO(this->get_logger(), "Init done!");
}
/**************************************************************************************************
 * 
 */
void TFOdom::OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& visoOdometry){

  transformedOdom_.header = visoOdometry->header;
  transformedOdom_.header.frame_id = transformed_odom_frame_;


  //write the image coordinates to the vehicle coordinates (see: https://www.cvlibs.net/software/libviso/ for representation)
  //position
  transformedOdom_.pose.pose.position.x = visoOdometry->pose.pose.position.z;
  transformedOdom_.pose.pose.position.y = -visoOdometry->pose.pose.position.x;
  transformedOdom_.pose.pose.position.z = -visoOdometry->pose.pose.position.y;

  //orientation
  q_.setX(visoOdometry->pose.pose.orientation.x);
  q_.setY(visoOdometry->pose.pose.orientation.y);
  q_.setZ(visoOdometry->pose.pose.orientation.z);
  q_.setW(visoOdometry->pose.pose.orientation.w);
  double roll_x, pitch_y, yaw_z;
  tf2::Matrix3x3 m(q_);
  m.getRPY(roll_x, pitch_y, yaw_z);
  
  //yaw += M_PI/2; ///turn 90 degrees
  roll_x = -roll_x;
  pitch_y = -pitch_y;

  q_.setRPY(yaw_z, roll_x, pitch_y);

  //orientation
  transformedOdom_.pose.pose.orientation.x = q_.getX();
  transformedOdom_.pose.pose.orientation.y = q_.getY();
  transformedOdom_.pose.pose.orientation.z = q_.getZ();
  transformedOdom_.pose.pose.orientation.w = q_.getW();

  //linear twist
  transformedOdom_.twist.twist.linear.x = visoOdometry->twist.twist.linear.z;
  transformedOdom_.twist.twist.linear.y = -visoOdometry->twist.twist.linear.x;
  transformedOdom_.twist.twist.linear.z = -visoOdometry->twist.twist.linear.y;

  //angular twist
  transformedOdom_.twist.twist.angular.x = visoOdometry->twist.twist.angular.z;
  transformedOdom_.twist.twist.angular.y = -visoOdometry->twist.twist.angular.x;
  transformedOdom_.twist.twist.angular.z = -visoOdometry->twist.twist.angular.y;

  //TODO Covariance

  pub_odometry_->publish(transformedOdom_);

}

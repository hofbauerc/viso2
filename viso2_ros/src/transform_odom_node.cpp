
#include <../include/transform_odom.h>

int main(int argc, char* argv[])

{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TFOdom>());

  rclcpp::shutdown();


  return 0;

}
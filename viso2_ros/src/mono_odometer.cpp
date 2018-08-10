#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <libviso2/viso_mono.h>

#include <viso2_ros/VisoInfo.h>

#include "odometer_base.h"
#include "odometry_params.h"

/** Create a feature flow visualization window */
//#define DBG_VISUALIZATION_IMAGES
#ifdef DBG_VISUALIZATION_IMAGES
/** Provide a "path/to/images" for storing flow visualization images there */
#define DBG_VISUALIZATION_IMAGES_PATH "path/to/images"
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/opencv.hpp>
#include <strstream>
#endif

namespace viso2_ros
{

// doubled the values from stereo odometer
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.2, 0, 0, 0, 0, 0,
    0, 0.2, 0, 0, 0, 0,
    0, 0, 0.2, 0, 0, 0,
    0, 0, 0, 0.34, 0, 0,
    0, 0, 0, 0, 0.34, 0,
    0, 0, 0, 0, 0, 0.34 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.10, 0, 0, 0, 0, 0,
    0, 0.10, 0, 0, 0, 0,
    0, 0, 0.10, 0, 0, 0,
    0, 0, 0, 0.18, 0, 0,
    0, 0, 0, 0, 0.18, 0,
    0, 0, 0, 0, 0, 0.18 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
    0, 9999, 0, 0, 0, 0,
    0, 0, 9999, 0, 0, 0,
    0, 0, 0, 9999, 0, 0,
    0, 0, 0, 0, 9999, 0,
    0, 0, 0, 0, 0, 9999 } };


class MonoOdometer : public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryMono> visual_odometer_;
  VisualOdometryMono::parameters visual_odometer_params_;

  image_transport::CameraSubscriber camera_sub_;

  ros::Publisher info_pub_;

  bool replace_;

public:

  MonoOdometer(const std::string& transport) : OdometerBase(), replace_(false)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    camera_sub_ = it.subscribeCamera("image", 1, &MonoOdometer::imageCallback, this, transport);

    info_pub_ = local_nh.advertise<VisoInfo>("info", 1);
  }

protected:

  void imageCallback(
      const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ros::WallTime start_time = ros::WallTime::now();
 
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      // read calibration info from camera info message
      // to fill remaining odometer parameters
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);
      visual_odometer_params_.calib.f  = model.fx();
      visual_odometer_params_.calib.cu = model.cx();
      visual_odometer_params_.calib.cv = model.cy();
      visual_odometer_.reset(new VisualOdometryMono(visual_odometer_params_));
      if (image_msg->header.frame_id != "") setSensorFrameId(image_msg->header.frame_id);
      ROS_INFO_STREAM("Initialized libviso2 mono odometry "
                      "with the following parameters:" << std::endl << 
                      visual_odometer_params_);
    }

    // convert image if necessary
    uint8_t *image_data;
    int step;
    cv_bridge::CvImageConstPtr cv_ptr;
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      image_data = const_cast<uint8_t*>(&(image_msg->data[0]));
      step = image_msg->step;
    }
    else
    {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
      image_data = cv_ptr->image.data;
      step = cv_ptr->image.step[0];
    }

    // run the odometer
    int32_t dims[] = {image_msg->width, image_msg->height, step};
    // on first run, only feed the odometer with first image pair without
    // retrieving data
    if (first_run)
    {
      visual_odometer_->process(image_data, dims);
      tf::Transform delta_transform;
      delta_transform.setIdentity();
      integrateAndPublish(delta_transform, image_msg->header.stamp);
    }
    else
    {
      bool success = visual_odometer_->process(image_data, dims);
      if(success)
      {
        replace_ = false;
        Matrix camera_motion = Matrix::inv(visual_odometer_->getMotion());
        ROS_DEBUG("Found %i matches with %i inliers.", 
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << camera_motion);

        tf::Matrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        setPoseCovariance(STANDARD_POSE_COVARIANCE);
        setTwistCovariance(STANDARD_TWIST_COVARIANCE);

        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }
      else
      {
        setPoseCovariance(BAD_COVARIANCE);
        setTwistCovariance(BAD_COVARIANCE);
        ROS_DEBUG("Call to VisualOdometryMono::process() failed. Assuming motion too small.");
        replace_ = true;
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }

#ifdef DBG_VISUALIZATION_IMAGES
      ROS_INFO("Start creating visualization image");

      cv_bridge::CvImagePtr pImgL = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);

      cv::Point2f pt1;
      cv::Point2f pt2;
      cv::Point2f ptd;

      float ptDist = 0.0f;

      Matcher::p_match 	goodMatch;

      cv::Mat pxVal(1, 1, CV_8UC1, cv::Scalar(0));
      cv::Mat pxRGB(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));

      std::vector<Matcher::p_match> matches = visual_odometer_->getMatches();
      std::vector<int> inlier_indices = visual_odometer_->getInlierIndices();

//      int widthPt = 5, widthLi = 2;
      int widthPt = 2, widthLi = 1;

      for (u_int32_t i = 0; i < inlier_indices.size(); ++i)
      {
    	  goodMatch = matches[inlier_indices[i]];

    	  pt1.x = goodMatch.u1p; 	pt1.y = goodMatch.v1p;
    	  pt2.x = goodMatch.u1c; 	pt2.y = goodMatch.v1c;

    	  // Colormap
    	  ptd = pt2 - pt1;
    	  ptDist = sqrtf(ptd.x * ptd.x + ptd.y * ptd.y);
    	  ptDist = rintf(ptDist * (84.0f / 35.0f));
    	  pxVal.at<uchar>(0,0) = 84 - (ptDist <= 84 ? ptDist : 84);
    	  cv::applyColorMap(pxVal, pxRGB, cv::COLORMAP_HSV);

    	  line(pImgL->image, pt1, pt2, cv::Scalar(pxRGB.at<cv::Vec3b>(0, 0)[0], pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]), widthLi);
    	  line(pImgL->image, pt1, pt1, cv::Scalar(pxRGB.at<cv::Vec3b>(0, 0)[0], pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]), widthPt);
      }

      imshow("matches", pImgL->image);
      cv::waitKey(1);

      // Write the image to disk
  #ifdef DBG_VISUALIZATION_IMAGES_PATH
      std::stringstream fnStream("");
      fnStream << std::string(DBG_VISUALIZATION_IMAGES_PATH) << "/mono_img_";
      fnStream << std::setw(8) << std::setfill ('0') << image_msg->header.seq << ".png";
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);
      cv::imwrite(fnStream.str(), pImgL->image, compression_params);
  #endif
#endif

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp = image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = false;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }
};

} // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono_odometer");
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("mono_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  viso2_ros::MonoOdometer odometer(transport);
  
  ros::spin();
  return 0;
}


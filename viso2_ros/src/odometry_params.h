#pragma once

#include <rclcpp/rclcpp.hpp>

#include <libviso2/viso_stereo.h>
#include <libviso2/viso_mono.h>

namespace viso2_ros
{

    namespace odometry_params
    {

        using rclcpp::node_interfaces::NodeParametersInterface;

/// loads matcher params
        void load_params(NodeParametersInterface::SharedPtr params, Matcher::parameters & out)
        {
            out.nms_n =
                    params->declare_parameter("match.nms_n", rclcpp::ParameterValue(out.nms_n)).get<int>();
            out.nms_tau = params->declare_parameter(
                    "match.nms_tau",
                    rclcpp::ParameterValue(out.nms_tau)).get<int>();
            out.match_binsize =
                    params->declare_parameter(
                            "match.match_binsize",
                            rclcpp::ParameterValue(out.match_binsize)).get<int>();
            out.match_radius =
                    params->declare_parameter(
                            "match.match_radius",
                            rclcpp::ParameterValue(out.match_radius)).get<int>();
            out.match_disp_tolerance = params->declare_parameter(
                    "match.match_disp_tolerance",
                    rclcpp::ParameterValue(out.match_disp_tolerance)).get<int>();
            out.outlier_disp_tolerance = params->declare_parameter(
                    "match.outlier_disp_tolerance",
                    rclcpp::ParameterValue(out.outlier_disp_tolerance)).get<int>();
            out.outlier_flow_tolerance = params->declare_parameter(
                    "match.outlier_flow_tolerance",
                    rclcpp::ParameterValue(out.outlier_flow_tolerance)).get<int>();
            out.multi_stage =
                    params->declare_parameter(
                            "match.multi_stage",
                            rclcpp::ParameterValue(out.multi_stage)).get<int>();
            out.half_resolution =
                    params->declare_parameter(
                            "match.half_resolution",
                            rclcpp::ParameterValue(out.half_resolution)).get<int>();
            out.refinement =
                    params->declare_parameter(
                            "match.refinement",
                            rclcpp::ParameterValue(out.refinement)).get<int>();
        }

/// loads bucketing params
        void load_params(NodeParametersInterface::SharedPtr params, VisualOdometry::bucketing & bucketing)
        {
            bucketing.max_features =
                    params->declare_parameter(
                            "bucket.max_features",
                            rclcpp::ParameterValue(bucketing.max_features)).get<int>();
            bucketing.bucket_width =
                    params->declare_parameter(
                            "bucket.bucket_width",
                            rclcpp::ParameterValue(bucketing.bucket_width)).get<double>();
            bucketing.bucket_height = params->declare_parameter(
                    "bucket.bucket_height",
                    rclcpp::ParameterValue(bucketing.bucket_height)).get<double>();
        }

/// loads common odometry params
        void load_common_params(NodeParametersInterface::SharedPtr params, VisualOdometry::parameters & out)
        {
            load_params(params, out.match);
            load_params(params, out.bucket);
        }

/// loads common & stereo specific params
        void load_params(NodeParametersInterface::SharedPtr params, VisualOdometryStereo::parameters & out)
        {
            load_common_params(params, out);
            out.ransac_iters =
                    params->declare_parameter(
                            "ransac_iters",
                            rclcpp::ParameterValue(out.ransac_iters)).get<int>();
            out.inlier_threshold =
                    params->declare_parameter(
                            "inlier_threshold",
                            rclcpp::ParameterValue(out.inlier_threshold)).get<double>();
            out.reweighting =
                    params->declare_parameter(
                            "reweighting",
                            rclcpp::ParameterValue(out.reweighting)).get<bool>();
            out.cov_svd_factor =
                    params->declare_parameter(
                            "cov_svd_factor",
                            rclcpp::ParameterValue(out.cov_svd_factor)).get<double>();
        }

    } // end of namespace

    std::ostream & operator<<(std::ostream & out, const Matcher::parameters & params)
    {
        out << "  nms_n                  = " << params.nms_n << std::endl;
        out << "  nms_tau                = " << params.nms_tau << std::endl;
        out << "  match_binsize          = " << params.match_binsize << std::endl;
        out << "  match_radius           = " << params.match_radius << std::endl;
        out << "  match_disp_tolerance   = " << params.match_disp_tolerance << std::endl;
        out << "  outlier_disp_tolerance = " << params.outlier_disp_tolerance << std::endl;
        out << "  outlier_flow_tolerance = " << params.outlier_flow_tolerance << std::endl;
        out << "  multi_stage            = " << params.multi_stage << std::endl;
        out << "  half_resolution        = " << params.half_resolution << std::endl;
        out << "  refinement             = " << params.refinement << std::endl;
        return out;
    }

    std::ostream & operator<<(std::ostream & out, const VisualOdometry::calibration & calibration)
    {
        out << "  f  = " << calibration.f << std::endl;
        out << "  cu = " << calibration.cu << std::endl;
        out << "  cv = " << calibration.cv << std::endl;
        return out;
    }

    std::ostream & operator<<(std::ostream & out, const VisualOdometry::bucketing & bucketing)
    {
        out << "  max_features  = " << bucketing.max_features << std::endl;
        out << "  bucket_width  = " << bucketing.bucket_width << std::endl;
        out << "  bucket_height = " << bucketing.bucket_height << std::endl;
        return out;
    }

    std::ostream & operator<<(std::ostream & out, const VisualOdometry::parameters & params)
    {
        out << "Calibration parameters:" << std::endl << params.calib;
        out << "Matcher parameters:" << std::endl << params.match;
        out << "Bucketing parameters:" << std::endl << params.bucket;
        return out;
    }

    std::ostream & operator<<(std::ostream & out, const VisualOdometryStereo::parameters & params)
    {
        out << static_cast<VisualOdometry::parameters>(params);
        out << "Stereo odometry parameters:" << std::endl;
        out << "  base             = " << params.base << std::endl;
        out << "  ransac_iters     = " << params.ransac_iters << std::endl;
        out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
        out << "  reweighting      = " << params.reweighting << std::endl;
        return out;
    }

    std::ostream & operator<<(std::ostream & out, const VisualOdometryMono::parameters & params)
    {
        out << static_cast<VisualOdometry::parameters>(params);
        out << "Mono odometry parameters:" << std::endl;
        out << "  camera_height    = " << params.height << std::endl;
        out << "  camera_pitch     = " << params.pitch << std::endl;
        out << "  ransac_iters     = " << params.ransac_iters << std::endl;
        out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
        out << "  motion_threshold = " << params.motion_threshold << std::endl;
        return out;
    }

} // namespace viso2_stereo
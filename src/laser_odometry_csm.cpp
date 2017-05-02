#include <pluginlib/class_list_macros.h>

#include <laser_odometry_core/laser_odometry_utils.h>
#include <laser_odometry_csm/laser_odometry_csm.h>

#include <boost/assign/list_of.hpp>

namespace laser_odometry {

OdomType LaserOdometryCsm::odomType() const noexcept
{
  return OdomType::Odom3DCov;
}

bool LaserOdometryCsm::configureImpl()
{
  params_ptr_ = std::make_shared<Parameters>(private_nh_);
  params_ptr_->fromParamServer();

  kf_dist_angular_ = params_ptr_->kf_dist_angular;
  kf_dist_linear_  = params_ptr_->kf_dist_linear;

  kf_dist_linear_sq_ = kf_dist_linear_*kf_dist_linear_;

  input_.max_angular_correction_deg   = params_ptr_->max_angular_correction_deg;
  input_.max_linear_correction        = params_ptr_->max_linear_correction;
  input_.max_iterations               = params_ptr_->max_iterations;
  input_.epsilon_xy                   = params_ptr_->epsilon_xy;
  input_.epsilon_theta                = params_ptr_->epsilon_theta;
  input_.max_correspondence_dist      = params_ptr_->max_correspondence_dist;
  input_.sigma                        = params_ptr_->sigma;
  input_.use_corr_tricks              = params_ptr_->use_corr_tricks;
  input_.restart                      = params_ptr_->restart;
  input_.restart_threshold_mean_error = params_ptr_->restart_threshold_mean_error;
  input_.restart_dt                   = params_ptr_->restart_dt;
  input_.restart_dtheta               = params_ptr_->restart_dtheta;
  input_.clustering_threshold         = params_ptr_->clustering_threshold;
  input_.orientation_neighbourhood    = params_ptr_->orientation_neighbourhood;
  input_.use_point_to_line_distance   = params_ptr_->use_point_to_line_distance;
  input_.do_alpha_test                = params_ptr_->do_alpha_test;
  input_.do_alpha_test_thresholdDeg   = params_ptr_->do_alpha_test_thresholdDeg;
  input_.outliers_maxPerc             = params_ptr_->outliers_maxPerc;
  input_.outliers_adaptive_order      = params_ptr_->outliers_adaptive_order;
  input_.outliers_adaptive_mult       = params_ptr_->outliers_adaptive_mult;
  input_.do_visibility_test           = params_ptr_->do_visibility_test;
  input_.outliers_remove_doubles      = params_ptr_->outliers_remove_doubles;
  input_.do_compute_covariance        = params_ptr_->do_compute_covariance;
  input_.debug_verify_tricks          = params_ptr_->debug_verify_tricks;
  input_.use_ml_weights               = params_ptr_->use_ml_weights;
  input_.use_sigma_weights            = params_ptr_->use_sigma_weights;

  ROS_INFO_STREAM("LaserOdometryCsm parameters:\n" << *params_ptr_);

  world_to_base_kf_ = tf::Transform::getIdentity();

  updateLaserPose();

  output_.cov_x_m  = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  return true;
}

bool LaserOdometryCsm::process_impl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                    const tf::Transform& prediction)
{
  convert(laser_msg, current_ldp_scan_);

  prev_scan_->odometry[0] = 0.0;
  prev_scan_->odometry[1] = 0.0;
  prev_scan_->odometry[2] = 0.0;

  prev_scan_->estimate[0] = 0.0;
  prev_scan_->estimate[1] = 0.0;
  prev_scan_->estimate[2] = 0.0;

  prev_scan_->true_pose[0] = 0.0;
  prev_scan_->true_pose[1] = 0.0;
  prev_scan_->true_pose[2] = 0.0;

  input_.first_guess[0] = prediction.getOrigin().getX();
  input_.first_guess[1] = prediction.getOrigin().getY();
  input_.first_guess[2] = tf::getYaw(prediction.getRotation());

  input_.laser_ref  = prev_scan_;
  input_.laser_sens = current_ldp_scan_;

  updateLaserPose();

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using icp from CSM
  sm_icp(&input_, &output_);

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame
    utils::tfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], correction_);
  }
  else
  {
    correction_.setIdentity();
    ROS_WARN("Error in scan matching");
  }

  return output_.valid;
}

void LaserOdometryCsm::convert(const sensor_msgs::LaserScanConstPtr& scan_msg,
                               LDP& ldp_scan)
{
  unsigned int n = scan_msg->ranges.size();
  ldp_scan = ld_alloc_new(n);

  if (n != theta_.size()) cache(scan_msg);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame
    double r = scan_msg->ranges[i];

    if (r > input_.min_reading && r < input_.max_reading)
    {
      ldp_scan->valid[i]    = 1;
      ldp_scan->readings[i] = r;
    }
    else
    {
      ldp_scan->valid[i]    =  0;
      ldp_scan->readings[i] = -1;  // for invalid range
    }

    ldp_scan->theta[i]    = theta_[i];
    ldp_scan->cluster[i]  = -1;
  }

  ldp_scan->min_theta = *theta_.begin();
  ldp_scan->max_theta = theta_.back();

  ldp_scan->odometry[0] = 0.0;
  ldp_scan->odometry[1] = 0.0;
  ldp_scan->odometry[2] = 0.0;

  ldp_scan->true_pose[0] = 0.0;
  ldp_scan->true_pose[1] = 0.0;
  ldp_scan->true_pose[2] = 0.0;
}

void LaserOdometryCsm::cache(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  const unsigned int n = scan_msg->ranges.size();

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;

  theta_.resize(n);

  for (unsigned int i = 0; i < n; i++)
    theta_[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
}

bool LaserOdometryCsm::initialize(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  //a_cos_.clear(); a_cos_.reserve(scan_msg.ranges.size());
  //a_sin_.clear(); a_sin_.reserve(scan_msg.ranges.size());

  //for (unsigned int i = 0; i < scan_msg.ranges.size(); ++i)
  //{
    //double angle = scan_msg.angle_min + i * scan_msg.angle_increment;
    //a_cos_.push_back(cos(angle));
    //a_sin_.push_back(sin(angle));
  //}

  cache(scan_msg);

  convert(scan_msg, prev_scan_);

  return true;
}

bool LaserOdometryCsm::isKeyFrame(const tf::Transform& tf)
{
  if (fabs(tf::getYaw(tf.getRotation())) > kf_dist_angular_) return true;

  double x = tf.getOrigin().getX();
  double y = tf.getOrigin().getY();

  if (x*x + y*y > kf_dist_linear_sq_) return true;

  return false;
}

void LaserOdometryCsm::isKeyFrame()
{
  ld_free(prev_scan_);
  prev_scan_ = current_ldp_scan_;
}

void LaserOdometryCsm::isNotKeyFrame()
{
  ld_free(current_ldp_scan_);
}

void LaserOdometryCsm::updateLaserPose()
{
  input_.laser[0] = base_to_laser_.getOrigin().getX();
  input_.laser[1] = base_to_laser_.getOrigin().getY();
  input_.laser[2] = tf::getYaw(base_to_laser_.getRotation());

  //  input_.laser[0] = laser_to_base_.getOrigin().getX();
  //  input_.laser[1] = laser_to_base_.getOrigin().getY();
  //  input_.laser[2] = tf::getYaw(laser_to_base_.getRotation());
}

} /* namespace laser_odometry */

PLUGINLIB_EXPORT_CLASS(laser_odometry::LaserOdometryCsm, laser_odometry::LaserOdometryBase);

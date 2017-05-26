#ifndef _LASER_ODOMETRY_CSM_LASER_ODOMETRY_CSM_H_
#define _LASER_ODOMETRY_CSM_LASER_ODOMETRY_CSM_H_

// @todo rely on csm rather than laser_scan_matcher pkg
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max

#include <laser_odometry_core/laser_odometry_core.h>

#include <laser_odometry_csm/LaserOdometryCsmParameters.h>

namespace laser_odometry
{

  class LaserOdometryCsm : public LaserOdometryBase
  {
    using Base = LaserOdometryBase;

    using Parameters = laser_odometry_csm::LaserOdometryCsmParameters;
    using ParametersPtr = std::shared_ptr<Parameters>;

  public:

    LaserOdometryCsm()  = default;
    ~LaserOdometryCsm() = default;

    OdomType odomType() const noexcept override;

  protected:

    bool process_impl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                      const tf::Transform& prediction) override;

  protected:

    bool initialized_ = false;

    ParametersPtr params_ptr_;

    double kf_dist_angular_;
    double kf_dist_linear_;
    double kf_dist_linear_sq_;

    std::vector<double> theta_;
    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    sm_params input_;
    sm_result output_;

    LDP prev_scan_;
    LDP current_ldp_scan_;

    void convert(const sensor_msgs::LaserScanConstPtr& scan_msg,
                 LDP& ldp_scan);

    bool configureImpl() override;

    void cache(const sensor_msgs::LaserScanConstPtr& scan_msg);
    bool initialize(const sensor_msgs::LaserScanConstPtr& scan_msg) override;

    void updateLaserPose();

    bool isKeyFrame(const tf::Transform& increment) override;
    void isKeyFrame() override;
    void isNotKeyFrame() override;
  };

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_CSM_LASER_ODOMETRY_CSM_H_ */

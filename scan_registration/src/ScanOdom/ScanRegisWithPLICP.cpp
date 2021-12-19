#include "ScanRegis/ScanRegisWithPLICP.h"

using namespace std;

ScanRegisWithPLICP::ScanRegisWithPLICP()
{
    InitParams();
    m_input.laser[0] = 0.0;
    m_input.laser[1] = 0.0;
    m_input.laser[2] = 0.0;

    m_output.cov_x_m = 0;
    m_output.dx_dy1_m = 0;
    m_output.dx_dy2_m = 0; 
    scan_regis_status = Initialzing;
}

void ScanRegisWithPLICP::Init(const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg)
{
    if(scan_regis_status!=Initialzing) return;
    m_input.min_reading = cur_scan_msg->range_min;
    m_input.max_reading = cur_scan_msg->range_max;
    LDP new_ldp_ref_scan;
    LaserScanToLDP(cur_scan_msg, new_ldp_ref_scan);
    m_ldp_ref_scan = new_ldp_ref_scan;
    scan_regis_status = Initialzed; 
}

void ScanRegisWithPLICP::UpdateRefScan(const sensor_msgs::LaserScan::ConstPtr& ref_scan_msg)
{
    if(scan_regis_status!=Initialzed)return;
    LDP new_ldp_ref_scan;
    LaserScanToLDP(ref_scan_msg, new_ldp_ref_scan);
    ld_free(m_ldp_ref_scan);
    m_ldp_ref_scan = new_ldp_ref_scan;
}

bool ScanRegisWithPLICP::ScanMatch(
    const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg,
    Eigen::Vector3d& predict_motion,
    Eigen::Vector3d& real_motion )
{
    if(scan_regis_status!=Initialzed)return false;
    LDP ldp_cur_scan;
    LaserScanToLDP(cur_scan_msg, ldp_cur_scan);

    m_ldp_ref_scan->odometry[0] = 0.0;
    m_ldp_ref_scan->odometry[1] = 0.0;
    m_ldp_ref_scan->odometry[2] = 0.0;

    m_ldp_ref_scan->estimate[0] = 0.0;
    m_ldp_ref_scan->estimate[1] = 0.0;
    m_ldp_ref_scan->estimate[2] = 0.0;

    m_ldp_ref_scan->true_pose[0] = 0.0;
    m_ldp_ref_scan->true_pose[1] = 0.0;
    m_ldp_ref_scan->true_pose[2] = 0.0;

    m_input.laser_ref = m_ldp_ref_scan;
    m_input.laser_sens = ldp_cur_scan;

    m_input.first_guess[0] = predict_motion[0];
    m_input.first_guess[1] = predict_motion[1];
    m_input.first_guess[2] = predict_motion[2];
    
    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    if (m_output.cov_x_m)
    {
        gsl_matrix_free(m_output.cov_x_m);
        m_output.cov_x_m = 0;
    }
    if (m_output.dx_dy1_m)
    {
        gsl_matrix_free(m_output.dx_dy1_m);
        m_output.dx_dy1_m = 0;
    }
    if (m_output.dx_dy2_m)
    {
        gsl_matrix_free(m_output.dx_dy2_m);
        m_output.dx_dy2_m = 0;
    }

    // 调用csm进行plicp计算
    sm_icp(&m_input, &m_output);
    ld_free(ldp_cur_scan);

    if(m_output.valid)
    {
        real_motion[0] = m_output.x[0];
        real_motion[1] = m_output.x[1];
        real_motion[2] = m_output.x[2];
        return true;
    }
    else 
    {
        return false;     
    }
} 


void ScanRegisWithPLICP::InitParams()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_("~");

    // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

    // Maximum angular displacement between scans
    if (!private_node_.getParam("max_angular_correction_deg", m_input.max_angular_correction_deg))
        m_input.max_angular_correction_deg = 45.0;

    // Maximum translation between scans (m)
    if (!private_node_.getParam("max_linear_correction", m_input.max_linear_correction))
        m_input.max_linear_correction = 1.0;

    // Maximum ICP cycle iterations
    if (!private_node_.getParam("max_iterations", m_input.max_iterations))
        m_input.max_iterations = 10;

    // A threshold for stopping (m)
    if (!private_node_.getParam("epsilon_xy", m_input.epsilon_xy))
        m_input.epsilon_xy = 0.000001;

    // A threshold for stopping (rad)
    if (!private_node_.getParam("epsilon_theta", m_input.epsilon_theta))
        m_input.epsilon_theta = 0.000001;

    // Maximum distance for a correspondence to be valid
    if (!private_node_.getParam("max_correspondence_dist", m_input.max_correspondence_dist))
        m_input.max_correspondence_dist = 1.0;

    // Noise in the scan (m)
    if (!private_node_.getParam("sigma", m_input.sigma))
        m_input.sigma = 0.010;

    // Use smart tricks for finding correspondences.
    if (!private_node_.getParam("use_corr_tricks", m_input.use_corr_tricks))
        m_input.use_corr_tricks = 1;

    // Restart: Restart if error is over threshold
    if (!private_node_.getParam("restart", m_input.restart))
        m_input.restart = 0;

    // Restart: Threshold for restarting
    if (!private_node_.getParam("restart_threshold_mean_error", m_input.restart_threshold_mean_error))
        m_input.restart_threshold_mean_error = 0.01;

    // Restart: displacement for restarting. (m)
    if (!private_node_.getParam("restart_dt", m_input.restart_dt))
        m_input.restart_dt = 1.0;

    // Restart: displacement for restarting. (rad)
    if (!private_node_.getParam("restart_dtheta", m_input.restart_dtheta))
        m_input.restart_dtheta = 0.1;

    // Max distance for staying in the same clustering
    if (!private_node_.getParam("clustering_threshold", m_input.clustering_threshold))
        m_input.clustering_threshold = 0.25;

    // Number of neighbour rays used to estimate the orientation
    if (!private_node_.getParam("orientation_neighbourhood", m_input.orientation_neighbourhood))
        m_input.orientation_neighbourhood = 20;

    // If 0, it's vanilla ICP
    if (!private_node_.getParam("use_point_to_line_distance", m_input.use_point_to_line_distance))
        m_input.use_point_to_line_distance = 1;

    // Discard correspondences based on the angles
    if (!private_node_.getParam("do_alpha_test", m_input.do_alpha_test))
        m_input.do_alpha_test = 0;

    // Discard correspondences based on the angles - threshold angle, in degrees
    if (!private_node_.getParam("do_alpha_test_thresholdDeg", m_input.do_alpha_test_thresholdDeg))
        m_input.do_alpha_test_thresholdDeg = 20.0;

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    if (!private_node_.getParam("outliers_maxPerc", m_input.outliers_maxPerc))
        m_input.outliers_maxPerc = 0.90;

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    if (!private_node_.getParam("outliers_adaptive_order", m_input.outliers_adaptive_order))
        m_input.outliers_adaptive_order = 0.7;

    if (!private_node_.getParam("outliers_adaptive_mult", m_input.outliers_adaptive_mult))
        m_input.outliers_adaptive_mult = 2.0;

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    if (!private_node_.getParam("do_visibility_test", m_input.do_visibility_test))
        m_input.do_visibility_test = 0;

    // no two points in laser_sens can have the same corr.
    if (!private_node_.getParam("outliers_remove_doubles", m_input.outliers_remove_doubles))
        m_input.outliers_remove_doubles = 1;

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    if (!private_node_.getParam("do_compute_covariance", m_input.do_compute_covariance))
        m_input.do_compute_covariance = 0;

    // Checks that find_correspondences_tricks gives the right answer
    if (!private_node_.getParam("debug_verify_tricks", m_input.debug_verify_tricks))
        m_input.debug_verify_tricks = 0;

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
    if (!private_node_.getParam("use_ml_weights", m_input.use_ml_weights))
        m_input.use_ml_weights = 0;

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    if (!private_node_.getParam("use_sigma_weights", m_input.use_sigma_weights))
        m_input.use_sigma_weights = 0;
}

void ScanRegisWithPLICP::LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp)
{
    unsigned int n = scan_msg->ranges.size();
    ldp = ld_alloc_new(n);

    for (unsigned int i = 0; i < n; i++)
    {
        // calculate position in laser frame
        double r = scan_msg->ranges[i];

        if (r > scan_msg->range_min && r < scan_msg->range_max)
        {
            // fill in laser scan data

            ldp->valid[i] = 1;
            ldp->readings[i] = r;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1; // for invalid range
        }

        ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
        ldp->cluster[i] = -1;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}
#include <nlohmann/json.hpp>
#include <mutex>
#include <condition_variable>
#include "so3_math.hpp"
#include "structure.hpp"

// typedef<typename Status, int dim_of_status, int dim_of_mea>
class KF {
    // Usage:
    // 1. init;
    // 2. forward... forward
    // 3. if get lidar, measurament update.
public:
    std::mutex mtx_kf;
    std::condition_variable cv_kf;
    
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Matrix<double, 12, 12> imu_bias_cov_Q;

    StatusManifold last_status;
    StatusManifold nomial_status;
    StatusTangent error_state;

    Eigen::Matrix<double, 18, 18> status_err_cov;
    // the ns timestamp of current prior status / nomial status
    // actually we the one of above once, never use them altogether.
    uint64_t timestamp = 0;
    bool ready = false;  // invoke init function to set this true;
    int ready_cnt = 0;
    int ready_cnt_overflow_sup;
private:
    Eigen::Matrix<double, 18, 18> F;
    Eigen::Matrix<double, 18, 12> FQ;
    Eigen::MatrixXd R;  // diagonal.
    M3D lidar_meas_cov_m3d;
    double lidar_meas_var;
    Eigen::VectorXd z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd K;
    
    
public:
    KF(const nlohmann::json kf_cfg) {
        Eigen::Vector<double, 12> imu_bias;
        imu_bias.block(0, 0, 3, 1) = V3D::Constant(kf_cfg["imu_nw"]);
        imu_bias.block(3, 0, 3, 1) = V3D::Constant(kf_cfg["imu_na"]);
        imu_bias.block(6, 0, 3, 1) = V3D::Constant(kf_cfg["imu_nbw"]);
        imu_bias.block(9, 0, 3, 1) = V3D::Constant(kf_cfg["imu_nba"]);        
        
        imu_bias_cov_Q = imu_bias.asDiagonal();
        lidar_meas_cov_m3d = V3D::Constant(kf_cfg["meas_xyz"]).asDiagonal();
        // status_err_cov = Eigen::Matrix<double, 18, 18>::Zero();
        status_err_cov = Eigen::Matrix<double, 18, 18>::Identity() * 1; 

        ready_cnt_overflow_sup = kf_cfg["init_imu_msgs"];

        F.setZero();
        FQ.setZero();
        // H.setZero();
        
    }
    
    void init(const IMUMsg& msg) {
        timestamp = msg.timestamp;
        ready_cnt += 1;
        nomial_status.g = nomial_status.g + msg.linear_acceleration;
        nomial_status.bw = nomial_status.bw + msg.angular_velocity;
        // std::cout << msg.linear_acceleration.norm();
        // std::cout << nomial_status.g.norm();
        // std::cout << std::endl;
        if (ready_cnt == ready_cnt_overflow_sup) {
            nomial_status.bw = nomial_status.bw / ready_cnt;
            V3D _g = nomial_status.g / ready_cnt;
            V3D std_g(0, 0, 1);
            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(_g.normalized(), std_g);
            nomial_status.R = q.toRotationMatrix();
            nomial_status.g = V3D(0, 0, -9.81);

            // nomial_status.ba = _g + nomial_status.R.transpose() * nomial_status.g;
            
            ready = true;
        }
    }

    void forward_propagation(const IMUMsg& msg) {
        if (timestamp == 0) {
            throw std::runtime_error("init KF first.");
        }
        if (msg.timestamp <= timestamp) {
            throw std::runtime_error("imu msg earlier than current frame!");
        }
        
        double dt = (double)(msg.timestamp - timestamp) / 1e9;
        timestamp = msg.timestamp;
        StatusTangent  increment;
        increment.dtheta = dt * (msg.angular_velocity - nomial_status.bw);
        increment.dp = dt * nomial_status.v;
        // \delta v = R_{ego2glob} (a - ba) * dt - gravity * dt
        increment.dv = nomial_status.R * (msg.linear_acceleration - nomial_status.ba) * dt + nomial_status.g * dt;
        // \hat{x} = f(\hat{x_{k-1}}, mea) + F\delta \hat{x_{k-1}} + F_Q Q;
        // thus \delta x = F \delta x_{k-1} + J q, for BCH formula, addition on lie algebra.
        std::unique_lock<std::mutex> lock(mtx_kf);
        last_status = nomial_status;
        nomial_status = boxplus(nomial_status, increment);  // note that the signature of boxplus
        lock.unlock();
        cv_kf.notify_all();

        // covariance update
        calc_Fx(increment, dt, msg.linear_acceleration - last_status.ba);
        calc_FQ(increment, dt);
        // P- = F P FT + F_Q Q F_Q T
        status_err_cov = F * status_err_cov * F.transpose()
                         + FQ * imu_bias_cov_Q * FQ.transpose();  // Cov = E[xx^T]
        
    }

    
    void measurament_update(std::vector<PlanarFeature>& planar_buf,
            std::vector<EdgeFeature>& edge_buf) {
        // solve MAP
        // planar_buf.clear();
        calc_H_R(nomial_status.R, planar_buf, edge_buf);
        if (H.rows() == 0) {
            return;
        }
        calc_K_status_dim_inv();
        Eigen::Vector<double, 18> e = K * z;
        // std::cout << z.norm() << std::endl;
        error_state = StatusTangent(e);

        std::unique_lock<std::mutex> lock(mtx_kf);
        nomial_status = boxplus(nomial_status, error_state);
        
        lock.unlock();
        cv_kf.notify_all();
        
        // posterior covariance update.
        status_err_cov = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * status_err_cov;

        // // adjoint update
        Eigen::Matrix<double, 18, 18> m;
        m.setIdentity();
        m.block(0, 0, 3, 3) = SO3_right_jacobian(error_state.dtheta);
        status_err_cov = m * status_err_cov * m.transpose();
        // // finish one solving.
    }

    StatusManifold get_status() {
        std::lock_guard<std::mutex> lock(mtx_kf);
        return nomial_status;
    }
    
    bool is_ready() {return ready;}
private:   
    void calc_Fx(const StatusTangent& t, const double dt, const V3D& acc) {
        M3D _R = Exp(t.dtheta);
        M3D J_r = SO3_right_jacobian(t.dtheta);
        F.block(0, 0, 3, 3) = _R.transpose();
        F.block(0, 9, 3, 3) = -1 * J_r * dt;
        F.block(3, 3, 3, 3).setIdentity();
        F.block(3, 6, 3, 3) = M3D::Identity() * dt;

        F.block(6, 0, 3, 3) = -1 * last_status.R * hat(acc) * dt;
        F.block(6, 12, 3, 3) = -1 * last_status.R * dt;
        F.block(6, 15, 3, 3) = M3D::Identity() * dt;

        F.block(9, 9, 9, 9).setIdentity();
        
    }
    
    Eigen::Matrix<double, 18, 12> calc_FQ(const StatusTangent& t, const double dt) {
        M3D J_r = SO3_right_jacobian(t.dtheta);
        FQ.block(0, 0, 3, 3) = J_r * dt;
        FQ.block(6, 3, 3, 3) = last_status.R * dt;
        FQ.block(9, 6, 3, 3) = M3D::Identity() * dt;
        FQ.block(12, 9, 3, 3) = M3D::Identity() * dt; 
        return FQ;
    }

    void calc_H_R(M3D& rotation,
             std::vector<PlanarFeature>& planar_buf,
            std::vector<EdgeFeature>& edge_buf) {
        
        H.resize(planar_buf.size() + edge_buf.size() * 3, 18);
        z.resize(planar_buf.size() + edge_buf.size() * 3);
        R.resize(planar_buf.size() + edge_buf.size() * 3, planar_buf.size() + edge_buf.size() * 3);
        
        H.setZero();
        z.setZero();
        R.setZero();

        auto&& it_planar = planar_buf.begin();
        for (size_t i = 0; i < planar_buf.size(); i ++) {
            H.block(i, 0, 1, 3) = -1 * it_planar->normal.transpose() * rotation * hat(it_planar->p_loc);
            H.block(i, 3, 1, 3) = it_planar->normal.transpose();
            z(i) = it_planar->residual;
            Eigen::Matrix<double, 1, 3> T = it_planar->normal.transpose() * rotation;            
            R(i, i) = T * lidar_meas_cov_m3d * T.transpose();
            it_planar ++;
        }
        
        auto&& it_edge = edge_buf.begin();
        for (size_t i = 0; i < edge_buf.size(); i++) {
            H.block(planar_buf.size() + i * 3, 0, 3, 3) = -1 * hat(it_edge->direction) * rotation * hat(it_edge->p_loc);
            H.block(planar_buf.size() + i * 3, 3, 3, 3) = hat(it_edge->direction);
            z.block(planar_buf.size() + i * 3, 0, 3, 1) = it_edge->residual;
            M3D T = hat(it_edge->direction) * rotation;
            R.block(planar_buf.size() + i * 3, planar_buf.size() + i * 3, 3, 3)
                = lidar_meas_cov_m3d;// T * lidar_meas_cov_m3d * T.transpose();
            it_edge ++;
        } // z + H \delta x == z - (-H) \delta x
    }
    
    // PH^\top(HPH^\top + R)^{-1}
    // void calc_K_measure_dim_inv() {
    //     K = status_err_cov * H.transpose() * (H * status_err_cov * H.transpose() + R).inverse();
    // }
    
    // (H^\topR^{-1}H + P^{-1})^{-1}H^\top R^{-1}
    void calc_K_status_dim_inv() {
        K = -1 * (H.transpose() * diag_inv(R) * H + status_err_cov.inverse()).inverse() * H.transpose() * diag_inv(R);
        
    }


    Eigen::MatrixXd diag_inv(Eigen::MatrixXd& m) {
        int col = m.cols();
        int row = m.rows();
        assert(col == row);

        Eigen::MatrixXd ret;
        ret.resizeLike(m);
        ret.setZero();

        for (size_t i = 0; i < col; i++) {
            ret(i, i) = 1 / (m(i, i) + 1e-8);
        }
        return ret;
    }
    // ABORTED
    template<int dim>
    Eigen::Matrix<double, dim, dim> diagonal_inverse(Eigen::Matrix<double, dim, dim>& diag) {
        Eigen::Matrix<double, dim, dim> m;
        m.setIdentity();
        for (size_t i = 0; i < dim; i++) {
            if (diag(i, i) == 0) {
                throw std::runtime_error("the diagonal matrix isn't invertable");
            }
            m(i, i) = 1 / diag(i, i);
        }
        return m;
    }
};

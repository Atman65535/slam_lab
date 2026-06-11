#include <deque>
#include <nlohmann/json.hpp>
#include <mutex>
#include <condition_variable>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "structure.hpp"
#include "so3_math.hpp"


class LiDARProcessor {
public:
    pcl::PointCloud<PointType>::Ptr global_map;
    pcl::PointCloud<PointType>::Ptr global_map_match;
    std::mutex global_map_mtx;
    std::mutex global_map_match_mtx;
    
    std::condition_variable cv_global_map_update;
private:
    double filter_angle;
    size_t window_size;

    std::atomic<bool> ready = false;
    pcl::VoxelGrid<PointType> vg;
    pcl::KdTreeFLANN<PointType> kdtree;
    
public:
    LiDARProcessor(const nlohmann::json lidar_processor_cfg) {
        filter_angle = 2 * std::numbers::pi / lidar_processor_cfg["feature_points"].get<double>();
        window_size = lidar_processor_cfg["window_size"];
        
        global_map = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        global_map_match = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }
    
    void from_ego_to_world(pcl::PointCloud<PointType>::Ptr cloud, const M3D& R, const V3D& p) {
        Eigen::Affine3f aff;
        aff.linear() = R.cast<float>();
        aff.translation() = p.cast<float>();
        pcl::transformPointCloud(*cloud, *cloud, aff);
    }
    
    void from_ego_to_world(pcl::PointCloud<PointType>::Ptr loc_cloud,
                           pcl::PointCloud<PointType>::Ptr glob_cloud,
                           const M3D& R_ego_to_world, const V3D& p) {
        Eigen::Affine3f aff;
        aff.linear() = R_ego_to_world.cast<float>();
        aff.translation() = p.cast<float>();
        pcl::transformPointCloud(*loc_cloud, *glob_cloud, aff);
    }

    V3D from_world_to_ego_v3d(const PointType& pnt, const M3D& R_ego_to_world, const V3D& p) {
        PointType p_;
        V3D org(pnt.x, pnt.y, pnt.z);
        org = org - p;
        org = R_ego_to_world.transpose() * org;
        return org;
    }
    
    void undistort(std::deque<IMUMsg>& imu_history, std::deque<StatusManifold>& status_history, LiDARMsg& msg) {
        // compensate 
        size_t idx_start;
        // find target msg / status segment
        for (idx_start = 0; idx_start < imu_history.size()-1; idx_start++) {
            if (imu_history[idx_start].timestamp <= msg.starttime &&
                imu_history[idx_start + 1].timestamp > msg.starttime) {
                break;
            }
        }
        imu_history.erase(imu_history.begin(), imu_history.begin() + idx_start);
        status_history.erase(status_history.begin(), status_history.begin() + idx_start);
        if (imu_history.size() < 5) {
            std::cout << imu_history.size() << std::endl;
            throw std::logic_error("LiDAR Processor: wrong start and end time");
        }
        assert(imu_history.size() == status_history.size());
                
        uint64_t cur_timestamp = imu_history.back().timestamp;

        auto&& it_p = msg.cloud_ptr->end();
        auto&& imu_prev = imu_history.end() - 1;
        auto&& status_prev = status_history.end() - 1;
        StatusManifold last_status = status_history.back();
        StatusManifold point_status;

        uint64_t start_time = imu_history.front().timestamp;
        double lidar_start_time = (double)(msg.starttime - start_time) / 1e9;
        
        while (true) {
            double dt = (lidar_start_time + it_p->curvature) - (double)(imu_prev->timestamp - start_time)/1e9;
            StatusTangent increment;
            increment.dtheta = dt * (imu_prev->angular_velocity - status_prev->bw);
            increment.dp = dt * status_prev->v;
            increment.dv = status_prev->R * (imu_prev->linear_acceleration - status_prev->ba) * dt + status_prev->g * dt;
            point_status = boxplus(*status_prev, increment);
            V3D p_i (it_p->x, it_p->y, it_p->z);
            p_i = last_status.R.transpose() * ((point_status.R * p_i + point_status.p) - last_status.p);
            it_p->x = p_i.x();
            it_p->y = p_i.y();
            it_p->z = p_i.z();

            if (it_p == msg.cloud_ptr->begin()) {
                break;
            }
            it_p --;
            if (msg.starttime + it_p->curvature < imu_prev->timestamp) {
                imu_prev--;
                status_prev--;
            }
        }
    }

    // this function might return too less points.
    pcl::PointCloud<PointType>::Ptr down_sampling(pcl::PointCloud<PointType>::Ptr original) {
        pcl::PointCloud<PointType>::Ptr filtered (new pcl::PointCloud<PointType>);
        bool find = false;
        V3D start_point;
        
        for (size_t i = 1; i < original->size() - 1; i ++) {
            V3D cur(original->points[i].x, original->points[i].y, original->points[i].z);
            V3D last (original->points[i-1].x, original->points[i-1].y, original->points[i-1].z);
            V3D next (original->points[i + 1].x, original->points[i+1].y, original->points[i+1].z);
            if (find == false) {
                if ((last - cur).squaredNorm() < 0.04 && (next - cur).squaredNorm() < 0.04) {
                    find = true;
                    filtered->push_back(original->points[i]);
                    start_point = cur;
                }
            }
            else {
                if (cur.normalized().transpose() * start_point.normalized() < std::cos(filter_angle)) {
                    find = false;
                }
            }
        }
        return filtered;
    }

    // the frame should project to global first.
    void knn_residual_calc(pcl::PointCloud<PointType>::Ptr frame_glob,
                           const StatusManifold& status,
                           std::vector<PlanarFeature>& planar_buf,
                           std::vector<EdgeFeature>& edge_buf) {
        
        std::lock_guard<std::mutex> lock_match(global_map_match_mtx);
        
        std::vector<float> dist_sq(5);
        std::vector<int> idx(5);
        for (size_t i = 0; i < frame_glob->size(); i ++) {
            if (kdtree.nearestKSearch(frame_glob->points[i], 5, idx, dist_sq)) {
                Eigen::Matrix<double, 3, 5> m;
                for (size_t j = 0; j < 5; j++) {
                    m.block(0, j, 3, 1) = V3D(global_map_match->points[idx[j]].x,
                                              global_map_match->points[idx[j]].y,
                                              global_map_match->points[idx[j]].z);
                }
                
                PointType mean_point;
                Eigen::SelfAdjointEigenSolver<M3D> solver = pca_feature(m, mean_point);
                double ratio = solver.eigenvalues()[2] / (solver.eigenvalues()[1] + 1e-5);
                V3D cur_world(frame_glob->points[i].x, frame_glob->points[i].y, frame_glob->points[i].z);
                
                // line feature assign
                // std::cout << ratio << " " << std::endl;
                if (ratio > 10) {
                    EdgeFeature fea;
                    fea.direction = solver.eigenvectors().col(2);
                    V3D map_point(mean_point.x, mean_point.y, mean_point.z);
                    fea.p_loc = from_world_to_ego_v3d(frame_glob->points[i], status.R, status.p);
                    fea.residual = edge_residual(fea.direction, cur_world, map_point);
                    if (fea.residual.squaredNorm() < 0.001) {
                        edge_buf.push_back(fea);
                    }
                }
                // planar
                // else if (ratio < 12) {
                //     PlanarFeature fea;
                //     fea.normal = solver.eigenvectors().col(0);
                //     fea.p_loc = from_world_to_ego_v3d(frame_glob->points[i], status.R, status.p);
                //     fea.residual = planar_residual(fea.normal, cur_world, V3D(mean_point.x, mean_point.y, mean_point.z));
                //     if (fea.residual < 0.1) {
                //         planar_buf.push_back(fea);
                //     }
                // }
            }
            
        }
    }

    void init(pcl::PointCloud<PointType>::Ptr cloud) {
        if (ready) {
            return;
        }
        
        window_size --;
        
        if (window_size == 0) {
            update_global_map(cloud);
            ready = true;
            return;
        }
        *global_map += *cloud;
    }
    
    bool is_ready() {
        return ready;
    }

    void update_global_map(pcl::PointCloud<PointType>::Ptr new_cloud) {
        std::unique_lock<std::mutex> global_map_lock(global_map_mtx);
        *global_map += *new_cloud;
        vg.setInputCloud(global_map);
        vg.setLeafSize(0.02, 0.02, 0.02);

        std::unique_lock<std::mutex> match_map_lock(global_map_match_mtx);
        vg.filter(*global_map_match);
        kdtree.setInputCloud(global_map_match);
        match_map_lock.unlock();
        
        vg.filter(*global_map);
        global_map_lock.unlock();
        cv_global_map_update.notify_all();
    }
    
private:
    // increasing order solving. sq sigma.
    // Get eigen vector, eigenvalue ** 2 and center point altogether.
    Eigen::SelfAdjointEigenSolver<M3D> pca_feature(Eigen::Matrix<double, 3, 5>& m, PointType& p_mean) {
        V3D mean = m.rowwise().mean();
        m = m.colwise() - mean;
        M3D res = m * m.transpose() / m.cols();
        Eigen::SelfAdjointEigenSolver<M3D> sol(res);
        
        p_mean.x = mean.x();
        p_mean.y = mean.y();
        p_mean.z = mean.z();
        
        return sol;
    }
    
    void global_filtering() {
        vg.setInputCloud(global_map);
        vg.setLeafSize(0.02, 0.02, 0.02);
        vg.filter(*global_map);
    }

    V3D edge_residual(const V3D& edge, const V3D& p_loc, const V3D& center) {
        return hat(edge) * ( p_loc - center);
    }

    double planar_residual(const V3D& normal, const V3D& p_loc, const V3D center) {
        return normal.transpose() * (p_loc - center);
    }
};


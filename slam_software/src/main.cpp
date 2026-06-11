#include <fstream>
#include <nlohmann/json.hpp>

#include "dataloader.hpp"
#include "structure.hpp"
#include "kf.hpp"  
#include "so3_math.hpp"
#include "view.hpp"
#include "lidar_processor.hpp"
#include <chrono>

using namespace std::chrono_literals;

bool imu_msg_ready = false;
bool lidar_msg_ready = false;
IMUMsg imu_msg;
LiDARMsg lidar_msg;

// The imu msgs are send in this buf when sync
// While consumed after distortion and kf init;
std::deque<IMUMsg> imu_msgs;
std::deque<StatusManifold> status_history;


nlohmann::json parse_cfg(const std::string& file_name);

// Design : this func finish the synchronizing between the two sensors
// Process IMU first, and process lidar only when the imu frames inside
// the scan are processd.
// WARNING Make sure the IMU integration is faster than the frequency of sensor.
bool main_data_synchronizing(DataLoader& loader) {
    bool ret = false;
    std::unique_lock<std::mutex> lock(loader.mtx);
    loader.cv.wait(lock, [&loader]{return !loader.lidar_q.empty() || !loader.imu_q.empty();});
    if (!loader.imu_q.empty() && imu_msg_ready == false) {
       imu_msg = loader.imu_q.front();
       loader.imu_q.pop_front();
       imu_msg_ready = true;
       ret = true;
    }
    if (!loader.lidar_q.empty() && lidar_msg_ready == false) {
        lidar_msg = loader.lidar_q.front();
        loader.lidar_q.pop_front();
        lidar_msg_ready = true;
        ret = true;
        // std::cout << lidar_msg.cloud_ptr->size() << std::endl;
    }
    lock.unlock();
    return ret;
}

void kf_update_task(DataLoader& loader, KF& kf, LiDARProcessor& lidar_handler) {
    
    pcl::PointCloud<PointType>::Ptr cur_loc_full_cloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cur_loc_to_match (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cur_global_to_match (new pcl::PointCloud<PointType>);

    // std::vector<LiDARMsg> init_frames;
    std::vector<EdgeFeature> edge_fea;
    std::vector<PlanarFeature> planar_fea;
    while (!kf.is_ready() ) {
        if (main_data_synchronizing(loader)) {
            if (imu_msg_ready) {
                imu_msg_ready = !imu_msg_ready;
                kf.init(imu_msg);
            }
            if (lidar_msg_ready) {
                lidar_msg_ready = !lidar_msg_ready;
                // init_frames.push_back(lidar_msg);
            }
        }
    }

    while (!lidar_handler.is_ready()) {
        if (main_data_synchronizing(loader)) {
            if (imu_msg_ready) {
                imu_msg_ready = false;
            }
            if (lidar_msg_ready) {
                lidar_msg_ready = false;
                auto status = kf.get_status();
                auto cloud = lidar_msg.cloud_ptr;
                lidar_handler.from_ego_to_world(cloud, status.R, status.p);
                lidar_handler.init(cloud);
            }
        }
    }
    
    while (true) {
        if (main_data_synchronizing(loader)) {
            // IMU propagation
            if (imu_msg_ready) {
                imu_msg_ready = !imu_msg_ready;
                imu_msgs.push_back(imu_msg);
                
                kf.forward_propagation(imu_msg);
                status_history.push_back(kf.get_status());
            }
            // lidar correction.
            if (lidar_msg_ready) {
                if (imu_msgs.empty() || imu_msgs.front().timestamp > lidar_msg.starttime) {
                    lidar_msg_ready = !lidar_msg_ready;
                    continue;
                }
                // OK we can calc it.
                if (imu_msgs.back().timestamp >= lidar_msg.endtime) {
                    auto status = kf.get_status();
                    
                    lidar_msg_ready = !lidar_msg_ready;
                    lidar_handler.undistort(imu_msgs, status_history, lidar_msg);
                    cur_loc_full_cloud = lidar_msg.cloud_ptr;
                    // std::cout << cur_loc_full_cloud->size() << " ";
                    // if (!lidar_handler.is_ready()) {
                    //     lidar_handler.from_ego_to_world(cur_loc_full_cloud, status.R, status.p);
                    //     lidar_handler.init(cur_loc_full_cloud);
                    //     continue;
                    // }
                    cur_loc_to_match = lidar_handler.down_sampling(cur_loc_full_cloud);
                    if (cur_loc_to_match->size() < 5) {
                        std::cout << "too less points to match! get " << cur_loc_to_match->size() << std::endl;
                        return;
                    }
                    lidar_handler.from_ego_to_world(cur_loc_to_match, cur_global_to_match, status.R, status.p);
                                        
                    edge_fea.clear();
                    planar_fea.clear();
                    
                    lidar_handler.knn_residual_calc(cur_global_to_match, status, planar_fea, edge_fea);
                    
                    kf.measurament_update(planar_fea, edge_fea);
                    status = kf.get_status();
                    pcl::PointCloud<PointType>::Ptr glob_temp(new pcl::PointCloud<PointType>);
                    lidar_handler.from_ego_to_world(cur_loc_full_cloud, glob_temp, status.R, status.p);
                    // lidar_handler.update_global_map(glob_temp);
                    // std::cout << "call lidar" << std::endl;
                }
            }
        }
    }
}

int main() {
    nlohmann::json cfg = parse_cfg("/home/atman/a_workspace/slam_software/config/config.json");
    
    DataLoader loader(cfg["loader_cfg"]);
    KF kf(cfg["kf_cfg"]);
    LiDARProcessor lidar_handler(cfg["lidar_processor_cfg"]);
    Viewer vis = Viewer();

    std::thread data_loading(&DataLoader::start_receiving, std::ref(loader), true, true);
    std::thread kf_update(kf_update_task, std::ref(loader), std::ref(kf), std::ref(lidar_handler));
    // initialize kf, make sure the machine is static now
    // std::deque<LiDARMsg> init_frames;
    while (true) {
        // std::this_thread::sleep_for(100ms);
        auto status = kf.get_status();
        // std::unique_lock<std::mutex> lock(lidar_handler.global_map_mtx);
        // lidar_handler.cv_global_map_update.wait(lock);
        vis.update_posture(status.R, status.p);
        vis.update_point_cloud(lidar_handler.global_map);
        // lock.unlock();
        vis.spin_once(1);
        
    }
    
    return 0;
}




nlohmann::json parse_cfg(const std::string& file_name) {
    std::ifstream file_in(file_name);
    return nlohmann::json::parse(file_in);
}

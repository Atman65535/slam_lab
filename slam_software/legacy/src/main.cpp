#include <bits/chrono.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <pcl/impl/point_types.hpp>
#include <string>
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

// #define DEBUG
#include "config.hpp"
#include "dataloader.hpp"
#include "kf.hpp"

 void pcl_visualization_task(DataLoader& loader) {
     pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
         pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
     viewer->setBackgroundColor (0, 0, 0);
         viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
     viewer->addCoordinateSystem (1.0);
     viewer->initCameraParameters ();

     while (!viewer->wasStopped()) {
         point_cloud_ptr->clear();
    
         if (!loader.lidar_q.empty()) {
             auto frame = loader.lidar_q.front();
             for (auto& i : frame.points) {
                 pcl::PointXYZI p(i.x, i.y, i.z, i.reflectivity);
                 point_cloud_ptr->push_back(p);
             }
             loader.lidar_q.pop_front();
        
             viewer->removeAllPointClouds();
             viewer->addPointCloud<pcl::PointXYZI> (point_cloud_ptr, "sample cloud");
        }
        if (!loader.imu_q.empty()) {
            auto frame = loader.imu_q.front();
            printf("%u: %f, %f, %f m/s2| %f, %f, %f deg/s\n", frame.second, frame.linear_acceleration.x(), frame.linear_acceleration.y(), frame.linear_acceleration.z(), frame.angular_velocity.x(), frame.angular_velocity.y(), frame.angular_velocity.z());
            loader.imu_q.pop_front();
        }
        viewer->spinOnce(100);
        
     }
 }

bool sync_data(DataLoader& loader, Measurement& mea);
void forward();
void backward_compensation();
void residual();
void kalman_update();

int main() {
    int frame_index = 0;
    
    Measurement mea;
    ESKF kf;
    
    DataLoader loader("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", 921600);
    std::function<void()> task = [&loader]() {
        loader.receive_data(true, false);
    };
    // loader.receive_data(true, true);
    std::thread thread1(task);
    std::thread thread2(pcl_visualization_task, std::ref(loader));

    while (true) {
        if (sync_data(loader, mea)) {
            frame_index += 1;
            if (frame_index <= cfg::kf_init_frame) {
                kf.init(mea);
                continue;
            }
        }
    }
     
    thread1.join();
    thread2.join();
 }

// capture one valid frame. 
bool sync_data(DataLoader& loader, Measurement& mea) {
    bool status = false;
    if (mea.imu_ok && mea.lidar_ok) {
        throw std::runtime_error("make sure set zero after using data");
    }
    
    if (!loader.lidar_q.empty() && !mea.lidar_ok) {
        mea.cloud = &loader.lidar_q.front();
        loader.lidar_q.pop_front();
        if (mea.empty) {
            mea.starttime = mea.cloud->starttime;
        }
        mea.lidar_ok = true;
    }
    // load in all imu message.
    while (!loader.imu_q.empty() && !mea.imu_ok) {
        mea.imu_deq.push_back(loader.imu_q.front());
        loader.imu_q.pop_front();
        
        if (mea.imu_deq.back().time > mea.cloud->endtime) {
            mea.imu_ok = true;
        }
    }
    
    if (mea.imu_ok && mea.lidar_ok)
        return true;
    
    return false;
}

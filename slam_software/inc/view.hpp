#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <deque>
#include <mutex>
#include <thread>

#include <structure.hpp>

class Viewer {
public:
    std::mutex vis_mtx;
private:
    pcl::PointCloud<PointType>::Ptr point_cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    int frame_idx;
    pcl::PointXYZ last_p;
    std::deque<Eigen::Affine3f> vis_buf;
public:
    Viewer(std::string name = "standard viewer") {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer(name));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0, "global");
        last_p = pcl::PointXYZ(0, 0, 0);
        point_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        viewer->addPointCloud<PointType>(point_cloud);

        // point_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }

    void update_posture(Eigen::Matrix3d& attitude, Eigen::Vector3d& translation) {
        viewer->removeCoordinateSystem("pose");
        Eigen::Affine3d a;
        a.linear() = attitude;
        a.translation() = translation;
        Eigen::Affine3f b = a.cast<float>();
        std::lock_guard<std::mutex> lock(vis_mtx);
        vis_buf.push_back(b);
        viewer->addCoordinateSystem(0.5, vis_buf.back(), "pose");
        pcl::PointXYZ cur (translation.x(), translation.y(), translation.z());
        viewer->addLine(last_p, cur, 255, 153, 104, "frame_" + std::to_string(frame_idx));
        last_p = cur;
        frame_idx += 1;
        
    }

    void update_point_cloud (pcl::PointCloud<PointType>::Ptr full_map) {
        point_cloud = full_map;
        viewer->updatePointCloud<PointType>(point_cloud, "cloud");
    }

    void spin_once(int time) {
        std::lock_guard<std::mutex> lock(vis_mtx);
        viewer->spinOnce(time);
    }
    
};

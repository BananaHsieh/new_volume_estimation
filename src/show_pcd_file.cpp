#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <iostream>
#include <cmath> 

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
int v1(0);
int v2(1);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
    static int fileIndex = 0;
    static std::vector<std::string> filenames = {
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0516_8.pcd",
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0516_16.pcd",
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0516_32.pcd",
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0516_64.pcd",
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0516_128.pcd",
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0517_256.pcd",
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0517_512.pcd",
        "/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/0517_encode.pcd"
    };

    if (event.getKeySym() == "space" && event.keyDown()) {
        if (fileIndex < filenames.size()) {
            
            PointCloudT::Ptr cloud_tgt(new PointCloudT);
            pcl::io::loadPCDFile<PointT>(filenames[fileIndex], *cloud_tgt);
            viewer->removePointCloud("after_original");
            viewer->removeText3D("info_2");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tgt_color_h(cloud_tgt, 180, 20, 20);
            viewer->addPointCloud(cloud_tgt, cloud_tgt_color_h, "after_original", v2);
            std::string info;
            if (fileIndex == 7) info = "Quantization encode";
            else info = "Quantization "+ std::to_string(static_cast<int>(std::pow(2, (fileIndex + 3))));
            viewer->addText(info, 10, 15, 16, 1.0, 1.0, 1.0, "info_2", v2);
            viewer->spinOnce();
            fileIndex = (fileIndex + 1) % filenames.size();
        }
    }
}


int main(int argc, char **argv) {
    PointCloudT::Ptr cloud_in(new PointCloudT);
    pcl::io::loadPCDFile<PointT>("/home/spie/catkin_ws/src/new_volume_estimation/resource/point_cloud/original.pcd", *cloud_in);

    pcl::visualization::PCLVisualizer viewer("Show differient residual");
    
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_ori_color_h(cloud_in, 20, 180, 20);
    viewer.addPointCloud(cloud_in, cloud_ori_color_h, "original", v1);
    viewer.addText("Original pointcloud", 10, 15, 16, 1.0, 1.0, 1.0, "info_1", v1);
    
    viewer.addText("after pointcloud", 10, 15, 16, 1.0, 1.0, 1.0, "info_2", v2);
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <iostream>
#include <sstream> 
#include <QMainWindow>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/surface/poisson.h>
#include <vtkRenderWindow.h>
#include <queue>
#include <vector>
#include <deque>
#include <optional>
#include <memory> 
#include <map>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include "qnode.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::octree::OctreePointCloudSearch<PointT> octreeSh;


namespace Ui
{
	class MainWindow;
}
struct VoxelData {
    std::vector<PointT> points; // 存儲體素內的所有點
    Eigen::Vector3f centroid;   // 體素的中心點
    size_t num_points;          // 點的數量
    int last_update_frame;      // 最後更新的帧編號

    VoxelData() : centroid(Eigen::Vector3f::Zero()), num_points(0), last_update_frame(0) {}

    void addPoint(const PointT& point, int current_frame) {
        points.push_back(point);
        num_points = points.size();
        last_update_frame = current_frame;
        updateCentroid();
    }

    void updateCentroid() {
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for (const auto& pt : points) {
            sum += Eigen::Vector3f(pt.x, pt.y, pt.z);
        }
        if (!points.empty()) {
            centroid = sum / static_cast<float>(points.size());
        }else{
            centroid = Eigen::Vector3f::Zero();
        }
    }
};

class CustomLeafContainer : public pcl::octree::OctreeContainerPointIndices {
public:
    VoxelData data;

    void addPoint(const PointT& point, int current_frame) {
        data.addPoint(point, current_frame);
    }

    VoxelData& getVoxelData() {
        return data;
    }
};

typedef pcl::octree::OctreePointCloud<PointT, CustomLeafContainer> CustomOctree;

struct BackgroundModel {
    std::deque<PointCloudT::Ptr> frames;
    int max_frames;

    // 構造函數，允許設定最大幀數
    BackgroundModel(int maxFrames = 10) : max_frames(maxFrames) {}
    
    // 添加新的點雲幀到背景模型中
    void addFrame(PointCloudT::Ptr new_frame) {
        PointCloudT::Ptr frame_copy(new PointCloudT(*new_frame));
        // 如果已達到最大幀數，則移除最舊的幀
        if (frames.size() >= max_frames) {
            frames.pop_front();
            // std::cout << "  remove old frame\n";
        }

        // 添加新的幀
        frames.push_back(frame_copy);
    }


    PointCloudT::Ptr getAccumulatedPointCloud() const{
        PointCloudT::Ptr accumulated_cloud(new PointCloudT());
        if (frames.empty()) return accumulated_cloud;

        for (auto& frame : frames) {
            *accumulated_cloud += *frame; 
            // std::cout << "  + frame\n";
        }
        // std::cout << "accumulated_cloud: " << accumulated_cloud->size() << std::endl;
        return accumulated_cloud;
    }

    bool isSceneStable(float threshold = 0.1) {
        // 此函數需要定義具體檢查穩定性的方法，這裡只是一個框架
        // 比如比較最新幀與前幾幀的差異
        return true;  // 根據實際情況返回是否穩定
    }
};

struct callback_args {
    // structure used to pass arguments to the callback function
	PointCloudT::Ptr cloud;
	PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args);
// void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args);

class MainWindow : public QMainWindow 
{
    Q_OBJECT
    public:
        MainWindow(int argc, char** argv, QWidget *parent = 0);
        ~MainWindow();

    // crop pointcloud
        void setMinMaxPointFromPointCloud(const PointCloudT::Ptr &cloud, PointT &minPt, PointT &maxPt);
        void updateCropBox(const PointCloudT::Ptr &cloud, const PointT &minPt, const PointT &maxPt, PointCloudT::Ptr &cloud_filtered);
    //find plane
        void ground_process(const PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud_filtered, bool flag);
        void ground_seg(const PointCloudT::Ptr &cloud);
    //濾波器
        void passThrough(const PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud_filtered);
    //聚類分割
        void ECE_process(const PointCloudT::Ptr &cloud);
    //變化偵測
        // void detectChanges(const PointCloudT::Ptr& baseCloud, const PointCloudT::Ptr& newCloud);
        // void updateBackgroundModel(CustomOctree &octree, const PointCloudT::Ptr &cloud, int current_frame, int max_age);
        // void updatePointCloudAndDetectChanges(PointCloudT::Ptr& cloud, int current_frame);
        // void updatePointCloudAndDetectChanges(PointCloudT::Ptr& cloud, float changeThreshold);
        // void updateBackgroundModel(PointCloudT::Ptr& cloud);
        // void detectChangesInPointCloud(PointCloudT::Ptr& cloud, float changeThreshold);
        // void processPointCloud(PointCloudT::Ptr& cloud, int current_frame);

        void processPointCloud(const PointCloudT::Ptr& cloud_cur, const PointCloudT::Ptr& cloud_pre, int current_frame);
        // void updateBackgroundModel(PointCloudT::Ptr& cloud, CustomOctree& myoctree);
        // void detectChangesInPointCloud(CustomOctree& octreeCurrent, CustomOctree& octreeBackground, float changeThreshold);
        void updateOctree(CustomOctree& octree, const PointCloudT::Ptr& cloud, int current_frame);
        void setupAndVisualizeOctree(CustomOctree& octree, float color);

    //update viewer
        void updateViewer();
        void updateViewer_plane(PointCloudT::Ptr &cloud);
        void addCubeWithDifferentColoredFaces(const PointT &minPt, const PointT &maxPt);
        void addColoredCubeFace(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2,
                                const Eigen::Vector3f& v3, const Eigen::Vector3f& v4,
                                const std::string& id, const Eigen::Vector3f& color);
        void addCubeWithLabels();
        void addPlaneCube();
        void extractPointsInsideCube(const float &zMin, const float &zMax);
       
       
    public Q_SLOTS:
        void mainFunction(PointCloudT::Ptr cloud);
    //Home page
        void roiPageClicked();
        void planePageClicked();

    //ROI page
        //maxZ
        void increaseMaxZ(); void decreaseMaxZ();
        //minZ
        void increaseMinZ(); void decreaseMinZ();
        //maxX
        void increaseMaxX(); void decreaseMaxX();
        //minX
        void increaseMinX(); void decreaseMinX();
        //maxY
        void increaseMaxY(); void decreaseMaxY();
        //minY
        void increaseMinY(); void decreaseMinY();

        //設定move step
        void on_movestepChanged();
        void loadLocalROI(); void saveLocalROI();
        void backToHomeClicked();

    //Plane page
        void onSliderValueChanged(int value);
        void backToHome2Clicked();
        void applyPlane();


    protected:
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_plane;
        // boost::shared_ptr<CustomOctree> myoctree;
        boost::shared_ptr<CustomOctree> myoctreeBackground;  // 背景模型
        boost::shared_ptr<CustomOctree> myoctreeCurrentbg;     // 當前處理的點雲模型
        
        
    private:
        Ui::MainWindow *ui; 
        QNode qnode;
        struct callback_args cb_args;
        
    //設定全局點雲
        PointCloudT::Ptr mainPointCloud, //主要點雲
                         prevPointCloud, //前一幀
                         curPointCloud, //後一幀
                         changePointCloud, //變動的點雲
                         ROIPointCloud,
                         ROI,
                         cropPointCloud, //經過裁剪的點雲，為手動尋找的ROI範圍
                         planeROI, //平面ROI擷取出來的平面
                         ground; //經過尋找平面的點雲

        BackgroundModel stabilityModel;  // 用於檢測穩定性的模型
        BackgroundModel backgroundModel; // 用於累積長期背景的模型

        BackgroundModel accumulatePointCloud;  

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPointCloud;
    //ROI邊界
        PointT minPoint, maxPoint;     
        float movestep = 0.5;    
    //一些條件變數
        int framesCount = 0;    
        //show boundingBox
        // bool showBoundingBox = false, showPlanecube = false, 
        bool groundApply = false;
        bool ROImode = false;
        
    //一些變數
        std::string ROI_path = "/home/spie/catkin_ws/src/new_volume_estimation/resource/ROI.txt";
        float zShift = 0.0;
        int accumulate_frame = 20, accumulate_count = 0; //累積的frame數
        int prevCount = 0;
        int stableFrameCount = 0;  // 記錄點雲穩定的幀數
        // pcl::PointIndices::Ptr ground_inliers;
        pcl::ModelCoefficients::Ptr ground_coefficients;
};
#endif // MAIN_WINDOW_H
#include "main_window.h"
#include "ui_mainwindow.h"

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    
    struct callback_args* data = (struct callback_args*)args;
    if (event.getPointIndex() == -1)
        return;

    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
   
    data->clicked_points_3d->points.push_back(current_point);

    // Draw clicked points in red:
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    // data->viewerPtr->removePointCloud("clicked_points");
    // data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    // data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
    
}
// int num = 0;
// void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
// {
//     struct callback_args* data = (struct callback_args*)args;
//     std::vector< int > indices;
//     if (event.getPointsIndices(indices)==-1)
//         return;

//     for (int i = 0; i < indices.size(); ++i)
//     {
//         data->clicked_points_3d->points.push_back(data->cloud->points.at(indices[i]));
//     }

//     // pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);

//     // int ss;
//     // std::string cloudName;
//     // ss << num++;
//     // cloudName = std::to_string(ss) + "_cloudName";

//     // data->viewerPtr->addPointCloud(data->clicked_points_3d, red, cloudName);
//     // data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
// }


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow), qnode(argc, argv){
    ui->setupUi(this);
    this->setWindowTitle("New Volume Estimate");
    ui->lineEditMovestep->setText("0.1");

    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_plane.reset(new pcl::visualization::PCLVisualizer("planeViewer", false));

    // myoctree.reset(new CustomOctree(0.1));
    myoctreeBackground.reset(new CustomOctree(0.1f)); // 假設解析度為0.1
    myoctreeCurrentbg.reset(new CustomOctree(0.1f));

//初始化全局點雲變數
    mainPointCloud.reset(new PointCloudT);
    cropPointCloud.reset(new PointCloudT);
    ROIPointCloud.reset(new PointCloudT);
    ROI.reset(new PointCloudT);
    planeROI.reset(new PointCloudT);
    ground.reset(new PointCloudT);
    prevPointCloud.reset(new PointCloudT);
    curPointCloud.reset(new PointCloudT);
    changePointCloud.reset(new PointCloudT);
    clusterPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // ground_inliers.reset(new pcl::PointIndices); 
    ground_coefficients.reset(new pcl::ModelCoefficients); //地面參數
    
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    //Visualizer clicked cb_function init
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.cloud = ROIPointCloud;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);

    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    // viewer->registerAreaPickingCallback(pp_callback, (void*)&cb_args);

//Connect部分
    // connect(ui->start, SIGNAL(clicked()), this, SLOT(start_clicked()));
    // connect(ui->close, SIGNAL(clicked()), this, SLOT(quit_clicked()));
    connect(ui->buttonToROIPage, &QPushButton::clicked, this, &MainWindow::roiPageClicked); //到ROI頁面
    connect(ui->buttonToPlanePage, &QPushButton::clicked, this, &MainWindow::planePageClicked); //到平面頁面
    connect(ui->buttonBackToHome, &QPushButton::clicked, this, &MainWindow::backToHomeClicked); //回主畫面
    connect(ui->buttonBackToHome2, &QPushButton::clicked, this, &MainWindow::backToHome2Clicked);//回主畫面
    // connect(ui->pause, SIGNAL(clicked()), this, SLOT(pauseButtonClicked()));
    // connect(ui->resume, SIGNAL(clicked()), this, SLOT(resumeButtonClicked()));

    connect(ui->lineEditMovestep, &QLineEdit::returnPressed, this, &MainWindow::on_movestepChanged);
    //cropBox 設定邊界按鈕連結
    connect(ui->buttonIncreaseMaxZ, &QPushButton::pressed, this, &MainWindow::increaseMaxZ); //z
    connect(ui->buttonDecreaseMaxZ, &QPushButton::pressed, this, &MainWindow::decreaseMaxZ);
    connect(ui->buttonIncreaseMinZ, &QPushButton::pressed, this, &MainWindow::increaseMinZ);
    connect(ui->buttonDecreaseMinZ, &QPushButton::pressed, this, &MainWindow::decreaseMinZ);
    connect(ui->buttonIncreaseMaxX, &QPushButton::pressed, this, &MainWindow::increaseMaxX); //x
    connect(ui->buttonDecreaseMaxX, &QPushButton::pressed, this, &MainWindow::decreaseMaxX);
    connect(ui->buttonIncreaseMinX, &QPushButton::pressed, this, &MainWindow::increaseMinX);
    connect(ui->buttonDecreaseMinX, &QPushButton::pressed, this, &MainWindow::decreaseMinX);
    connect(ui->buttonIncreaseMaxY, &QPushButton::pressed, this, &MainWindow::increaseMaxY); //y
    connect(ui->buttonDecreaseMaxY, &QPushButton::pressed, this, &MainWindow::decreaseMaxY);
    connect(ui->buttonIncreaseMinY, &QPushButton::pressed, this, &MainWindow::increaseMinY);
    connect(ui->buttonDecreaseMinY, &QPushButton::pressed, this, &MainWindow::decreaseMinY);
    connect(ui->buttonLoadLocalROI, &QPushButton::clicked, this, &MainWindow::loadLocalROI); //save roi
    connect(ui->buttonSaveLocalROI, &QPushButton::clicked, this, &MainWindow::saveLocalROI); //load roi
    connect(ui->buttonApplyPlane, &QPushButton::clicked, this, &MainWindow::applyPlane); //apply plane
    //Plane page
    

//Render部分
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    ui->qvtkWidget_plane->SetRenderWindow(viewer_plane->getRenderWindow());
    viewer_plane->setupInteractor(ui->qvtkWidget_plane->GetInteractor(), ui->qvtkWidget_plane->GetRenderWindow());
    ui->qvtkWidget_plane->update();
    

// 設置初始顯示頁面為首頁
    ui->stackedWidget->setCurrentIndex(0); // 設置為首頁
}



void MainWindow::mainFunction(PointCloudT::Ptr rawPointCloud) {
    qnode.process_flag = false;
    if (accumulate_count < accumulate_frame) {
        // std::cout << "framesCount: " << framesCount << " ,accumulate_count: " << accumulate_count << "\n";
        accumulate_count += 1;
        *mainPointCloud += *rawPointCloud; 
        qnode.process_flag = true;
        // process_finish = true;
        return;
    } else {
        if (framesCount == 0){ //設定點雲初始ROI邊界
            setMinMaxPointFromPointCloud(rawPointCloud, minPoint, maxPoint); //尋找輸入點雲的最大邊界
            std::cout<< "init maxPt.x: " << maxPoint.x << ", minPt.x" << minPoint.x << "\n";
            std::cout<< "init maxPt.y: " << maxPoint.y << ", minPt.y" << minPoint.y << "\n";
            std::cout<< "init maxPt.z: " << maxPoint.z << ", minPt.z" << minPoint.z << "\n";   
        }
        if (ROImode){
            ROIPointCloud->clear();
            ROI->clear();
            *ROIPointCloud += *mainPointCloud;
            float step = 0.5;
            for (float x = minPoint.x; x <= maxPoint.x; x += step) {
                for (float y = minPoint.y; y <= maxPoint.y; y += step) {
                    for (float z = minPoint.z; z <= maxPoint.z; z += step) {
                        PointT p;
                        p.x = x; p.y = y; p.z = z;
                        ROI->push_back(p);
                    }
                }
            }
            if (ROIPointCloud->size() > 0){
                ROImode = false;
            }
        }    
        updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
        
        if (groundApply && ui->stackedWidget->currentIndex() == 0){
            passThrough(cropPointCloud, mainPointCloud);
            // std::cout << "\rprocessing... " << std::flush;
            PointCloudT::Ptr ExgroundCloud (new PointCloudT);
            ground_process(cropPointCloud, ground, false);
            ground_process(mainPointCloud, ExgroundCloud, true);
            *curPointCloud += *ExgroundCloud;
            // *curPointCloud += *ground; 
            // if (prevCount == 0){
            //     updateOctree(*myoctreeBackground, curPointCloud, framesCount);
            //     *prevPointCloud += *curPointCloud;
            //     prevCount = 1; 
            // }else{
            //     processPointCloud(curPointCloud, prevPointCloud, framesCount);
            // }
            
        } 
        updateViewer();     
        prevPointCloud->clear();
        *prevPointCloud += *curPointCloud;
        curPointCloud->clear();      
    }
    mainPointCloud->clear();
    
    accumulate_count = 0;
    framesCount += 1;
    qnode.process_flag = true;
}


void MainWindow::setupAndVisualizeOctree(CustomOctree& octree, float color) {

    // 通过自定义遍历显示每个叶节点
    int c = 0;
    float w = 0.05;
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        Eigen::Vector3f voxel_center = it.getLeafContainer().getVoxelData().centroid;

        viewer->addCube(voxel_center[0] - w, voxel_center[0] + w,
                       voxel_center[1] - w, voxel_center[1] + w,
                       voxel_center[2] - w, voxel_center[2] + w, 1.0, 0.0, color, "voxel_" + std::to_string(c) + std::to_string(color));
        c++;
    }
}

void MainWindow::updateViewer() {
//更新前先清除所有Id
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer_plane->removeAllPointClouds();
    
    pcl::visualization::PointCloudColorHandlerCustom<PointT> main_ColorHandler(curPointCloud, 0, 255, 0); // 綠
    pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_ColorHandler(ground, 255, 255, 0); // 黃
    pcl::visualization::PointCloudColorHandlerCustom<PointT> crop_ColorHandler(cropPointCloud, 255, 255, 0); // 黃
    pcl::visualization::PointCloudColorHandlerCustom<PointT> roi_ColorHandler(ROIPointCloud, 255, 255, 0); // 黃
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cb_args.clicked_points_3d, 255, 0, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> red(ROI, 255, 0, 0);


    switch(ui->stackedWidget->currentIndex()){
        case 0:
            if (groundApply){
                ECE_process(curPointCloud);
                viewer->addPointCloud<PointT>(curPointCloud, main_ColorHandler, "mainPointCloud");
                viewer->addPointCloud<PointT>(ground, ground_ColorHandler, "ground");
                viewer->addPointCloud<PointT>(changePointCloud, "changeCloud");
            }else{
                
                viewer->addPointCloud<PointT>(cropPointCloud, crop_ColorHandler, "cropPointCloud");
            }
            break;
        case 1:
            // addCubeWithLabels();
            viewer->addPointCloud<PointT>(ROIPointCloud, roi_ColorHandler, "ROIPointCloud");
            viewer->addPointCloud(cb_args.clicked_points_3d, red, "ROI");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "ROI");

            
            break;
        case 2:
            addPlaneCube();
            viewer->addPointCloud<PointT>(cropPointCloud, crop_ColorHandler, "cropPointCloud");
            break;
        default:
            break;
    }
    /*
    if (groundApply && ui->stackedWidget->currentIndex() == 0){
        
        pcl::visualization::PointCloudColorHandlerCustom<PointT> main_ColorHandler(curPointCloud, 0, 255, 0); // 綠
        viewer->addPointCloud<PointT>(curPointCloud, main_ColorHandler, "mainPointCloud");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_ColorHandler(ground, 255, 255, 0); // 黃
        viewer->addPointCloud<PointT>(ground, ground_ColorHandler, "ground");
        // ECE_process(mainPointCloud);
        // viewer->addPointCloud(clusterPointCloud, "cluster");
        
        // pcl::visualization::PointCloudColorHandlerCustom<PointT> changeCloud_ColorHandler(changePointCloud, 255, 255, 255); // 白
        // viewer->addPointCloud<PointT>(changePointCloud, changeCloud_ColorHandler, "changeCloud");

        viewer->addPointCloud<PointT>(changePointCloud, "changeCloud");
        // setupAndVisualizeOctree(*myoctree, 0.0);
        // setupAndVisualizeOctree(*myoctreeBackground, 0.0);
        // setupAndVisualizeOctree(*myoctreeCurrentbg, 1.0);
    }else{
    //show cropBox bounding
        if (ui->stackedWidget->currentIndex() == 1) addCubeWithLabels();

    //show palne cube
        if (ui->stackedWidget->currentIndex() == 2){
            addPlaneCube();
        }
    //show cropBoxPointCloud
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cropPointCloud_ColorHandler(cropPointCloud, 255, 255, 0); // 黃
        viewer->addPointCloud<PointT>(cropPointCloud, cropPointCloud_ColorHandler, "cropPointCloud");

        // pcl::visualization::PointCloudColorHandlerCustom<PointT> main_ColorHandler(mainPointCloud, 0, 255, 0); // 綠
        // viewer->addPointCloud<PointT>(mainPointCloud, main_ColorHandler, "mainPointCloud");
    }
    */
//最後總更新
    ui->qvtkWidget->update();
}

std::vector<int> countVoxel(CustomOctree& octree) {
    
    int totalVoxels = 0;
    int ignore = 0;
    auto it = octree.leaf_begin();

    while (it != octree.leaf_end()) {
        totalVoxels++;
        auto& voxelDataCurrent = it.getLeafContainer().data;
        if (voxelDataCurrent.points.size() < 3) {
            ++it;
            ignore++;
            continue;  // 忽略點數過少的體素
        }
        ++it;
    }
    std::vector<int> countInfo = {totalVoxels-ignore, ignore};
    return countInfo;
}

bool isOctreeStable(CustomOctree& currentOctree, CustomOctree& bgOctree, float threshold) {
    pcl::octree::OctreePointCloudChangeDetector<PointT> octree(0.1f);
    std::vector<int> countInfo;
    int totalVoxels = 0;
    int ignore = 0;
    // 為背景模型建立八分樹
    octree.setInputCloud(bgOctree.getInputCloud());
    octree.addPointsFromInputCloud();

    // 切換緩衝區並為當前幀建立八分樹
    octree.switchBuffers();
    octree.setInputCloud(currentOctree.getInputCloud());
    octree.addPointsFromInputCloud();

    // 獲取新體素的點索引，這些體素代表了變化的部分
    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector, 3);

    totalVoxels += countVoxel(currentOctree)[0];
    totalVoxels += countVoxel(bgOctree)[0];
    // ignore += countVoxel(currentOctree)[1];
    // ignore += countVoxel(bgOctree)[1];
    
    // 計算新體素占總體素的比例
    float stabilityRatio = static_cast<float>(newPointIdxVector.size()) / static_cast<float>(totalVoxels);
    // std::cout << "  Stability ratio: " << stabilityRatio << " (" << newPointIdxVector.size() << "/" << totalVoxels << ")";
    // std::cout << ", threshold: " << threshold;
    // 如果變化的體素數量少於閾值，則認為是穩定的
    return stabilityRatio <= threshold || totalVoxels == 0;
}


void MainWindow::updateOctree(CustomOctree& octree, const PointCloudT::Ptr& cloud, int current_frame) {
    
    octree.deleteTree();
    octree.setResolution(0.1f);
    octree.defineBoundingBox(minPoint.x, minPoint.y, minPoint.z, maxPoint.x, maxPoint.y, maxPoint.z);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();


    // /// 遍歷所有葉節點並更新體素
    // for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
    //     std::vector<int>& pointIndices = it.getLeafContainer().getPointIndicesVector();
    //     for (int idx : pointIndices) {
    //         PointT& point = (*cloud)[idx];
    //         it.getLeafContainer().data.addPoint(point, current_frame);
    //     }
    // }

    /// 遍歷所有葉節點並更新體素
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        std::vector<int>& pointIndices = it.getLeafContainer().getPointIndicesVector();
        Eigen::Vector3f voxel_center = it.getLeafContainer().getVoxelData().centroid;  // 獲取體素中心作為鍵
        VoxelData& voxelData = it.getLeafContainer().data;
        for (int idx : pointIndices) {
            PointT& point = (*cloud)[idx];
            voxelData.addPoint(point, current_frame);
        }
    }
}

PointCloudT::Ptr detectChangesInPointCloud(CustomOctree& currentOctree, CustomOctree& bgOctree) {
    float resolution = 0.05;
    pcl::octree::OctreePointCloudChangeDetector<PointT> octree(resolution);
    // 為背景模型建立八分樹
    octree.setInputCloud(bgOctree.getInputCloud());
    octree.addPointsFromInputCloud();

    // 切換緩衝區並為當前幀建立八分樹
    octree.switchBuffers();
    octree.setInputCloud(currentOctree.getInputCloud());
    octree.addPointsFromInputCloud();

    // 獲取新體素的點索引，這些體素代表了變化的部分
    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector, 3);
    
    // 存儲變化點的點雲
    PointCloudT::Ptr changePoints(new PointCloudT);
    
    // 添加新出現的點
    for (int idx : newPointIdxVector) {
        changePoints->push_back(currentOctree.getInputCloud()->points[idx]);
    }

    // 清空八分樹以重新使用
    octree.deleteTree();
    
    // 為當前幀建立八分樹
    octree.setInputCloud(currentOctree.getInputCloud());
    octree.addPointsFromInputCloud();

    // 切換緩衝區並為背景模型建立八分樹
    octree.switchBuffers();
    octree.setInputCloud(bgOctree.getInputCloud());
    octree.addPointsFromInputCloud();

    // 獲取在當前幀消失的體素的點索引
    std::vector<int> disappearedPointIdxVector;
    octree.getPointIndicesFromNewVoxels(disappearedPointIdxVector);

    // 添加消失的點
    for (int idx : disappearedPointIdxVector) {
        changePoints->push_back(bgOctree.getInputCloud()->points[idx]);
    }

    return changePoints;
}

void MainWindow::processPointCloud(const PointCloudT::Ptr& cloud_cur, const PointCloudT::Ptr& cloud_pre, int current_frame) {
    std::cout << "Processing frame: " << current_frame << "\n";
    PointCloudT::Ptr currentFrame (new PointCloudT);
    PointCloudT::Ptr bgFrame (new PointCloudT);
    // *currentFrame += *cloud;
    stabilityModel.addFrame(cloud_pre);
    currentFrame = stabilityModel.getAccumulatedPointCloud();

    updateOctree(*myoctreeCurrentbg, currentFrame, current_frame);

    CustomOctree::Ptr myoctreeCurrent(new CustomOctree(0.1f)); //輸入當前點雲模型
    updateOctree(*myoctreeCurrent, cloud_cur, current_frame);

    if (isOctreeStable(*myoctreeCurrent, *myoctreeCurrentbg, 0.3)) {
        std::cout << "  Octree is stable." << std::endl;
        if (stableFrameCount >= 7) {  // 點雲穩定5幀後執行
            changePointCloud = detectChangesInPointCloud(*myoctreeCurrent, *myoctreeBackground);

            std::cout << "  DetectChanges" << std::endl;
            backgroundModel.addFrame(cloud_cur);
            bgFrame = backgroundModel.getAccumulatedPointCloud();
            updateOctree(*myoctreeBackground, bgFrame, current_frame);
            
            // stableFrameCount = 0;  // 重置計數器
        } else {
            stableFrameCount++;  // 穩定幀數累加
        }
    } else {
        std::cout << "  PointCloud is not stable." << std::endl;
        stableFrameCount = 0;  // 如果不穩定，重置計數器
        changePointCloud->clear();
    }
    std::cout << "\n";
}

void temp(PointCloudT::Ptr &cloud, pcl::visualization::PCLVisualizer::Ptr viewer, int color){
    // Create the octree
    float resolution = 0.05f; // Octree resolution
    pcl::octree::OctreePointCloudVoxelCentroid<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    float minZ = std::numeric_limits<float>::max();
    PointCloudT bottomCentroids;

    int c = 0;
    float w = resolution / 2;

    // Iterate over all voxel centroids
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        // Eigen::Vector3f centroid;
        PointT centroid;
        it.getLeafContainer().getCentroid(centroid);
        viewer->addCube(centroid.x - w, centroid.x + w,
                       centroid.y - w, centroid.y + w,
                       centroid.z - w, centroid.z + w, 1.0, 0.0, color, "voxel_" + std::to_string(c) + std::to_string(color));
        c++;
    }


    // std::cout << "Lowest level Z value: " << minZ << std::endl;
    // std::cout << "Number of centroids at lowest level: " << bottomCentroids.size() << std::endl;

    // for (const auto& centroid : bottomCentroids) {
    //     std::cout << "Centroid at (" << centroid.x << ", " << centroid.y << ")" << std::endl;
    // }
}







void MainWindow::ECE_process(const PointCloudT::Ptr &cloud) {
    clusterPointCloud->clear();
    std::vector<pcl::PointIndices> ece_inlier;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::EuclideanClusterExtraction<PointT> ece;
    ece.setInputCloud(cloud);
    ece.setClusterTolerance(0.05);
    ece.setMinClusterSize(100);
    ece.setMaxClusterSize(25000);
    ece.setSearchMethod(tree);
    ece.extract(ece_inlier);

    std::vector<unsigned char> color;
    for (int i_segment = 0; i_segment < ece_inlier.size(); i_segment++) {
        color.push_back(static_cast<unsigned char>(rand() % 256));
        color.push_back(static_cast<unsigned char>(rand() % 256));
        color.push_back(static_cast<unsigned char>(rand() % 256));
    }

    int cluster_id = 0;
    for (const auto& indices : ece_inlier) {
        PointCloudT::Ptr cluster(new PointCloudT);
        for (int index : indices.indices) {
            cluster->points.push_back(cloud->points[index]);
        }

        temp(cluster, viewer, cluster_id);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        // Colorful points
        for (const auto& point : cluster->points) {
            pcl::PointXYZRGB rgb_point;
            rgb_point.x = point.x;
            rgb_point.y = point.y;
            rgb_point.z = point.z;
            rgb_point.r = color[3 * cluster_id];
            rgb_point.g = color[3 * cluster_id + 1];
            rgb_point.b = color[3 * cluster_id + 2];
            clusterPointCloud->push_back(rgb_point);
        }

        std::string cluster_id_str = std::to_string(++cluster_id);  // Cluster number
        viewer->addText3D(cluster_id_str, pcl::PointXYZ(centroid[0], centroid[1], centroid[2]), 0.1, 1.0, 0.0, 0.0, "cluster_num_" + std::to_string(cluster_id));
        

        // std::vector<float> x_values, y_values, z_values;
        // for (const auto& point : cluster->points) {
        //     x_values.push_back(point.x);
        //     y_values.push_back(point.y);
        //     z_values.push_back(point.z);
        // }
        // auto getPercentile = [&](std::vector<float>& values, int per) {
        //     std::nth_element(values.begin(), values.begin() + values.size() * per / 100, values.end());
        //     return values[values.size() * per / 100];
        // };
        // int percentile = 2;
        // float xmin = getPercentile(x_values, percentile);
        // float xmax = getPercentile(x_values, 100 - percentile);
        // float ymin = getPercentile(y_values, percentile);
        // float ymax = getPercentile(y_values, 100 - percentile);
        // float zmin = getPercentile(z_values, percentile);
        // float zmax = getPercentile(z_values, 100 - percentile);

        // float length = xmax - xmin;
        // float width = ymax - ymin;
        // float height = zmax - zmin;
        // float volume = length * width * height;

        // // double length = clusterMax.x - clusterMin.x;
        // // double width  = clusterMax.y - clusterMin.y;
        // // double height = clusterMax.z - clusterMin.z;
        // // float volume = length * width * height;
        // printf("Cluster %d's volume: %f * %f * %f = %f m3.\n", cluster_id, length, width, height, volume);
        
    }
    viewer->addPointCloud(clusterPointCloud, "color");
    // std::cout << "\n";
}

void MainWindow::passThrough(const PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud_filtered) {
    pcl::StatisticalOutlierRemoval<PointT> sor;		//创建对象
    sor.setInputCloud(cloud);								//设置输入点云
    sor.setMeanK(30);										//设置统计时考虑查询点邻近点数
    /*
    * 设置判断是否为离群点的阈值
    * 更具体为设置标准差倍数阈值 std_mul ，点云中所有点与其邻域的距离大于 μ ±σ• std_mul
    * 则被认为是离群点，其中 μ 代表估计的平均距离， σ 代表标准差 。
    */
    sor.setStddevMulThresh(1);	//设置为1代表：如果一个点的距离超过平均距离一个标准差以上，则会被当做离群点去除
    sor.filter(*cloud_filtered);
}

void MainWindow::ground_process(const PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud_filtered, bool flag = false){ //false為提取平面 
// 提取地面點雲
//    ground.reset(new PointCloudT);
    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(cloud);
    // extract.setIndices(ground_coefficients);
    // extract.setNegative(flag);
    // extract.filter(*ground);

    cloud_filtered->clear();
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        PointT pt = cloud->points[i];
        float val = ground_coefficients->values[0] * pt.x + ground_coefficients->values[1] * pt.y + ground_coefficients->values[2] * pt.z + ground_coefficients->values[3];
        // if ((std::abs(val) < 0.02 && !flag) || (std::abs(val) >= 0.02 && flag)) {  // 根据需要调整阈值
        //     cloud_filtered->points.push_back(pt);
        // }
        if (((std::abs(val) < 0.01) && !flag) || ((std::abs(val) >= 0.01) && flag)){
            cloud_filtered->points.push_back(pt);
        }
    }

    // Create the filtering object
    // pcl::PointIndicesPtr ground_id (new pcl::PointIndices);
    // pcl::ProgressiveMorphologicalFilter<PointT> pmf;
    // pmf.setInputCloud (cloud);
    // pmf.setMaxWindowSize (20);
    // pmf.setSlope (1.0);
    // pmf.setInitialDistance (0.5);
    // pmf.setMaxDistance (0.5);
    // pmf.extract (ground_id->indices);

    // // Create the filtering object
    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud (cloud);
    // extract.setIndices (ground_id);
    // extract.setNegative(flag);
    // extract.filter (*cloud_filtered);
}

void MainWindow::ground_seg(const PointCloudT::Ptr &cloud) {
// 地面估計
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));  // 限制法向量與z軸垂直
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(1000); //預設迭代最大次數為1000
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *ground_coefficients);
}

void MainWindow::updateViewer_plane(PointCloudT::Ptr &cloud) {
    viewer_plane->removePointCloud();
    viewer_plane->addPointCloud<PointT>(cloud, "plane");
    ui->qvtkWidget_plane->update();
}

void MainWindow::onSliderValueChanged(int value) {
    zShift = static_cast<float>(value) / 100.0; // 假設滑條範圍為0-100，並轉成浮點數
    // viewer->updateShapePose("planeCube", Eigen::Affine3f(Eigen::Translation3f(0, 0, zShift)));
    // ui->qvtkWidget->update();
    // extractPointsInsideCube();
    updateViewer();
}

void MainWindow::extractPointsInsideCube(const float &zMin, const float &zMax) {
    // PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minPoint.x, minPoint.y, zMin, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxPoint.x, maxPoint.y, zMax, 1.0));
    boxFilter.setInputCloud(cropPointCloud);
    boxFilter.filter(*planeROI);
    updateViewer_plane(planeROI);
}

void MainWindow::addPlaneCube() {
    
    float zMin = minPoint.z + zShift; 
    float zMax = minPoint.z + zShift + 0.3;

    viewer->addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, zMin, zMax, 1.0, 1.0, 1.0, "planeCube");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "planeCube");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "planeCube"); // 设置颜色为绿色

    extractPointsInsideCube(zMin, zMax);
}

void MainWindow::addCubeWithLabels() {
    viewer->addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z, 1.0, 0.0, 0.0, "bounding_box");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "bounding_box");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "bounding_box");

    Eigen::Vector3f center((minPoint.x + maxPoint.x) / 2, (minPoint.y + maxPoint.y) / 2, (minPoint.z + maxPoint.z) / 2);
    Eigen::Vector3f size(maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z);

    float textSize = 0.5; // 文本大小

    viewer->addText3D("Top", pcl::PointXYZ(center[0], center[1], center[2] + size[2]/2 + 0.1),    textSize, 1, 0, 0, "text_top"); // 红色
    viewer->addText3D("Bottom", pcl::PointXYZ(center[0], center[1], center[2] - size[2]/2 - 0.1), textSize, 0, 1, 0, "text_bottom");//绿色
    viewer->addText3D("Front", pcl::PointXYZ(center[0], center[1] + size[1]/2 + 0.1, center[2]),  textSize, 0, 0, 1, "text_front");//蓝色
    viewer->addText3D("Back", pcl::PointXYZ(center[0], center[1] - size[1]/2 - 0.1, center[2]),   textSize, 1, 1, 0, "text_back");//黄色
    viewer->addText3D("Left", pcl::PointXYZ(center[0] - size[0]/2 - 0.1, center[1], center[2]),   textSize, 1, 0, 1, "text_left");//紫色
    viewer->addText3D("Right", pcl::PointXYZ(center[0] + size[0]/2 + 0.1, center[1], center[2]),  textSize, 0, 1, 1, "text_right");//青色
}

void MainWindow::addColoredCubeFace(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3, const Eigen::Vector3f& v4, const std::string& id, const Eigen::Vector3f& color) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(v1[0], v1[1], v1[2]));
    cloud->push_back(pcl::PointXYZ(v2[0], v2[1], v2[2]));
    cloud->push_back(pcl::PointXYZ(v3[0], v3[1], v3[2]));
    cloud->push_back(pcl::PointXYZ(v4[0], v4[1], v4[2]));        

    viewer->addPolygon<pcl::PointXYZ>(cloud, color[0], color[1], color[2], id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2], id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
}

void MainWindow::addCubeWithDifferentColoredFaces(const PointT &minPt, const PointT &maxPt) {
    // 計算cube立方體的8個頂點
    Eigen::Vector3f vertices[8] = {
        {minPt.x, minPt.y, minPt.z},
        {maxPt.x, minPt.y, minPt.z},
        {maxPt.x, maxPt.y, minPt.z},
        {minPt.x, maxPt.y, minPt.z},
        {minPt.x, minPt.y, maxPt.z},
        {maxPt.x, minPt.y, maxPt.z},
        {maxPt.x, maxPt.y, maxPt.z},
        {minPt.x, maxPt.y, maxPt.z}
    };
    
    //新增每個面 並給予不同顏色
    addColoredCubeFace(vertices[0], vertices[1], vertices[5], vertices[4], "face1", Eigen::Vector3f(1, 0, 0)); // 红色
    addColoredCubeFace(vertices[1], vertices[2], vertices[6], vertices[5], "face2", Eigen::Vector3f(0, 1, 0)); // 绿色
    addColoredCubeFace(vertices[2], vertices[3], vertices[7], vertices[6], "face3", Eigen::Vector3f(0, 0, 1)); // 蓝色
    addColoredCubeFace(vertices[3], vertices[0], vertices[4], vertices[7], "face4", Eigen::Vector3f(1, 1, 0)); // 黄色
    addColoredCubeFace(vertices[4], vertices[5], vertices[6], vertices[7], "face5", Eigen::Vector3f(1, 0, 1)); // 紫色
    addColoredCubeFace(vertices[0], vertices[3], vertices[2], vertices[1], "face6", Eigen::Vector3f(0, 1, 1)); // 青色
}

void MainWindow::updateCropBox(const PointCloudT::Ptr &cloud, const PointT &minPt, const PointT &maxPt, PointCloudT::Ptr &cloud_filtered) {
    if (!cloud || !cloud_filtered) {
        std::cerr << "Input or output cloud is null." << std::endl;
        return;
    }
    pcl::CropBox<PointT> cropBoxFilter;
    
    Eigen::Vector4f minVec(minPt.x, minPt.y, minPt.z, 1.0f), //cropBoxFilter minmax型態是 Eigen::Vector4f
                    maxVec(maxPt.x, maxPt.y, maxPt.z, 1.0f); 
    
    cropBoxFilter.setMin(minVec);
    cropBoxFilter.setMax(maxVec);
    cropBoxFilter.setInputCloud(cloud);
    cropBoxFilter.filter(*cloud_filtered);

}

void MainWindow::setMinMaxPointFromPointCloud(const PointCloudT::Ptr &cloud, PointT &minPt, PointT &maxPt) {
    if (cloud && !cloud->empty()) {
        pcl::getMinMax3D(*cloud, minPt, maxPt);
    }else{
        std::cout << "The point cloud is empty!" << std::endl;
    }
}


MainWindow::~MainWindow() 
{
    delete ui;
}

//================================lineEdit================================
void MainWindow::on_movestepChanged() {
    bool ok;
    double newMovestep = ui->lineEditMovestep->text().toDouble(&ok);
    if (ok){
        movestep = newMovestep;
    }else{
        QMessageBox::warning(this, "輸入錯誤", "請輸入有效的數字");
    }
}
//================================Button================================
void MainWindow::roiPageClicked() {
    ui->stackedWidget->setCurrentIndex(1); //0 is first page
    //啟動
    QObject::connect(&qnode, SIGNAL(pclUpdated_sub(PointCloudT::Ptr)), this, SLOT(mainFunction(PointCloudT::Ptr)));
    qnode.start();
    ROImode = true;
    // showBoundingBox = true;
}
void MainWindow::planePageClicked() {
    ui->stackedWidget->setCurrentIndex(2); //0 is first page
    // float minZ = minPoint.z;  // 最小 z 值
    // float maxZ = maxPoint.z;  // 最大 z 值
    int sliderRange = static_cast<int>((maxPoint.z - minPoint.z) * 100);
    
    std::cout << "sliderRange: " << sliderRange << "\n";

    ui->verticalSlider->setMinimum(0);
    ui->verticalSlider->setMaximum(sliderRange);

    // 設置滑條的初始位置在 minPoint.z
    ui->verticalSlider->setValue(minPoint.z);
    connect(ui->verticalSlider, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged); 

    // showPlanecube = true;
}
void MainWindow::backToHomeClicked() {
    ui->stackedWidget->setCurrentIndex(0); //0 is first page
    // showBoundingBox = false;
}
void MainWindow::backToHome2Clicked() {
    ui->stackedWidget->setCurrentIndex(0); //0 is first page
    // showPlanecube = false;
}

//apply plane
void MainWindow::applyPlane(){
    ground_seg(planeROI);
    groundApply = true;
}
//save & load ROI
void MainWindow::saveLocalROI() {
    QFile file("/home/spie/catkin_ws/src/new_volume_estimation/resource/ROI.txt");
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        out << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << "\n";
        out << minPoint.x << " " << minPoint.y << " " << minPoint.z << "\n";
        file.close();
    }
    
}
void MainWindow::loadLocalROI() {
    QFile file("/home/spie/catkin_ws/src/new_volume_estimation/resource/ROI.txt");
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        in >> maxPoint.x >> maxPoint.y >> maxPoint.z;
        in >> minPoint.x >> minPoint.y >> minPoint.z;
        file.close();
    }
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
}

//maxZ
void MainWindow::increaseMaxZ() {
    maxPoint.z += movestep;
    std::cout << "maxPoint.z" << maxPoint.z << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
void MainWindow::decreaseMaxZ() {
    maxPoint.z -= movestep;
    std::cout << "maxPoint.z" << maxPoint.z << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
//minZ
void MainWindow::increaseMinZ() {
    minPoint.z += movestep;
    std::cout << "minPt.z: " << minPoint.z << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
void MainWindow::decreaseMinZ() {
    minPoint.z -= movestep;
    std::cout << "minPt.z: " << minPoint.z << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}

//maxX
void MainWindow::increaseMaxX() {
    maxPoint.x += movestep;
    std::cout << "maxPoint.x: " << maxPoint.x << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
void MainWindow::decreaseMaxX() {
    maxPoint.x -= movestep;
    std::cout << "maxPoint.x: " << maxPoint.x << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
//minX
void MainWindow::increaseMinX() {
    minPoint.x += movestep;
    std::cout << "minPt.x: " << minPoint.x << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
void MainWindow::decreaseMinX() {
    minPoint.x -= movestep;
    std::cout << "minPt.x: " << minPoint.x << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}

//maxY
void MainWindow::increaseMaxY() {
    maxPoint.y += movestep;
    std::cout << "maxPoint.y: " << maxPoint.y << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
void MainWindow::decreaseMaxY() {
    maxPoint.y -= movestep;
    std::cout << "maxPoint.y: " << maxPoint.y << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
//minY
void MainWindow::increaseMinY() {
    minPoint.y += movestep;
    std::cout << "minPt.y: " << minPoint.y << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
void MainWindow::decreaseMinY() {
    minPoint.y -= movestep;
    std::cout << "minPt.y: " << minPoint.y << "\n";
    updateCropBox(mainPointCloud, minPoint, maxPoint, cropPointCloud);
    updateViewer();
}
//======================================================================
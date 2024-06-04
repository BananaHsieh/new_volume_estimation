#include "qnode.h"

using namespace std;

QNode::QNode(int argc, char** argv ) :init_argc(argc),init_argv(argv)
{
    bool flag;
    flag = init();
    process_flag = true;
    
}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"ros_subscribe");
    if (!ros::master::check() )
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    raw_point_cloud.reset(new PointCloudT);
//    point_cloud_all.reset(new PointCloudT);
    ros::NodeHandle nh;
    // /livox/lidar_3GGDJAB00101161
    // /livox/lidar_3GGDJAB00101281
    // /merge_pointcloud_test
    // pointcloud_sub = nh.subscribe("/merge_pointcloud_test_0424", 10, &QNode::pointcloud_sub_callback, this);
    raw_pointcloud0 = nh.subscribe("/livox/lidar_3GGDJAB00101161", 10, &QNode::raw_pointcloud0_callback, this);
    raw_pointcloud1 = nh.subscribe("/livox/lidar_3GGDJAB00101281", 10, &QNode::raw_pointcloud1_callback, this);
    raw_pointcloud2 = nh.subscribe("/livox/lidar_3GGDJA900100261", 10, &QNode::raw_pointcloud2_callback, this);

    start();
    return true;
}

void QNode::run()
{
    ros::Rate loop_rate(10);
    
    while ( ros::ok() )
    {
        if (process_flag) {
            ros::spinOnce();
            loop_rate.sleep();
        }        
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
} 

// void QNode::pointcloud_sub_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
// {   
// //     cout << "pointcloud_sub_callback" << endl;
//     // test_num = 0;
//     pcl::fromROSMsg(*pointcloud, *raw_point_cloud);
//     Q_EMIT pclUpdated_sub(raw_point_cloud);
// }

void QNode::merge() {
    PointCloudT point_cloud_all;
    point_cloud_all += raw_point_cloud0;
    point_cloud_all += raw_point_cloud1;
    point_cloud_all += raw_point_cloud2;
    PointCloudT::Ptr merge_output(new PointCloudT);
    merge_output = point_cloud_all.makeShared();
    Q_EMIT pclUpdated_sub(merge_output);
}
void QNode::raw_pointcloud0_callback(const sensor_msgs::PointCloud2ConstPtr& lidar0) {
    pcl::fromROSMsg(*lidar0, raw_point_cloud0);
}
void QNode::raw_pointcloud1_callback(const sensor_msgs::PointCloud2ConstPtr& lidar1) {;
    PointCloudT temp1;
    pcl::fromROSMsg(*lidar1, temp1);
    Eigen::Matrix4d  eigen_m1;
    eigen_m1 << -0.999,  0.045,  0.021,  5.596,
                -0.044, -0.999,  0.019,  0.166,
                0.022,  0.018,  1.000,  0.022,
                0.000,  0.000,  0.000,  1.000;          
    pcl::transformPointCloud (temp1, raw_point_cloud1, eigen_m1);
}
void QNode::raw_pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& lidar2) {
    PointCloudT temp2;
    pcl::fromROSMsg(*lidar2, temp2);
    Eigen::Matrix4d  eigen_m2;
    eigen_m2 << 0.101,  0.994, -0.034,  1.945,
                -0.995,  0.101, -0.003,  4.402,
                0.000,  0.034,  0.999, -0.073,
                0.000,  0.000,  0.000,  1.000;         
    pcl::transformPointCloud (temp2, raw_point_cloud2, eigen_m2);
    merge();
}


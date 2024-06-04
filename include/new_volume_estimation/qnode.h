#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// QT
#include <QThread>

using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class QNode : public QThread
{
    Q_OBJECT
    public:
        QNode(int argc, char** argv );
        virtual ~QNode();

        void pointcloud_sub_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
        void raw_pointcloud0_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
        void raw_pointcloud1_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
        void raw_pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
        void merge();
        bool init();
        void run();

        bool process_flag = true;

    Q_SIGNALS:
        void pclUpdated_sub(PointCloudT::Ptr);
        void rosShutdown();

    private:
        int init_argc;
        char** init_argv;

        ros::Subscriber pointcloud_sub; //add
        ros::Subscriber raw_pointcloud0, raw_pointcloud1, raw_pointcloud2;
        PointCloudT::Ptr raw_point_cloud;
        PointCloudT raw_point_cloud0, raw_point_cloud1, raw_point_cloud2;
        QString pointcloud_topicName;

};
#endif // QNODE_H
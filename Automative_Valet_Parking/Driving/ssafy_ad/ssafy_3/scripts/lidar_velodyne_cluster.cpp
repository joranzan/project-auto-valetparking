#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h> // Include for pcl::ExtractIndices
#include <pcl/common/centroid.h> // Include for pcl::computeCentroid
#include <pcl_conversions/pcl_conversions.h>

class SCANCluster {
public:
    SCANCluster() {
        scan_sub = nh.subscribe("/velodyne_points", 1, &SCANCluster::callback, this);
        cluster_pub = nh.advertise<geometry_msgs::PoseArray>("clusters", 1);

        // Set DBSCAN parameters
        // eps and min_samples are the parameters for DBSCAN clustering algorithm
        // You need to adjust these values based on your requirement
        dbscan.setClusterTolerance(2.0); // Set the tolerance for clustering
        dbscan.setMinClusterSize(100); // Set the minimum cluster size
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        // std::cout<<"======================================================================================";
        for(int i = 0;i<cloud->points.size();i++){
            pcl::PointXYZ point = cloud->points[i];
            double dis = sqrt(point.x * point.x + point.y * point.y);
            if(point.z<0.1 && point.z>-0.1 && dis <10) std::cout<<"x = " << point.x << " y = "<<point.y<<" z = "<<point.z<<'\n';
        }
        //std::cout<<"x = "<<msg->data.size() <<'\n';
        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            geometry_msgs::PoseArray cluster_msg;
            printf("empty\n");
            cluster_pub.publish(cluster_msg);
            return;
        }

        geometry_msgs::PoseArray cluster_msg;
        cluster_msg.header = msg->header;
        
        
        cluster_pub.publish(cluster_msg);
        */
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cluster_pub;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> dbscan;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_clustering");
    SCANCluster scan_cluster;
    ros::spin();
    return 0;
}

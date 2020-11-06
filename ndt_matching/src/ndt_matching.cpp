 /*
 Localization program using Normal Distributions Transform
linhq
 */
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

static pose previous_pose,ndt_pose;
static pcl::PointCloud<pcl::PointXYZ> map;
static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
static pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_omp;
//pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
// Default values
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon
static bool has_converged=0;
static int iteration = 0;
static float leafsize = 1;
static double fitness_score = 0.0;
static double trans_probability = 0.0;
static int threadsnum = 100;
static int methods = 1;
static ros::Publisher map_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
static int _queue_size = 1000;
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr  &scan)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ> point;
    pcl::fromROSMsg(*scan, point);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZ>(point));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //voxel_filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
    voxel_filter.setInputCloud(input_ptr);
    voxel_filter.filter(*filtered_ptr);
    int scan_points_num = filtered_ptr->size();
    std::cout<<scan_points_num;

    // Guess the initial gross estimation of the transformation
    Eigen::Translation3f init_translation(previous_pose.x, previous_pose.y, previous_pose.z);
    Eigen::AngleAxisf init_rotation_x(previous_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(previous_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(previous_pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    //ndt_matching
    // ndt.setInputSource(filtered_ptr);
    // ndt.align(*output_cloud, init_guess);
    // Eigen::Matrix4f t(Eigen::Matrix4f::Identity());

    //ndt_omp_matching
    ndt_omp.setInputSource(filtered_ptr);
    ndt_omp.align(*output_cloud, init_guess);
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());

    //recording results
    // has_converged = ndt.hasConverged();
    // iteration = ndt.getFinalNumIteration();
    // t = ndt.getFinalTransformation();
    // fitness_score = ndt.getFitnessScore();
    // trans_probability = ndt.getTransformationProbability();

    // recording omp results
    has_converged = ndt_omp.hasConverged();
    iteration = ndt_omp.getFinalNumIteration();
    t = ndt_omp.getFinalTransformation();
    fitness_score = ndt_omp.getFitnessScore();
    trans_probability = ndt_omp.getTransformationProbability();

    // Update ndt_pose
    tf::Matrix3x3 mat_l;
    mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
    ndt_pose.x = t(0, 3);
    ndt_pose.y = t(1, 3);
    ndt_pose.z = t(2, 3);
    mat_l.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);
    tf::Quaternion ndt_q;
    ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);

    // Send TF "/base_link" to "/map"
    transform.setOrigin(tf::Vector3(ndt_pose.x, ndt_pose.y, ndt_pose.z));
    transform.setRotation(ndt_q);
    br.sendTransform(tf::StampedTransform(transform, scan->header.stamp, "/map", "/PandarQT"));

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Fitness Score: " << fitness_score << std::endl;
    std::cout << "Transformation Probability: " << trans_probability << std::endl;
    std::cout << "Number of Iterations: " << iteration << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
    std::cout << "(" << ndt_pose.x << ", " << ndt_pose.y << ", " << ndt_pose.z << ", " << ndt_pose.roll
              << ", " << ndt_pose.pitch << ", " << ndt_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix: " << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    // Update previous_***
    previous_pose.x = ndt_pose.x;
    previous_pose.y = ndt_pose.y;
    previous_pose.z = ndt_pose.z;
    previous_pose.roll = ndt_pose.roll;
    previous_pose.pitch = ndt_pose.pitch;
    previous_pose.yaw = ndt_pose.yaw;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_matching");
  ros::NodeHandle nh;
  // Geting parameters
  nh.getParam("queue_size", _queue_size);
  nh.getParam("threads_num", threadsnum);
  nh.getParam("methods", methods);
  nh.getParam("leafsize", leafsize);
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "queue_size: " << _queue_size << std::endl;
  std::cout << "threadsnum : " << threadsnum  << std::endl;
  std::cout << "methods: " << methods << std::endl;
  std::cout << "leafsize: " << leafsize << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  // Updated in initialpose_callback or gnss_callback
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;
  // Publishers
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("map", 1, true);
  //read map
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yxt/catkin_ws3/src/ndt_matching/cheku.pcd", *map_ptr);
  //map ndt
  // ndt.setInputTarget(map_ptr);
  // ndt.setResolution(ndt_res);
  // ndt.setMaximumIterations(max_iter);
  // ndt.setStepSize(step_size);
  // ndt.setTransformationEpsilon(trans_eps);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  // ndt.align(*output_cloud1, Eigen::Matrix4f::Identity());

  //ndt_omp
  std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
    {"KDTREE", pclomp::KDTREE},
    {"DIRECT7", pclomp::DIRECT7},
    {"DIRECT1", pclomp::DIRECT1}
  };
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
  ndt_omp.setResolution(1.0);
  ndt_omp.setNumThreads(threadsnum/*omp_get_max_threads()*/);
  ndt_omp.setNeighborhoodSearchMethod(search_methods[methods].second);
  ndt_omp.setInputTarget(map_ptr);
  ndt_omp.setStepSize(step_size);
  ndt_omp.setMaximumIterations(max_iter);
  ndt_omp.setTransformationEpsilon(trans_eps);
  ndt_omp.align(*aligned, Eigen::Matrix4f::Identity());
  //sendmap
  sensor_msgs::PointCloud2 map1;
  std::cout<<"map_size: "<<map_ptr->points.size();
  pcl::toROSMsg(*map_ptr,map1);
  map1.header.frame_id = "map";
  map_pub.publish(map1);
  std::cout<<"map loaded"<<std::endl;
  //Subscribers
  ros::Subscriber points_sub = nh.subscribe("pandar", _queue_size, points_callback);

  ros::spin();
  return 0;
}

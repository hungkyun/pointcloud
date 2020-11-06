/*
  author: linhq
*/
#include "lidar_main.h"
using namespace Eigen;
using namespace std;
//显示对象创建
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
class Lidar_node{
public:
	Lidar_node();
	void process(const sensor_msgs::PointCloud2 &scan);
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber sub;
	ros::Publisher pub;
	ros::Publisher fpub;
};
Lidar_node::Lidar_node(){
	pub = node_handle_.advertise<sensor_msgs::PointCloud2 >("raw",10);
	fpub = node_handle_.advertise<sensor_msgs::PointCloud2 >("exract",10);
	sub = node_handle_.subscribe("pandar", 1028, &Lidar_node::process, this);
}
void aabb(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,string aabb,string p)//点云AABB包围盒
{
	pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	pcl::PointXYZI min_point_AABB,max_point_AABB;
	Eigen::Vector3f mass_center,major_vector, middle_vector, minor_vector;
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getMassCenter (mass_center);
  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
	//绘制AABB包围盒
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZI> RandomColor(cloud);//设置随机颜色
	viewer->addPointCloud<pcl::PointXYZI>(cloud, RandomColor, p);
  // viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, p);
	viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, aabb);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, aabb);
}
void clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1){
	vector<pcl::PointIndices> cluster_indices;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ece;
	ece.setInputCloud(cloud1);
	ece.setClusterTolerance(0.4);
	ece.setMinClusterSize(50);
	ece.setMaxClusterSize(20000);
	ece.setSearchMethod(tree);
	ece.extract(cluster_indices);
	pcl::ExtractIndices<pcl::PointXYZI> ext;
	ext.setInputCloud(cloud1);
	cout<<"ok"<<cluster_indices.size()<<endl;
	int i=1;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it,++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    	cloud_cluster->points.push_back (cloud1->points[*pit]);
		aabb(cloud_cluster,boost::to_string(*it),boost::to_string(i*10));
	}
	viewer->spinOnce(10);
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
}
void get_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2){ //cloud输入点云，cloud2平面内的点（这个只是用来显示，要参数再另外加）
	//创建分割对象 -- 检测平面参数
	pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr outputby(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr output1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); //存储输出的模型的系数
	pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients()); //存储输出的模型的系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //存储内dian
	pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	// pcl::PointXYZI tempsmall,tempbig,osmall,obig;
	// pcl::getMinMax3D(*cloud1,tempsmall,tempbig);
	//可选设置
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
	seg.setMethodType(pcl::SAC_RANSAC);      //设置方法【聚类或随机样本一致性】
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.08);
	seg.setInputCloud(cloud1);
	seg.segment(*inliers, *coefficients);    //分割操作
	extract.setInputCloud(cloud1);
	extract.setIndices(inliers);
	extract.filter(*output);//提取对于索引的点云 内点
	// pcl::getMinMax3D(*output,osmall,obig);
	if(/*osmall.z>tempsmall.z+0.5*/output->points.size()<250){
		extract.setNegative(true);
		extract.filter(*outputby);//提取对于索引的点云 内点
		cout<<"outputby: "<<outputby->points.size()<<endl;
		seg.setInputCloud(outputby);
		seg.setDistanceThreshold(0.1);
		seg.segment(*inliers1, *coefficients1);    //分割操作
		extract.setInputCloud(outputby);
		extract.setIndices(inliers1);
		extract.setNegative(true);
		extract.filter(*output1);//提取对于索引的点云 内点
		*output1=*output1+*output;
	}
	else{
		extract.setNegative(true);
		extract.setIndices(inliers);
		extract.filter(*output1);//提取对于索引的点云 内点
	}
	cout<<output->points.size()<<" "<<output1->points.size()<<endl;
	//平面参数
	// pcl::ModelCoefficients coeffs;
	// coeffs.values.push_back(coefficients->values[0]);
	// coeffs.values.push_back(coefficients->values[1]);
	// coeffs.values.push_back(coefficients->values[2]);
	// coeffs.values.push_back(coefficients->values[3]);
	std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
		<< coefficients->values[2] << " " << coefficients->values[3] << std::endl;
	// std::cerr << "output point size : " << output->points.size() << std::endl;
	*cloud2=*output1;

	// Eigen::Vector4f centroid;
  // pcl::compute3DCentroid(*output, centroid);
  // std::cout << "The XYZ coordinates of the centroid are: ("<< centroid[0] << ", "<< centroid[1] << ","<< centroid[2] << ")." << std::endl;
}
void Lidar_node::process(const sensor_msgs::PointCloud2 &scan){
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(scan,pcl_pc);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromPCLPointCloud2(pcl_pc,*cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr groundinput(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr groundoutput(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ModelCoefficients model;
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
			pcl::PointXYZI points=cloud->points[i];
			if(points.z<2&&fabs(points.x)<2&&(points.y<3&&points.y>-10)){
				// temp_cloud->points.push_back (points);
				if(((fabs(points.x)>0.6)||(points.y>0.75||points.y<-0.6))||(points.z>0.4||points.z<-0.29))
					temp_cloud->points.push_back(points);
			}
	}
	sor.setInputCloud(temp_cloud);
	sor.setLeafSize(0.1f,0.1f,0.1f);
	sor.filter(*groundinput);
	get_plane(groundinput,groundoutput); //这里改成了去除平面
	clustering(groundoutput);
  //publish points
	sensor_msgs::PointCloud2 raw;
	pcl::toROSMsg(*groundinput,raw);
	raw.header.frame_id = "PandarQT";
	pub.publish(raw);
	sensor_msgs::PointCloud2 extract;
	pcl::toROSMsg(*groundoutput,extract);
	extract.header.frame_id = "PandarQT";
	fpub.publish(extract);

}
int main(int argc, char **argv)
{
	srand(time(NULL));
  ros::init(argc,argv,"Lidar_node");
	Lidar_node node;
	cout<<"standby!"<<endl;
	ros::spin();
  return 0;
}

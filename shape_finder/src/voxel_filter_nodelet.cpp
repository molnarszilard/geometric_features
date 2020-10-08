#include "includes.h"
using namespace shape_finder;
class VoxelFilterNodelet
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef pcl::PointXYZ PointT;

  VoxelFilterNodelet()
      : private_nh("~")
  {
    //pub_.advertise(nh_, "voxel_cloud", 1);
    pub_.advertise(nh_, "voxel_cloud", 1);
    sub_ = nh_.subscribe("point_cloud_in", 1, &VoxelFilterNodelet::cloudCallback, this);
    config_server_.setCallback(boost::bind(&VoxelFilterNodelet::dynReconfCallback, this, _1, _2));
    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
  }
  ~VoxelFilterNodelet() {}

  void
  cloudCallback(const PointCloud::ConstPtr &cloud_in)
  {
    std::cout << std::endl;
    std::cout << "PointCloud before filtering has: " << cloud_in->size() << " data points." << std::endl;
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_in, *cloud);
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_filtered_voxel);
    std::cout << "PointCloud after filtering has: " << cloud_filtered_voxel->size() << " data points." << std::endl;
    sensor_msgs::PointCloud2 cloud_colored_sensor;
    pcl::toROSMsg(*cloud_filtered_voxel, cloud_colored_sensor);
    pub_.publish(cloud_colored_sensor);
  }

  void
  dynReconfCallback(shape_finder::voxel_filter_nodeletConfig &config, uint32_t level)
  {
    leaf_size = config.leafsize;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh;
  std::string tf_frame = "pico_zense_depth_frame";
  float leaf_size;
  ros::Subscriber sub_;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;
  dynamic_reconfigure::Server<shape_finder::voxel_filter_nodeletConfig> config_server_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shape_finder");
  VoxelFilterNodelet vf;
  ros::spin();
}
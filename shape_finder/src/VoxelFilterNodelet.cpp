#include "includes.h"

namespace shape_finder
{
  class VoxelFilterNodelet : public nodelet::Nodelet
  {
  public:
    VoxelFilterNodelet(){};
    ~VoxelFilterNodelet(){};

  private:
    virtual void onInit()
    {
      //pub_.advertise(nh_, "voxel_cloud", 1);
      pub_.advertise(nh_, "voxel_cloud", 1);
      sub_ = nh_.subscribe("point_cloud_in", 1, &VoxelFilterNodelet::cloudCallback, this);
      config_server_.setCallback(boost::bind(&VoxelFilterNodelet::dynReconfCallback, this, _1, _2));
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
    }

    void
    cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in)
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
    dynReconfCallback(shape_finder::shape_finderConfig &config, uint32_t level)
    {
      if(leaf_size!=config.leafsize) std::cout << "VOXEL "<< leaf_size << ", " << config.leafsize << std::endl;
      leaf_size = config.leafsize;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh;
    std::string tf_frame = "pico_zense_depth_frame";
    float leaf_size;
    ros::Subscriber sub_;
    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;
    dynamic_reconfigure::Server<shape_finder::shape_finderConfig> config_server_;
  };

} // namespace shape_finder
PLUGINLIB_EXPORT_CLASS(shape_finder::VoxelFilterNodelet, nodelet::Nodelet);
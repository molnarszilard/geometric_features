#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/gp3.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
// #include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/vtk_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <pcl/io/io.h>
#include <pcl_ros/publisher.h>
#include <ros/publisher.h>
#include <string>
#include <shape_finder/shape_finder_nodeConfig.h>
#include "sensor_msgs/Imu.h"
#include <math.h>
#include <visualization_msgs/Marker.h>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <boost/thread/mutex.hpp>

//#include "imu_transformer/tf2_sensor_msgs.h"

// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;
class ShapeFinderNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef pcl::PointXYZ PointT;

  ShapeFinderNode()
      : private_nh("~")
  {

    std::string cloud_topic = "objects";
    pub_.advertise(nh_, cloud_topic.c_str(), 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("imu_out", 1);
    //odom_trans.child_frame_id = "base_link";
    sub_ = nh_.subscribe("point_cloud_in", 1, &ShapeFinderNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&ShapeFinderNode::dynReconfCallback, this, _1, _2));
    sub_imu = nh_.subscribe("imu_data", 1, &ShapeFinderNode::imuCallback, this);
    ros::NodeHandle private_nh("~");

    //tf_frame="/base_link";
    //private_nh="~";
    private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
  }
  ~ShapeFinderNode() {}

  /*std::vector<int> inliers;
  simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, char *title)
  {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
    //size_t size = cloud->size();
    pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
    for (int i = 0; i < cloud->points.size(); i++)
    {
      centroid.add(pcl::PointXYZRGB(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }
    pcl::PointXYZRGB c1;
    centroid.get(c1);
    viewer->addText3D(title, c1, 0.05, 1.0, 0.5, 0.5, title, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters();
    return (viewer);
  }*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  ret_sphere(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
        model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    model->setRadiusLimits(0.0, 0.25);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
    pcl::copyPointCloud(*cloud, inliers, *final);
    return final;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  ret_plane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
    pcl::copyPointCloud(*cloud, inliers, *final);
    return final;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  ret_cylinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.5);
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);
    return cloud_cylinder;
  }

  pcl::PointCloud<pcl::PointXYZRGB> shapefind(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, int j)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZ>);
    size_t size_cloud = cloud_cluster->size();
    int big_plane = 0;
    int horizontal = 0;
    int vertical = 0;

    printf("cluster %d, size: %d\n", j, size_cloud);
    cloud_s = ret_sphere(cloud_cluster);
    cloud_p = ret_plane(cloud_cluster);
    cloud_c = ret_cylinder(cloud_cluster);

    size_t size_cloud_s = cloud_s->size();
    printf("Size sphere point cloud: %zu\n", size_cloud_s);
    float overlap_s = (float)size_cloud_s / (float)size_cloud;
    printf("sphere overlap: %f\n", overlap_s);

    size_t size_cloud_p = cloud_p->size();
    printf("Size plane point cloud: %zu\n", size_cloud_p);
    float overlap_p = (float)size_cloud_p / (float)size_cloud;
    printf("plane overlap: %f\n", overlap_p);

    size_t size_cloud_c = cloud_c->size();
    printf("Size cylinder point cloud: %zu\n", size_cloud_c);
    float overlap_c = (float)size_cloud_c / (float)size_cloud;
    printf("cylinder overlap: %f\n", overlap_c);

    int cluster_shape = 0; //0-nothing(sub threshold-th), 1-plane, 2-sphere, 3-cylinder
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    if ((overlap_s > overlap_p) && (overlap_s > overlap_c) && (overlap_s > th) && (size_cloud_s > min_cloud_size))
    {
      cluster_shape = 2;
      pcl::copyPointCloud(*cloud_s, *cloud_colored_cluster);
    }

    if ((overlap_p > overlap_s) && (overlap_p > overlap_c) && (overlap_p > th) && (size_cloud_p > min_cloud_size))
    {
      cluster_shape = 1;
      pcl::copyPointCloud(*cloud_p, *cloud_colored_cluster);
      if (cloud_colored_cluster->size() > big_plane_size)
      {
        big_plane = 1;
      }
      Eigen::Matrix3f covariance_matrix;
      Eigen::Vector4f xyz_centroid;
      compute3DCentroid(*cloud_p, xyz_centroid);
      computeCovarianceMatrix(*cloud_p, xyz_centroid, covariance_matrix);
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud(cloud_p);
      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
      ne.setSearchMethod(tree);
      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch(0.03);
      // Compute the features
      ne.compute(*cloud_normals);

      double PI = 3.14159265;
      int pnr = cloud_normals->size() / 2;
      Eigen::Vector3f normalvector(cloud_normals->points[pnr].normal_x, cloud_normals->points[pnr].normal_y, cloud_normals->points[pnr].normal_z);
      /*float angle = acos(gravity.dot(normalvector));
      std::cout << "gravity: " << gravity[0] << ", " <<gravity[1] <<", "<<gravity[2]<< std::endl;
       std::cout << "normal: " << normalvector[0] << ", " <<normalvector[1] <<", "<<normalvector[2]<< std::endl;
       std::cout << "angle: " << angle << std::endl;

      if (angle < PI / 12 && angle > PI - PI / 12)
        horizontal = 1;
      if (angle > PI / 2 - PI / 12 && angle < PI / 2 + PI / 12)
        vertical = 1;*/
    }

    if ((overlap_c > overlap_s) && (overlap_c > overlap_p) && (overlap_c > th) && (size_cloud_c > min_cloud_size))
    {
      cluster_shape = 3;
      pcl::copyPointCloud(*cloud_c, *cloud_colored_cluster);
    }

    std::uint8_t r = 255 * (cluster_shape == 2), g = 255 * (cluster_shape == 1), b = 255 * (cluster_shape == 3);
    if (big_plane)
    {
      r = 125 * vertical;
      g = 125;
      b = 125 * horizontal;
    }
    std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    //cloud_colored_cluster->points.rgb = *reinterpret_cast<float *>(&rgb);
    printf("cluster+shape=%d\nrgb:%d,%d,%d\n", cluster_shape, r, g, b);
    for (int i = 0; i < cloud_colored_cluster->size(); i++)
    {
      cloud_colored_cluster->points[i].rgb = *reinterpret_cast<float *>(&rgb);
    }

    printf("SIZE of cluster_colored_cluster:%d\n\n", cloud_colored_cluster->size());
    return *cloud_colored_cluster;
  }

  void clusterfind(const PointCloud::ConstPtr &cloud_in)
  {
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 cloud_colored_sensor;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    //reader.read("/home/szilard/catkin_ws/src/shape_finder/src/table_scene_lms400.pcd", *cloud);
    //pcl::fromROSMsg(*cloud_in, cloud);
    //pcl_ros::transformPointCloud(*cloud_in, *cloud, const tf::Transform &transform);
    pcl::copyPointCloud(*cloud_in, *cloud);

    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);

    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int j = 0;
    int i = 0, nr_points = (int)cloud_filtered->size();
    while (cloud_filtered->size() > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Get the points associated with the planar surface
      extract.filter(*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_f);
      *cloud_filtered = *cloud_f;

      *cloud_colored_pcl = *cloud_colored_pcl + shapefind(cloud_plane, j);

      /*std::stringstream ss;
      ss << "/home/szilard/catkin_ws/src/shape_finder/src/cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ>(ss.str(), *cloud_plane, false);*/
      j++;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->push_back((*cloud_filtered)[*pit]);

      cloud_cluster->width = cloud_cluster->size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      size_t size_cloud = cloud_cluster->size();

      *cloud_colored_pcl = *cloud_colored_pcl + shapefind(cloud_cluster, j);

      /*std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
      std::stringstream ss;
      ss << "/home/szilard/catkin_ws/src/shape_finder/src/cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); */
      j++;
    }

    pcl::toROSMsg(*cloud_colored_pcl, cloud_colored_sensor);
    cloud_colored_sensor.header.frame_id = "pico_zense_depth_frame";
    //ros::Rate loop_rate(10);

    std::string fields_list = pcl::getFieldsList(cloud_colored_sensor);
    std::cout << "Colored PointCloud before filtering has: " << cloud_colored_sensor.width << " data points."
              << " points " << fields_list << "\" in frame \"" << cloud_colored_sensor.header.frame_id << std::endl;

    /*while (ros::ok())
    {
      cloud_colored_sensor.header.stamp = ros::Time::now();
      pub_.publish(cloud_colored_sensor);
      ros::spinOnce();
      loop_rate.sleep();
    }*/
    cloud_colored_sensor.header.stamp = ros::Time::now();
    pub_.publish(cloud_colored_sensor);
  }

  void
  dynReconfCallback(shape_finder::shape_finder_nodeConfig &config, uint32_t level)
  {
    //pt_.setFilterLimits(config.lower_limit, config.upper_limit);
    leaf_size = config.leafsize;
    th = config.overlap_threshold;
    big_plane_size = config.big_plane_size;
    min_cloud_size = config.min_cloud_size;
  }

  void
  cloudCallback(const PointCloud::ConstPtr &cloud_in)
  {
    clusterfind(cloud_in);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {
    //ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double g = 9.89;
    double PI = 3.14159265;
    la_x = msg->linear_acceleration.x;
    la_y = msg->linear_acceleration.y;
    la_z = msg->linear_acceleration.z;
    gravity[0] = la_x;
    gravity[1] = la_y;
    gravity[2] = la_z;
    std::string frame = msg->header.frame_id;
    ros::Time time = msg->header.stamp;
    /*double angle_x=atan(la_z/la_y);
   double angle_y=atan(la_x/la_z);
   double angle_z=atan(la_y/la_x);
tf2::Quaternion q;
q.normalize();
q.setRPY( angle_x, angle_y, angle_z );
ROS_INFO_STREAM(angle_x);
ROS_INFO_STREAM(angle_y);
ROS_INFO_STREAM(angle_z);
ROS_INFO_STREAM(q);*/

    /*rviz::DisplayContext* context_;
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    context_->getFrameManager()->getTransform(msg->header.frame_id,
                                              msg->header.stamp,
                                              position, orientation);*/

    tf::Quaternion bt_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Vector3 bt_position(0.0, 0.0, 0.0);
    if (bt_orientation.x() == 0.0 && bt_orientation.y() == 0.0 && bt_orientation.z() == 0.0 && bt_orientation.w() == 0.0)
    {
      bt_orientation.setW(1.0);
    }

    tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation, bt_position), time, frame);
    tf::Stamped<tf::Pose> pose_out;

    // convert pose into new frame
    boost::shared_ptr<tf::TransformListener> tf_;
    tf_.reset(new tf::TransformListener(ros::NodeHandle(), ros::Duration(10*60), true));
    try
    {
      tf_->transformPose(tf_frame, pose_in, pose_out);
    }
    catch (std::runtime_error &e)
    {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", frame.c_str(), tf_frame.c_str(), e.what());
    }
    bt_orientation = pose_out.getRotation();

    visualization_msgs::Marker marker;
    uint32_t shape = visualization_msgs::Marker::ARROW;
    marker.ns = "basic_shapes";
    marker.id = 1;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    //Eigen::Affine3f rotation;
    //pcl::getTransformationFromTwoUnitVectors(gravity.unitOrthogonal(), gravity, rotation);
    //Eigen::Quaternionf qu(rotation.inverse().rotation());
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = bt_orientation.x();
    marker.pose.orientation.y = bt_orientation.y();
    marker.pose.orientation.z = bt_orientation.z();
    marker.pose.orientation.w = bt_orientation.w();
    /*Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    geometry_msgs::Point pt;
    pt.x = centroid(0);
    pt.y = centroid(1);
    pt.z = centroid(2);
    marker.points.push_back(pt);
    pt.x = centroid(0) + gravity(0);
    pt.y = centroid(1) + gravity(1);
    pt.z = centroid(2) + gravity(2);
    marker.points.push_back(pt);*/

    marker.scale.x = 1.0;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();
    marker.header.frame_id = tf_frame;
    marker.header.stamp = ros::Time::now();

    marker_pub.publish(marker);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh;
  std::string tf_frame = "pico_zense_depth_frame";
  float leaf_size = 0.02f;
  float th = 0.6;
  size_t big_plane_size = 1000;
  size_t min_cloud_size = 100;
  ros::Subscriber sub_;
  ros::Subscriber sub_imu;
  sensor_msgs::Imu::Ptr imu;
  ros::Publisher marker_pub;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;
  double la_x, la_y, la_z;
  double angle_x0 = 90, angle_y0 = 90, angle_z0 = 0;
  double angle_x, angle_y, angle_z;
  double angle_xrot, angle_yrot, angle_zrot;
  dynamic_reconfigure::Server<shape_finder::shape_finder_nodeConfig> config_server_;
  Eigen::Vector3f gravity;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "shapefinder_node");

  ShapeFinderNode sf;
  //sf.clusterfind();
  ros::spin();
  //return (0);
}
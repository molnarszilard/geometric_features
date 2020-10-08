#include "includes.h"

namespace shape_finder
{
  class GravityNodelet : public nodelet::Nodelet
  {
  private:
    double PI = 3.14159265;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh;
    std::string tf_frame = "pico_zense_depth_frame";
    ros::Subscriber sub_imu;
    ros::Publisher gravity_marker_pub;
    ros::Publisher g_pub;
    dynamic_reconfigure::Server<shape_finder::gravity_nodeletConfig> config_server_;
    Eigen::Vector3f gravity, gravity0;
    bool g_init;
    double lean_tolerance;
    geometry_msgs::Point gravity_vector;

    void
    dynReconfCallback(shape_finder::gravity_nodeletConfig &config, uint32_t level)
    {
      lean_tolerance = config.lean_tolerance * PI / 180;
    }

    void
    imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
      if (!g_init)
      {
        gravity0[0] = msg->linear_acceleration.x;
        gravity0[1] = msg->linear_acceleration.y;
        gravity0[2] = msg->linear_acceleration.z;
        gravity0.normalize();
        g_init = true;
      }
      gravity[0] = msg->linear_acceleration.x;
      gravity[1] = msg->linear_acceleration.y;
      gravity[2] = msg->linear_acceleration.z;
      gravity.normalize();

      gravity_vector.x = gravity[0];
      gravity_vector.y = gravity[1];
      gravity_vector.z = gravity[2];
      g_pub.publish(gravity_vector);
      std::cout << "GRAVITY0 X: " << gravity0[0] << ", Y: " << gravity0[1] << ", Z: " << gravity0[2] << std::endl;
      std::cout << "GRAVITY ORIG X: " << gravity[0] << ", Y: " << gravity[1] << ", Z: " << gravity[2] << std::endl;
      float g_angle = acos(gravity.dot(gravity0));
      std::cout << "angle of camera: " << g_angle << std::endl;
      if (g_angle > lean_tolerance)
      {
        if (gravity[1] < 0)
          std::cout << "The robot is UPSIDE_DOWN!" << std::endl;
        else
        {
          if (abs(gravity[0]) > abs(gravity[2]))
          {
            if (gravity[0] > 0)
              std::cout << "The robot is LEANING RIGHT!" << std::endl;
            else
              std::cout << "The robot is LEANING LEFT!" << std::endl;
          }
          else
          {
            if (gravity[2] > 0)
              std::cout << "The robot is LEANING FORWARD!" << std::endl;
            else
              std::cout << "The robot is LEANING BACKWARDS!" << std::endl;
          }
        }
      }
      visualization_msgs::Marker gravity_marker;
      uint32_t shape = visualization_msgs::Marker::ARROW;
      gravity_marker.ns = "basic_shapes";
      gravity_marker.id = 1;
      gravity_marker.type = shape;
      gravity_marker.action = visualization_msgs::Marker::ADD;
      Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
      geometry_msgs::Point pt;
      pt.x = centroid(0);
      pt.y = centroid(1);
      pt.z = centroid(2);
      gravity_marker.points.push_back(pt);
      pt.x = centroid(0) + gravity(0);
      pt.y = centroid(1) + gravity(1);
      pt.z = centroid(2) + gravity(2);
      gravity_marker.points.push_back(pt);
      gravity_marker.scale.x = 0.05;
      gravity_marker.scale.y = 0.1;
      gravity_marker.scale.z = 0.05;
      gravity_marker.color.r = 1.0f;
      gravity_marker.color.g = 0.0f;
      gravity_marker.color.b = 0.0f;
      gravity_marker.color.a = 0.5;
      gravity_marker.lifetime = ros::Duration();
      gravity_marker.header.frame_id = tf_frame;
      gravity_marker.header.stamp = ros::Time::now();
      gravity_marker_pub.publish(gravity_marker);
    }

  public:
    virtual void
    onInit()
    {
      //private_nh("~");
      g_init = false;
      gravity_marker_pub = nh_.advertise<visualization_msgs::Marker>("imu_out", 1);
      config_server_.setCallback(boost::bind(&GravityNodelet::dynReconfCallback, this, _1, _2));
      sub_imu = nh_.subscribe("imu_data", 1, &GravityNodelet::imuCallback, this);
      g_pub = private_nh.advertise<geometry_msgs::Point>("gravity_point", 1);
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
    }
  };

} // namespace shape_finder
PLUGINLIB_EXPORT_CLASS(shape_finder::GravityNodelet, nodelet::Nodelet);
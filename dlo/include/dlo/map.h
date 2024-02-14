/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"

#include <dlo_msgs/srv/save_pcd.hpp>

namespace dlo
{

  class MapNode : public rclcpp::Node
  {

  public:
    MapNode(const std::string &node_name);

    void start();
    void stop();

  private:
    void publishTimerCB();

    void keyframeCB(const sensor_msgs::msg::PointCloud2::SharedPtr keyframe);

    bool savePcd(dlo_msgs::srv::SavePCD::Request::SharedPtr req,
                 dlo_msgs::srv::SavePCD::Response::SharedPtr res);

    void getParams();

    rclcpp::TimerBase::SharedPtr publish_timer;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

    rclcpp::Service<dlo_msgs::srv::SavePCD>::SharedPtr save_pcd_srv;

    pcl::PointCloud<PointType>::Ptr dlo_map;
    pcl::VoxelGrid<PointType> voxelgrid;

    rclcpp::Time map_stamp;
    std::string odom_frame;

    bool publish_full_map;
    double publish_freq;
    double leaf_size;
  };

} // namespace dlo
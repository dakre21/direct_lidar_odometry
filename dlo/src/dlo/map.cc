/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"

/**
 * Constructor
 **/

dlo::MapNode::MapNode(const std::string &node_name) : rclcpp::Node(node_name)
{
  this->getParams();

  if (this->publish_full_map) 
  {
    const uint32_t sec = 1000 * this->publish_freq;
    this->publish_timer = this->create_wall_timer(std::chrono::milliseconds(sec), std::bind(&dlo::MapNode::publishTimerCB, this));
  }

  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("keyframes", 1, std::bind(&dlo::MapNode::keyframeCB, this, std::placeholders::_1));
  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);

  this->save_pcd_srv = this->create_service<dlo_msgs::srv::SavePCD>("save_pcd", std::bind(&dlo::MapNode::savePcd, this, std::placeholders::_1, std::placeholders::_2));

  // initialize map
  this->dlo_map = std::make_shared<pcl::PointCloud<PointType>>();

  RCLCPP_INFO(this->get_logger(), "DLO Map Node Initialized");
}

/**
 * Get Params
 **/

void dlo::MapNode::getParams()
{
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<bool>("publishFullMap", true);
  this->declare_parameter<double>("publishFreq", 1.0);
  this->declare_parameter<double>("leafSize", 0.5);

  this->odom_frame = this->get_parameter("odom_frame").as_string();
  this->publish_full_map = this->get_parameter("publishFullMap").as_bool();
  this->publish_freq = this->get_parameter("publishFreq").as_double();
  this->leaf_size = this->get_parameter("leafSize").as_double();
}

/**
 * Start Map Node
 **/

void dlo::MapNode::start()
{
  RCLCPP_INFO(this->get_logger(), "Starting DLO Map Node");
}

/**
 * Stop Map Node
 **/

void dlo::MapNode::stop()
{
  RCLCPP_WARN(this->get_logger(), "Stopping DLO Map Node");

  // shutdown
  rclcpp::shutdown();
}

/**
 * Publish Timer Callback
 **/

void dlo::MapNode::publishTimerCB()
{
  if (this->dlo_map->points.size() == this->dlo_map->width * this->dlo_map->height)
  {
    sensor_msgs::msg::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlo_map, map_ros);
    map_ros.header.stamp = this->get_clock()->now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub->publish(map_ros);
  }
}

/**
 * Node Callback
 **/

void dlo::MapNode::keyframeCB(const sensor_msgs::msg::PointCloud2::SharedPtr keyframe)
{
  // convert scan to pcl format
  auto keyframe_pcl = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size, this->leaf_size, this->leaf_size);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save keyframe to map
  this->map_stamp = keyframe->header.stamp;
  *this->dlo_map += *keyframe_pcl;

  if (!this->publish_full_map)
  {
    if (keyframe_pcl->points.size() == keyframe_pcl->width * keyframe_pcl->height)
    {
      sensor_msgs::msg::PointCloud2 map_ros;
      pcl::toROSMsg(*keyframe_pcl, map_ros);
      map_ros.header.stamp = this->get_clock()->now();
      map_ros.header.frame_id = this->odom_frame;
      this->map_pub->publish(map_ros);
    }
  }
}

bool dlo::MapNode::savePcd(dlo_msgs::srv::SavePCD::Request::SharedPtr req,
                           dlo_msgs::srv::SavePCD::Response::SharedPtr res)
{
  auto m = std::make_shared<pcl::PointCloud<PointType>>(*this->dlo_map);

  float leaf_size = req->leaf_size;
  std::string p = req->save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlo_map.pcd"
            << "... ";
  std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlo_map.pcd", *m);
  res->success = ret == 0;

  if (res->success)
    std::cout << "done" << std::endl;
  else
    std::cout << "failed" << std::endl;

  return res->success;
}
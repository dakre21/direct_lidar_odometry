/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<dlo::MapNode>("dlo_map");
  rclcpp::spin(node);

  return 0;
}

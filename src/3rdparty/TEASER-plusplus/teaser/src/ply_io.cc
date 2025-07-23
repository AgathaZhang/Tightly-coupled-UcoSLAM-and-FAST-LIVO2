/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include <sstream>
#include <fstream>
#include <iostream>
#include <memory>
#include <cstring>

#include "teaser/ply_io.h"


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// Internal datatypes for storing ply vertices
struct float3 {
  float x, y, z;
};
struct double3 {
  double x, y, z;
};

int teaser::PLYReader::read(const std::string& file_name, teaser::PointCloud& cloud) {
   pcl::PCDReader reader;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZ>());
  
   reader.read<pcl::PointXYZ>(file_name, *cloud_read);

  for(int i=0;i<cloud_read->points.size();i++){
    cloud.push_back(
        {static_cast<float>(cloud_read->points[i].x), static_cast<float>(cloud_read->points[i].y), static_cast<float>(cloud_read->points[i].z)});
  }
    
  

  return 0;
}

int teaser::PLYWriter::write(const std::string& file_name, const teaser::PointCloud& cloud,
                             bool binary_mode) {
  /**
  // Open file buffer according to binary mode
  std::filebuf fb;
  if (binary_mode) {
    fb.open(file_name, std::ios::out | std::ios::binary);
  } else {
    fb.open(file_name, std::ios::out);
  }

  // Open output stream
  std::ostream outstream(&fb);
  if (outstream.fail()) {
    std::cerr << "Failed to open " << file_name << std::endl;
    return -1;
  }

  // Use tinyply to write to ply file
  tinyply::PlyFile ply_file;
  std::vector<float3> temp_vertices;
  for (auto& i : cloud) {
    temp_vertices.push_back({i.x, i.y, i.z});
  }
  ply_file.add_properties_to_element(
      "vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, temp_vertices.size(),
      reinterpret_cast<uint8_t*>(temp_vertices.data()), tinyply::Type::INVALID, 0);
  ply_file.write(outstream, binary_mode);
  */
  return 0;
}

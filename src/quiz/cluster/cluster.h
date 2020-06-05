#ifndef CLUSTER_H
#define CLUSTER_H
#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points);

void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0);

void clusterHelper(const std::vector<std::vector<float>>& points, std::vector<int>& cluster, int index,std::vector<bool>& processed, KdTree* tree, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif
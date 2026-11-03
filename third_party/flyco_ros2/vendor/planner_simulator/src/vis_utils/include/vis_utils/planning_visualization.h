/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the header file of PlanningVisualization class, which
 *                   implements the visualization tools for FlyCo.
 * Copyright    :    Copyright (c) 2026 Chen Feng and Guiyong Zheng.
 * License      :    PolyForm Noncommercial License 1.0.0
 *                   <https://polyformproject.org/licenses/noncommercial/1.0.0/>
 *
 *                   This software is released for noncommercial research and
 *                   educational use only. You may use, modify, and distribute
 *                   this software for noncommercial purposes, subject to the
 *                   terms of the PolyForm Noncommercial License 1.0.0.
 *
 *                   Commercial use, including use in commercial products,
 *                   commercial services, paid consulting, or internal business
 *                   operations, is prohibited without prior written permission
 *                   from the copyright holders.
 *
 *                   This software is provided "as is", without warranty of any
 *                   kind, express or implied.
 * Project      :    FlyCo: Foundation Model-Empowered Drones for Autonomous 3D Structure Scanning in Open-World Environments
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include "quadrotor_msgs/msg/polynomial_traj.hpp"
#include <cstdlib>
#include <ctime>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <unordered_map>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::vector, std::map, std::unordered_map;
namespace flyco {

class PlanningVisualization {
private:

  /* data */
  visualization_msgs::msg::MarkerArray path_markers_;
  visualization_msgs::msg::MarkerArray vp_set_;

  visualization_msgs::msg::MarkerArray path_markers_updated_;
  visualization_msgs::msg::MarkerArray vp_set_updated_;
  /* visib_pub is seperated from previous ones for different info */
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inlier_candidates_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr strict_inlier_candidates_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relaxed_inlier_candidates_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_inlier_candidates_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dcrosa_topology_vertices_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dcrosa_topology_edges_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dcrosa_topology_branches_publisher_;
  // Local Planner
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_pub_;      
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr localVP_pub_;  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_path_order_pub_;  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localReg_pub_;   
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr localBox_pub_;     
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_updated_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr localVP_updated_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_updated_path_order_pub_; 
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_used_g_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_used_g_prior_pub_;
  // ROSA
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_pub_; 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_; 
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rosa_orientation_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr drosa_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr decomp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sub_space_pub_;
  // ROSA Intermediate Vis
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inliers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr inliers_intersection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inliers_isec_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr subspace_cluster_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intra_edge_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr subspace_skeleton_pub_;
  // Global Planner
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr init_vps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hcopp_occ_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hcopp_internal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hcopp_uncovered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hcopp_global_uncovered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hcopp_correctnormal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hcopp_sub_finalvps_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hcopp_globalseq_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hcopp_globalboundary_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hcopp_local_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hcopp_full_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hcopp_full_waypts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr jointSphere_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hcoppYaw_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_start_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr before_consistency_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_next_sub_consistency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_vp_vis_graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_vp_decomp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_decomp_path_pub_;
  // Flight
  nav_msgs::msg::Path path_msg;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr exec_path_pub_;
  // Prediction
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pred_mesh_pub_;

public:
  PlanningVisualization(/* args */) {
  }
  ~PlanningVisualization() {
  }
  PlanningVisualization(const rclcpp::Node::SharedPtr& nh);

  /* Skeleton */
  void publishSurface(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);
  void publishMesh(std::string& mesh);
  void publishSurfaceNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publishROSAOrientation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publish_dROSA(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);
  void publishInliers(const pcl::PointCloud<pcl::PointXYZ>& inliers, const pcl::PointCloud<pcl::PointXYZ>& all_pts, const Eigen::MatrixXf& directions, const vector<vector<Eigen::Vector3d>>& isec_pts);
  void publishInlierCandidates(const Eigen::MatrixXd& candidates);
  void publishInlierCandidateClasses(const Eigen::MatrixXd& strict_candidates,
                                     const Eigen::MatrixXd& relaxed_candidates,
                                     const Eigen::MatrixXd& rejected_candidates);
  void publishVisGraph(const vector<Eigen::Vector3d>& inliers, const Eigen::MatrixXi& vis_graph, const vector<vector<int>>& subspace_sets);
  void publishIntraEdge(const vector<Eigen::MatrixX3d>& intra_edges);
  void publishSubspaceSkeleton(const vector<vector<Eigen::Vector3d>>& skeleton_paths);
  void publishDcrosaTopology(const Eigen::MatrixXd& skelver,
                             const Eigen::MatrixXi& skeladj,
                             const vector<vector<Eigen::Vector3d>>& skeleton_paths);
  void publishSubSpace(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& space);
  void publishOccupied(pcl::PointCloud<pcl::PointXYZ>& occupied);
  void publishInternal(pcl::PointCloud<pcl::PointXYZ>& internal);
  void publishUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered);
  void publishGlobalUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered);
  void publishRevisedNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  /* Global Planning */
  void publishFinalFOV(map<int, vector<vector<Eigen::Vector3d>>>& list1, map<int, vector<vector<Eigen::Vector3d>>>& list2, map<int, vector<double>>& yaws);
  void publishGlobalSeq(Eigen::Vector3d& start_, vector<Eigen::Vector3d>& sub_rep, vector<int>& global_seq);
  void publishGlobalBoundary(Eigen::Vector3d& start_, map<int, vector<int>>& boundary_id_, map<int, vector<Eigen::VectorXd>>& sub_vps, vector<int>& global_seq);
  void publishLocalPath(map<int, vector<Eigen::VectorXd>>& sub_paths_);
  void publishHCOPPPath(vector<Eigen::VectorXd>& fullpath_);
  void publishJointSphere(vector<Eigen::Vector3d>& joints, double& radius, vector<vector<Eigen::Vector3d>>& InnerVps);
  void publishYawTraj(vector<Eigen::Vector3d>& waypt, vector<double>& yaw);
  void publishInitVps(pcl::PointCloud<pcl::PointNormal>::Ptr& init_vps);
  void publishGlobalStart(const Eigen::VectorXd& start_);
  void publishExecPart(const vector<Eigen::VectorXd>& path);
  void publishBeforeConsistencyPath(const vector<Eigen::VectorXd>& path);
  void publishGlobalNextSubConsistency(const vector<Eigen::Vector3d>& last, const vector<Eigen::Vector3d>& current);
  void publishGlobalVPVisGraph(const vector<Eigen::VectorXd>& vps, const Eigen::MatrixXi& vp_vis_graph, const vector<vector<int>>& decomp_vps);
  void publishDecompGlobalPath(const vector<Eigen::Vector3d>& global_path);
  /* Local Planning */
  void publishLocalRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound);
  void publishCurPath(const vector<Eigen::VectorXd>& local_path, const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2);
  void publishUpdatedCurPath(const vector<Eigen::VectorXd>& local_path, const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2);
  void publishLocalUsedGlobal(const vector<Eigen::VectorXd>& path, const vector<Eigen::VectorXd>& prior_path);
  /* Prediction */
  void publishPredMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces);

private:
  vector<double> red_list;
  vector<double> green_list;
  vector<double> blue_list;
};
}
#endif

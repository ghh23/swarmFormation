#include <string.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
#include <iostream>


#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>


using namespace std;

static string mesh_resource, map_mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale, half_len_, half_wid_;
bool cross_config = false;
bool tf45 = false;
bool cov_pos = false;
bool cov_vel = false;
bool cov_color = false;
bool origin = false;
bool isOriginSet = false;


ros::Publisher fov_pub_;
ros::Publisher fov_edge_Pub;
ros::Publisher drone_frame_pub;
ros::Timer map_vis_timer_;
ros::Subscriber drone_0_fov_sub_;

string _frame_id;

// fov visualize
double max_dis_ = 4.0;
double x_max_dis_gain_ = 0.64;
double y_max_dis_gain_ = 0.82;
visualization_msgs::Marker markerNode_fov;
visualization_msgs::Marker markerEdge_fov;
visualization_msgs::Marker marker_line, fast_marker_line;
std::vector<Eigen::Vector3d> fov_node;

std::vector<Eigen::Vector3d> lz_set_;
visualization_msgs::Marker drone_frame;

double map_scale_;
Eigen::Vector3d T_;
Eigen::Quaterniond map_q_;

void fov_visual_init(std::string msg_frame_id) {
  double x_max_dis = max_dis_ * x_max_dis_gain_;
  double y_max_dis = max_dis_ * y_max_dis_gain_;

  fov_node.resize(5);
  fov_node[0][0] = 0;
  fov_node[0][1] = 0;
  fov_node[0][2] = 0;

  fov_node[1][2] = x_max_dis;
  fov_node[1][1] = y_max_dis;
  fov_node[1][0] = max_dis_;

  fov_node[2][2] = x_max_dis;
  fov_node[2][1] = -y_max_dis;
  fov_node[2][0] = max_dis_;

  fov_node[3][2] = -x_max_dis;
  fov_node[3][1] = -y_max_dis;
  fov_node[3][0] = max_dis_;

  fov_node[4][2] = -x_max_dis;
  fov_node[4][1] = y_max_dis;
  fov_node[4][0] = max_dis_;

  markerNode_fov.header.frame_id = msg_frame_id;
  // markerNode_fov.header.stamp = msg_time;
  markerNode_fov.action = visualization_msgs::Marker::ADD;
  markerNode_fov.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNode_fov.ns = "fov_nodes";
  // markerNode_fov.id = 0;
  markerNode_fov.pose.orientation.w = 1;
  markerNode_fov.scale.x = 0.05;
  markerNode_fov.scale.y = 0.05;
  markerNode_fov.scale.z = 0.05;
  markerNode_fov.color.r = 0;
  markerNode_fov.color.g = 0.8;
  markerNode_fov.color.b = 1;
  markerNode_fov.color.a = 1;

  markerEdge_fov.header.frame_id = msg_frame_id;
  // markerEdge_fov.header.stamp = msg_time;
  markerEdge_fov.action = visualization_msgs::Marker::ADD;
  markerEdge_fov.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge_fov.ns = "fov_edges";
  // markerEdge_fov.id = 0;
  markerEdge_fov.pose.orientation.w = 1;
  markerEdge_fov.scale.x = 0.05;
  markerEdge_fov.color.r = 0.5f;
  markerEdge_fov.color.g = 0.0;
  markerEdge_fov.color.b = 0.0;
  markerEdge_fov.color.a = 1;

  drone_frame.header.frame_id = msg_frame_id;
  // markerEdge_fov.header.stamp = msg_time;
  drone_frame.action = visualization_msgs::Marker::ADD;
  drone_frame.type = visualization_msgs::Marker::LINE_LIST;
  drone_frame.ns = "fov_edges";
  // markerEdge_fov.id = 0;
  drone_frame.pose.orientation.w = 1;
  drone_frame.scale.x = 0.05;
  drone_frame.color.r = 0.5f;
  drone_frame.color.g = 0.0;
  drone_frame.color.b = 0.0;
  drone_frame.color.a = 1;
}

void pub_fov_visual(Eigen::Vector3d& p, Eigen::Quaterniond& q, const ros::Time& s) {
  visualization_msgs::Marker clear_previous_msg;
  clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray markerArray_fov;
  markerNode_fov.points.clear();
  markerEdge_fov.points.clear();
  std::vector<geometry_msgs::Point> fov_node_marker;
  for (int i = 0; i < (int)fov_node.size(); i++) {
    Eigen::Vector3d vector_temp;
    vector_temp = q * fov_node[i] + p;
    geometry_msgs::Point point_temp;
    point_temp.x = vector_temp[0];
    point_temp.y = vector_temp[1];
    point_temp.z = vector_temp[2];
    fov_node_marker.push_back(point_temp);
  }

  // markerNode_fov.points.push_back(fov_node_marker[0]);
  // markerNode_fov.points.push_back(fov_node_marker[1]);
  // markerNode_fov.points.push_back(fov_node_marker[2]);
  // markerNode_fov.points.push_back(fov_node_marker[3]);
  // markerNode_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerEdge_fov.points.push_back(fov_node_marker[1]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[2]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[3]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[4]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  // markerArray_fov.markers.push_back(clear_previous_msg);
  markerArray_fov.markers.push_back(markerNode_fov);
  markerArray_fov.markers.push_back(markerEdge_fov);
  fov_pub_.publish(markerArray_fov);
  markerEdge_fov.header.stamp = s;
  fov_edge_Pub.publish(clear_previous_msg);
  fov_edge_Pub.publish(markerEdge_fov);

//   visualization_msgs::MarkerArray markerArray_drone_frame;
//   drone_frame.points.clear();
//   std::vector<geometry_msgs::Point> drone_frame_marker;
//   for (int i = 0; i < (int)lz_set_.size(); i++) {
//     Eigen::Vector3d vector_temp;
//     vector_temp = q * lz_set_[i] + p;
//     geometry_msgs::Point point_temp;
//     point_temp.x = vector_temp[0];
//     point_temp.y = vector_temp[1];
//     point_temp.z = vector_temp[2];
//     drone_frame_marker.push_back(point_temp);
//   }
//   // std::cout << "drone_frame pub0" << std::endl;


//   drone_frame.points.push_back(drone_frame_marker[0]);
//   drone_frame.points.push_back(drone_frame_marker[1]);
    
//   drone_frame.points.push_back(drone_frame_marker[1]);
//   drone_frame.points.push_back(drone_frame_marker[2]);

//   drone_frame.points.push_back(drone_frame_marker[1]);
//   drone_frame.points.push_back(drone_frame_marker[3]);

//   markerArray_drone_frame.markers.push_back(drone_frame);
//   // std::cout << "drone_frame pub1" << std::endl;
//   drone_frame.header.stamp = s;
//   drone_frame_pub.publish(clear_previous_msg);
//   // std::cout << "drone_frame pub2" << std::endl;
//   drone_frame_pub.publish(drone_frame);
//   // std::cout << "drone_frame pub3" << std::endl;
}


void drone_0_fovCallback(const geometry_msgs::PoseStamped &msg){

  Eigen::Vector3d fov_p_0(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Quaterniond fov_q_0(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
  pub_fov_visual(fov_p_0, fov_q_0, msg.header.stamp);

  }



int main(int argc, char** argv) {
  ros::init(argc, argv, "fov_visualization");
  ros::NodeHandle nh("~");
  drone_0_fov_sub_ = nh.subscribe("/drone_0_odom_visualization/pose", 1, drone_0_fovCallback);

  fov_pub_ = nh.advertise<visualization_msgs::MarkerArray>("fov_visual", 1, false);
  fov_edge_Pub = nh.advertise<visualization_msgs::Marker>("fov_E", 1, true);
  //drone_frame_pub = n.advertise<visualization_msgs::Marker>("drone_frame", 1, true);

  fov_visual_init("world");
  ros::spin();

  return 0;
}

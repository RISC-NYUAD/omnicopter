#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <ros/publisher.h>

#include <octomap/OcTreeNode.h>
#include <octomap/OcTreeKey.h>

// Pseudo-type: in practice, include DynamicEDT3D headers
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

struct QuadForm {
  Eigen::SparseMatrix<double> H;
  int nvars{0};
};

struct Sampled {
  std::vector<double> T;
  int n_rows{0};
  struct RowMap { int seg; int cidx[4]; double w[4]; };
  std::vector<RowMap> map;
  std::vector<std::array<double,3>> closest_points;
};

struct Params {
  int dim{3};
  int num_ctrl_pts{10};
  int N_samples{20};
  double C_len{0.1}, C_goal{8.0};
  double C_speed{0.1}, C_acc{0.02}, C_jerk{0.005};
  double C_quat{1.0},  C_theta{0.5};
  double C_init{2.0};
  double C_slack{1000.0};
  std::vector<double> x0_3D, xf_3D;
  std::vector<double> x0, xf, x_init;
  std::vector<double> RPY0;
  std::vector<std::array<double,3>> points;
  Sampled g_lastS;// last sampled spline (positions + map)
  bool g_have_samples = false;
  
  // closed-form smoothness forms
  QuadForm Qspeed, Qacc, Qjerk;
  bool     forms_ready{false};

  // interior bounds per dimension (read from ROS params)
  std::array<double,3> lb3{-6.0,-6.0,-6.0}, ub3{6.0,6.0,6.0};
  std::vector<double> lb7{-6.0,-6.0,-6.0, -1, -1, -1, -1}, ub7{6.0,6.0,6.0, 1, 1, 1, 1};

  double d_safe_xy{0.5}, d_safe_z{0.7};
  std::shared_ptr<DynamicEDTOctomap> edt;
  double resolution;  
  // PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_const;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

};



void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                   Params* par,
                   ros::Publisher* cloud_pub);





// Functions
Eigen::Matrix4d gram_speed();
Eigen::Matrix4d gram_acc();
Eigen::Matrix4d gram_jerk();
QuadForm assemble_form(const Params& par,
                       const std::vector<std::array<int,4>>& segments,
                       const Eigen::Matrix4d& M);

std::vector<std::array<int,4>> build_segments(int num_ctrl_pts);
void cubic_basis(double t, double b[4]);
Sampled sample_with_map(const std::vector<double>& P, const Params& par);

std::array<double,3> nearest_point3d(const std::vector<std::array<double,3>>& pts,
                                     double x,double y,double z);
std::array<double,3> nearest_point3d_forward(const std::vector<std::array<double,3>>& pts,
                                     double x,double y,double z,
                                     double x_next,double y_next,double z_next);
std::array<double,3> normalize3(std::array<double,3> v);
std::array<double,3> cross3(const std::array<double,3>& a,
                            const std::array<double,3>& b);
                            
                            
                            
                            
// ================= Global state & helpers =================
extern Params  g_par;          // parameters (dim=3)
extern Params  g_par7;         // parameters (dim=7)


void enforce_endpoints(std::vector<double>& P, const Params& par) ;
void ensure_samples_from_x(const std::vector<double>& x) ;
void maybe_build_forms(Params& par) ;
struct PerConCtx { int con_idx; };

double collision_con_7D_i(const std::vector<double>& x,
                              std::vector<double>& grad,
                              void* d);
double objective_7D(const std::vector<double>& x, std::vector<double>& grad, void* data) ;

void helical_escape_preproject(std::vector<double>& P, Params& par,
                               int outer_iters,
                               double base_radius,   
                               double radius_growth, 
                               double eta,            
                               double angle_step);
std::array<double,3> nearest_point3d_EDT(const std::shared_ptr<DynamicEDTOctomap>& edt,
                                         double x, double y, double z, double resolution);

std::array<double,3> nearest_point3d_pcl(Params& par,
                                         double x, double y, double z); 

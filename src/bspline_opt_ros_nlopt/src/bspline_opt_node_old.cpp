#include "bspline_core.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nlopt.hpp>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>


#include "utils.cpp"

#include <geometry_msgs/Pose.h>
#include <bspline_opt_ros/SetGoalPose.h> 

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <optional>

#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nav_msgs/OccupancyGrid.h>

#include <omni_firmware/FullPose.h>

int stride = 1;  // stride=2 → 10cm voxels, stride=4 → 20cm voxels if base=5cm
std::random_device rd;
std::mt19937 gen(rd());
std::normal_distribution<> noise(0.0, 0.000001);  // mean=0, stddev=0.01
std::atomic<bool> g_x0_ready{false};
std::mutex g_x0_mtx;
std::array<double,7> g_x0_full_buf{0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0}; // default until first msg

// ================== Globals ==================
Params  g_par;
Params  g_par7;


std::atomic<bool> g_goal_received(false);
std::atomic<uint64_t> g_goal_epoch{0};
std::mutex g_goal_mtx;
std::vector<double> g_xf_full(7, 0.0);

std::shared_ptr<octomap::OcTree>      g_tree;    // plain occupancy
std::shared_ptr<octomap::ColorOcTree> g_ctree;   // colored occupancy
bool   g_tree_ready = false;
double g_tree_res   = 0.10;
Eigen::Vector3d g_map_min(-1e9,-1e9,-1e9), g_map_max(1e9,1e9,1e9);
std::shared_ptr<DynamicEDTOctomap> g_edt;
bool start_plan = false;
bool service_called = false;
nav_msgs::OccupancyGridConstPtr g_grid;
bool g_have_grid = false;

// Forward decls (place above astar_on_octomap_any)
static std::vector<octomap::OcTreeKey> neighbors_any(const octomap::OcTreeKey& key, int connectivity);
static double h_key_any(const octomap::OcTreeKey& a, const octomap::OcTreeKey& b, double wz=2.0);
static bool losFreeInflated_any(const Eigen::Vector3d& A, const Eigen::Vector3d& B,
                                double inflation, double step_m, double occ_th=0.5);



inline void normalize_quat_inplace(std::array<double,4>& q) {
  double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 1e-12) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
  else { q = {0,0,0,1}; }
}




void x0Cb(const omni_firmware::FullPose::ConstPtr& msg)

{
  // Pull position + quaternion (qx,qy,qz,qw) from the message
  std::array<double,7> x0 = {
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  };

  // Keep your existing frame flip (remove if you don’t want it anymore)
  x0[0] = -x0[0];
  x0[1] = -x0[1];

  {
    std::lock_guard<std::mutex> lk(g_x0_mtx);
    g_x0_full_buf = x0;

    // Write into Params so the rest of your code uses it
    g_par.x0      = std::vector<double>(x0.begin(), x0.end());      // 7D
    g_par.x0_3D   = { x0[0], x0[1], x0[2] };                        // 3D
    g_par7.x0      = std::vector<double>(x0.begin(), x0.end());      // 7D
    g_par7.x0_3D   = { x0[0], x0[1], x0[2] };                        // 3D

  }

  g_x0_ready.store(true);
  /*ROS_INFO("x0 updated from /pose_full: (%.2f, %.2f, %.2f)",
           g_par.x0_3D[0], g_par.x0_3D[1], g_par.x0_3D[2]);*/
}

void gridMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    g_grid = msg;
    g_have_grid = true;
    /*ROS_INFO("Received occupancy grid: %d x %d (res=%.2f)", 
              msg->info.width, msg->info.height, msg->info.resolution);*/
}

void octomapCb(const octomap_msgs::OctomapConstPtr& msg)
{
  auto* abstract = octomap_msgs::msgToMap(*msg);
  if (!abstract) { ROS_WARN("octomap_msgs::msgToMap failed."); return; }

  // Reset both, then set the one we actually have
  g_tree.reset();
  g_ctree.reset();

  if (auto* t = dynamic_cast<octomap::OcTree*>(abstract)) {
    g_tree.reset(t);
    g_tree_res = g_tree->getResolution();

    double xmin,ymin,zmin,xmax,ymax,zmax;
    g_tree->getMetricMin(xmin,ymin,zmin);
    g_tree->getMetricMax(xmax,ymax,zmax);
    g_map_min = Eigen::Vector3d(xmin,ymin,zmin);
    g_map_max = Eigen::Vector3d(xmax,ymax,zmax);
    g_tree_ready = true;
    // Inside octomapCb(), after g_tree_ready = true:
if (g_tree) {
  ros::Time t1 = ros::Time::now();

    octomap::point3d minPt(g_map_min.x(), g_map_min.y(), g_map_min.z());
    octomap::point3d maxPt(g_map_max.x(), g_map_max.y(), g_map_max.z());
    float max_dist = 30.0f; // max distance to propagate, choose appropriately
    g_par7.edt = std::make_shared<DynamicEDTOctomap>(
    max_dist,
    g_tree.get(),
    minPt,
    maxPt,
    false  // treat unknown as unoccupied
);
g_par7.edt->update();   // <--- missing

//ROS_INFO("DynamicEDTOctomap built (maxdist=%.2f, res=%.3f)", max_dist, g_tree_res);
ros::Duration dt = ros::Time::now() - t1;
//ROS_INFO("DynamicEDTOctomap update took %.3f sec", dt.toSec());
}
    //ROS_INFO("OctoMap received: OcTree (res=%.3f).", g_tree_res);
    return;
  }

  if (auto* ct = dynamic_cast<octomap::ColorOcTree*>(abstract)) {
    g_ctree.reset(ct);
    g_tree_res = g_ctree->getResolution();
     g_tree_res = g_ctree->getResolution();

    // Convert to plain OcTree
    g_tree = std::make_shared<octomap::OcTree>(g_tree_res);
    for (auto it = g_ctree->begin(); it != g_ctree->end(); ++it) {
        if (g_ctree->isNodeOccupied(*it)) {
            g_tree->updateNode(it.getKey(), true);
        }
    }
    double xmin,ymin,zmin,xmax,ymax,zmax;
    g_tree->getMetricMin(xmin,ymin,zmin);
    g_tree->getMetricMax(xmax,ymax,zmax);
    g_map_min = Eigen::Vector3d(xmin,ymin,zmin);
    g_map_max = Eigen::Vector3d(xmax,ymax,zmax);
    g_tree_ready = true;
    // Inside octomapCb(), after g_tree_ready = true:
if (g_tree) {

  ros::Time t1 = ros::Time::now();
    octomap::point3d minPt(g_map_min.x(), g_map_min.y(), g_map_min.z());
    octomap::point3d maxPt(g_map_max.x(), g_map_max.y(), g_map_max.z());
    float max_dist = 30.0f; // max distance to propagate, choose appropriately
    g_par7.edt = std::make_shared<DynamicEDTOctomap>(
    max_dist,
    g_tree.get(),
    minPt,
    maxPt,
    false  // treat unknown as unoccupied
);
g_par7.edt->update();   // <--- missing

//ROS_INFO("DynamicEDTOctomap built (maxdist=%.2f, res=%.3f)", max_dist, g_tree_res);
ros::Duration dt = ros::Time::now() - t1;
//ROS_INFO("DynamicEDTOctomap update took %.3f sec", dt.toSec());
}
    //ROS_INFO("OctoMap received: ColorOcTree (res=%.3f).", g_tree_res);
    return;
  }

  // Unknown tree type — free the abstract tree to avoid leak
  std::string id = abstract->getTreeType();
  ROS_WARN("Unsupported OctoMap tree type: %s", id.c_str());
  delete abstract;
}

// Weighted nearest-to-goal / furthest-from-x0 (2D) with EDT clearance at z_target.
// J = w_goal * ||c - goal_xy|| - w_far * ||c - x0_xy||  --> minimize J
std::optional<Eigen::Vector3d> nearestSafeSubgoal(
    const Eigen::Vector3d& goal3D,
    double d_tol)
{
    // --- Weights (tune as you like) ---
    const double w_goal = 0.5;  // bias toward goal
    const double w_far  = 1 - w_goal;  // bias away from current position
    // Optional gate to keep choices relevant & fast (meters). Set to INF to disable.
    const double goal_gate_R = std::numeric_limits<double>::infinity();

    // --- Map metadata ---
    const int    width  = g_grid->info.width;
    const int    height = g_grid->info.height;
    const double res    = g_grid->info.resolution;
    const double ox     = g_grid->info.origin.position.x;
    const double oy     = g_grid->info.origin.position.y;

    // --- Targets ---
    const double z_target = goal3D.z();
    const Eigen::Vector2d goal_xy(goal3D.x(), goal3D.y());

    // --- Current position (from std::vector<double>) ---
    if (g_par7.x0_3D.size() < 3) return std::nullopt;
    const Eigen::Map<const Eigen::Vector3d> x0_3d(g_par7.x0_3D.data());
    const Eigen::Vector2d x0_xy(x0_3d.x(), x0_3d.y());

    // --- World<->Grid helpers ---
    auto cellCenterXY = [&](int cx, int cy) -> Eigen::Vector2d {
        return { ox + (cx + 0.5) * res, oy + (cy + 0.5) * res };
    };

    // --- Scan grid once and keep arg-min of weighted J ---
    bool found = false;
    double bestJ = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_xy(0.0, 0.0);

    for (int cy = 0; cy < height; ++cy) {
        const int row = cy * width;
        for (int cx = 0; cx < width; ++cx) {
            const int idx = row + cx;
            const int val = g_grid->data[idx];       // 0=free, 100=occ, -1=unknown
            if (val != 0) continue;                  // only consider free cells

            const Eigen::Vector2d cxy = cellCenterXY(cx, cy);

            // (Optional) keep search within radius of goal for relevance/perf
            if (goal_gate_R < std::numeric_limits<double>::infinity()) {
                if ((cxy - goal_xy).norm() > goal_gate_R) continue;
            }

            // EDT clearance at z_target
            const octomap::point3d c3(cxy.x(), cxy.y(), z_target);
            const double clearance = g_par7.edt->getDistance(c3);
            if (clearance < d_tol) continue;

            // Weighted objective: smaller is better
            const double d_goal = (cxy - goal_xy).norm();
            const double d_x0   = (cxy - x0_xy).norm();
            const double J      = w_goal * d_goal - w_far * d_x0;

            if (J < bestJ) {
                bestJ  = J;
                best_xy = cxy;
                found  = true;
            }
        }
    }

    if (!found) return std::nullopt;

    // Return the chosen 3D subgoal at z_target
    return Eigen::Vector3d(best_xy.x(), best_xy.y(), z_target);
}

bool setGoalCb(bspline_opt_ros::SetGoalPose::Request& req,
               bspline_opt_ros::SetGoalPose::Response& res)
{
  // Build 7D goal from service request
  double gx = req.pose.position.x;
  double gy = req.pose.position.y;
  double gz = req.pose.position.z;

  std::array<double,4> q = {
    req.pose.orientation.x,
    req.pose.orientation.y,
    req.pose.orientation.z,
    req.pose.orientation.w
  };
  normalize_quat_inplace(q);

  gx = -gx;
  gy = -gy;
  Eigen::Vector3d global_goal;
  global_goal << gx, gy, gz;
  auto local_goal = nearestSafeSubgoal(global_goal,g_par.d_safe_z);
  Eigen::Vector3d gl;
  gl = *local_goal;
  gx = gl[0];
  gy = gl[1];
  gz = gl[2];
  
  // Write into globals safely
  {
    std::lock_guard<std::mutex> lk(g_goal_mtx);
    g_par.xf_3D = { gx, gy, gz };
    g_par.xf    = { gx, gy, gz, q[0], q[1], q[2], q[3] };

    // Keep 7D struct in sync
    g_par7.xf_3D = g_par.xf_3D;
    g_par7.xf    = g_par.xf;

    g_goal_received.store(true);
    g_goal_epoch.fetch_add(1, std::memory_order_relaxed);
  }

  // Kick the planner loop
  start_plan = true;
  service_called = true;
  res.accepted = true;
  std::ostringstream oss;
  oss << "xf set to [" << -gx << ", " << -gy << ", " << gz << "] "
      << "quat [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]";
  res.message = oss.str();
  ROS_INFO("%s", res.message.c_str());
  return true;
}

inline bool haveTree() { return (bool)g_tree || (bool)g_ctree; }

inline bool coordToKeyChecked_any(const octomap::point3d& p, octomap::OcTreeKey& k){
  if (g_tree)  return g_tree->coordToKeyChecked(p, k);
  if (g_ctree) return g_ctree->coordToKeyChecked(p, k);
  return false;
}
inline octomap::point3d keyToCoord_any(const octomap::OcTreeKey& k){
  return g_tree ? g_tree->keyToCoord(k) : g_ctree->keyToCoord(k);
}
inline double res_any(){
  return g_tree ? g_tree->getResolution() : (g_ctree ? g_ctree->getResolution() : 0.1);
}
inline bool isOccAt_any(const octomap::OcTreeKey& key, double occ_th=0.5){
  if (g_tree)  { auto* n=g_tree->search(key);  return n && g_tree->isNodeOccupied(n)  && n->getOccupancy()>=occ_th; }
  auto* n=g_ctree->search(key);                 return n && g_ctree->isNodeOccupied(n) && n->getOccupancy()>=occ_th;
}
inline bool isInflatedFree_any(const octomap::OcTreeKey& key, double inflation, double occ_th=0.5){
  const int r = std::max(0, (int)std::ceil(inflation / std::max(1e-6, res_any() * stride)));
  for (int dz=-r; dz<=r; ++dz)
    for (int dy=-r; dy<=r; ++dy)
      for (int dx=-r; dx<=r; ++dx) {
        octomap::OcTreeKey kk(key.k[0]+dx, key.k[1]+dy, key.k[2]+dz);
        if (isOccAt_any(kk, occ_th)) return false;
      }
  return true;
}
static std::vector<Eigen::Vector3d> astar_on_octomap_with_edt(
    const Eigen::Vector3d& start_w,
    const Eigen::Vector3d& goal_w,
    int connectivity, int max_expand, double z_weight,
    double inflation, double bbox_margin)
{
  if (!haveTree()) return {};

  // clamp to known map bounds (expanded)
  Eigen::Vector3d wmin = g_map_min - Eigen::Vector3d::Constant(bbox_margin);
  Eigen::Vector3d wmax = g_map_max + Eigen::Vector3d::Constant(bbox_margin);
  auto clampPt = [&](const Eigen::Vector3d& p){ return p.cwiseMax(wmin).cwiseMin(wmax); };

  Eigen::Vector3d sW=clampPt(start_w), gW=clampPt(goal_w);
  octomap::OcTreeKey sK,gK;
  if (!coordToKeyChecked_any(octomap::point3d(sW.x(),sW.y(),sW.z()), sK)) return {};
  if (!coordToKeyChecked_any(octomap::point3d(gW.x(),gW.y(),gW.z()), gK)) return {};

  // nudge into free if needed
  auto nearestFree = [&](octomap::OcTreeKey seed)->std::optional<octomap::OcTreeKey>{
    std::queue<octomap::OcTreeKey> q; std::unordered_set<uint64_t> vis;
    auto pack=[&](const octomap::OcTreeKey& k){ return ( (uint64_t)k.k[0]<<42 ) ^ ( (uint64_t)k.k[1]<<21 ) ^ (uint64_t)k.k[2]; };
    q.push(seed); vis.insert(pack(seed));
    int iters=0;
    while(!q.empty() && iters<300000){
      auto cur=q.front(); q.pop(); ++iters;
      octomap::point3d c = keyToCoord_any(cur);
      bool free_ok=false;
      if (g_par7.edt){ // O(1) distance query
        free_ok = g_par7.edt->getDistance(c) >= inflation;
      } else {
        free_ok = isInflatedFree_any(cur, inflation);
      }
      if ((c.x()>=wmin.x() && c.y()>=wmin.y() && c.z()>=wmin.z() &&
           c.x()<=wmax.x() && c.y()<=wmax.y() && c.z()<=wmax.z()) &&
          free_ok)
        return cur;
      for (auto& nb : neighbors_any(cur, 26)){
        uint64_t h=pack(nb); if (vis.count(h)) continue; vis.insert(h); q.push(nb);
      }
    }
    return std::nullopt;
  };
  
  auto s_ok=nearestFree(sK), g_ok=nearestFree(gK);
  if (!s_ok) ROS_WARN("Start not in free space (after inflation/EDT).");
  if (!g_ok) ROS_WARN("Goal not in free space (after inflation/EDT).");
  if (!s_ok || !g_ok) return {};
  sK=*s_ok; gK=*g_ok;

  struct Node{ double f,g; octomap::OcTreeKey k, parent; bool has_parent=false; };
  struct Cmp{ bool operator()(const Node&a,const Node&b)const{ return a.f>b.f; } };
  std::priority_queue<Node,std::vector<Node>,Cmp> open;
  std::unordered_map<uint64_t,double> best_g;
  std::unordered_map<uint64_t,octomap::OcTreeKey> parent;
  auto pack=[&](const octomap::OcTreeKey& k){ return ( (uint64_t)k.k[0]<<42 ) ^ ( (uint64_t)k.k[1]<<21 ) ^ (uint64_t)k.k[2]; };

  Node sN; sN.k=sK; sN.g=0.0; sN.f=h_key_any(sK,gK,z_weight); open.push(sN); best_g[pack(sK)]=0.0;

  int expands=0; bool found=false;
  while(!open.empty()){
    Node cur=open.top(); open.pop();
    if (cur.k.k[0]==gK.k[0] && cur.k.k[1]==gK.k[1] && cur.k.k[2]==gK.k[2]) { found=true; break; }
    if (++expands > max_expand) break;

    for (auto& nb : neighbors_any(cur.k, connectivity)){
      octomap::point3d c = keyToCoord_any(nb);
      if (!(c.x()>=wmin.x() && c.y()>=wmin.y() && c.z()>=wmin.z() &&
            c.x()<=wmax.x() && c.y()<=wmax.y() && c.z()<=wmax.z())) continue;

      // ensure neighbor is in search bounds
      if (!(c.x() >= g_par.lb3[0] && c.x() <= g_par.ub3[0] &&
            c.y() >= g_par.lb3[1] && c.y() <= g_par.ub3[1] &&
            c.z() >= g_par.lb3[2] && c.z() <= g_par.ub3[2])) continue;

      // EDT clearance check
      bool ok=false;
      if (g_par7.edt){
        double dist = g_par7.edt->getDistance(c);
        //if (dist < 30){
        //std::cout << c.x() << ", " << c.y() << ", " << c.z() << ": dist: " << dist  << std::endl;}
        ok = dist >= inflation;
      } else {
        ok = isInflatedFree_any(nb, inflation);
      }
      if (!ok) continue;

      double step = h_key_any(cur.k, nb, z_weight);
      double gn = cur.g + step;
      uint64_t h=pack(nb);
      auto it=best_g.find(h);
      if (it==best_g.end() || gn < it->second - 1e-12){
        best_g[h]=gn; parent[h]=cur.k;
        Node nx; nx.k=nb; nx.g=gn; nx.f=gn + h_key_any(nb,gK,z_weight); nx.parent=cur.k; nx.has_parent=true;
        open.push(nx);
      }
    }
  }
  if (!found) return {};

  // backtrack
  std::vector<octomap::OcTreeKey> chain; chain.push_back(gK);
  for (octomap::OcTreeKey cur=gK; !(cur.k[0]==sK.k[0] && cur.k[1]==sK.k[1] && cur.k[2]==sK.k[2]); ){
    auto it = parent.find(pack(cur)); if (it==parent.end()) break; cur = it->second; chain.push_back(cur);
  }
  std::reverse(chain.begin(), chain.end());

  // to world
  std::vector<Eigen::Vector3d> poly; poly.reserve(chain.size());
  for (auto& k : chain){ auto c = keyToCoord_any(k); poly.emplace_back(c.x(),c.y(),c.z()); }
  return poly;
}

static std::vector<Eigen::Vector3d> astar_on_octomap_any(
    const Eigen::Vector3d& start_w,
    const Eigen::Vector3d& goal_w,
    int connectivity, int max_expand, double z_weight,
    double inflation, double bbox_margin)
{
  if (!haveTree()) return {};

  // clamp to known map bounds (expanded)
  Eigen::Vector3d wmin = g_map_min - Eigen::Vector3d::Constant(bbox_margin);
  Eigen::Vector3d wmax = g_map_max + Eigen::Vector3d::Constant(bbox_margin);
  auto clampPt = [&](const Eigen::Vector3d& p){ return p.cwiseMax(wmin).cwiseMin(wmax); };

  Eigen::Vector3d sW=clampPt(start_w), gW=clampPt(goal_w);
  octomap::OcTreeKey sK,gK;
  if (!coordToKeyChecked_any(octomap::point3d(sW.x(),sW.y(),sW.z()), sK)) return {};
  if (!coordToKeyChecked_any(octomap::point3d(gW.x(),gW.y(),gW.z()), gK)) return {};

  // nudge into free if needed
  auto nearestFree = [&](octomap::OcTreeKey seed)->std::optional<octomap::OcTreeKey>{
    std::queue<octomap::OcTreeKey> q; std::unordered_set<uint64_t> vis;
    auto pack=[&](const octomap::OcTreeKey& k){ return ( (uint64_t)k.k[0]<<42 ) ^ ( (uint64_t)k.k[1]<<21 ) ^ (uint64_t)k.k[2]; };
    q.push(seed); vis.insert(pack(seed));
    int iters=0;
    while(!q.empty() && iters<300000){
      auto cur=q.front(); q.pop(); ++iters;
      octomap::point3d c = keyToCoord_any(cur);
      if ((c.x()>=wmin.x() && c.y()>=wmin.y() && c.z()>=wmin.z() &&
           c.x()<=wmax.x() && c.y()<=wmax.y() && c.z()<=wmax.z()) &&
          isInflatedFree_any(cur, inflation))
        return cur;
      for (auto& nb : neighbors_any(cur, 26)){
        uint64_t h=pack(nb); if (vis.count(h)) continue; vis.insert(h); q.push(nb);
      }
    }
    return std::nullopt;
  };
  
  auto s_ok=nearestFree(sK), g_ok=nearestFree(gK);
  if (!s_ok) ROS_WARN("Start not in free space (after inflation).");
  if (!g_ok) ROS_WARN("Goal not in free space (after inflation).");
  if (!s_ok || !g_ok) return {};
  sK=*s_ok; gK=*g_ok;

  struct Node{ double f,g; octomap::OcTreeKey k, parent; bool has_parent=false; };
  struct Cmp{ bool operator()(const Node&a,const Node&b)const{ return a.f>b.f; } };
  std::priority_queue<Node,std::vector<Node>,Cmp> open;
  std::unordered_map<uint64_t,double> best_g;
  std::unordered_map<uint64_t,octomap::OcTreeKey> parent;
  auto pack=[&](const octomap::OcTreeKey& k){ return ( (uint64_t)k.k[0]<<42 ) ^ ( (uint64_t)k.k[1]<<21 ) ^ (uint64_t)k.k[2]; };

  Node sN; sN.k=sK; sN.g=0.0; sN.f=h_key_any(sK,gK,z_weight); open.push(sN); best_g[pack(sK)]=0.0;

  int expands=0; bool found=false;
  while(!open.empty()){
    Node cur=open.top(); open.pop();
    //std::cout << cur.k.k[0] << " " << cur.k.k[1] << " " << cur.k.k[2] << std::endl;
    if (cur.k.k[0]==gK.k[0] && cur.k.k[1]==gK.k[1] && cur.k.k[2]==gK.k[2]) { found=true; break; }
    if (++expands > max_expand) break;

    for (auto& nb : neighbors_any(cur.k, connectivity)){
      octomap::point3d c = keyToCoord_any(nb);
      if (!(c.x()>=wmin.x() && c.y()>=wmin.y() && c.z()>=wmin.z() &&
            c.x()<=wmax.x() && c.y()<=wmax.y() && c.z()<=wmax.z())) continue;


      // ensure neighbor is in search bounds
      if (!(c.x() >= g_par.lb3[0] && c.x() <= g_par.ub3[0] &&
        c.y() >= g_par.lb3[1] && c.y() <= g_par.ub3[1] &&
        c.z() >= g_par.lb3[2] && c.z() <= g_par.ub3[2])) continue;
      if (!isInflatedFree_any(nb, inflation)) continue;

      double step = h_key_any(cur.k, nb, z_weight);
      double gn = cur.g + step;
      uint64_t h=pack(nb);
      auto it=best_g.find(h);
      if (it==best_g.end() || gn < it->second - 1e-12){
        best_g[h]=gn; parent[h]=cur.k;
        Node nx; nx.k=nb; nx.g=gn; nx.f=gn + h_key_any(nb,gK,z_weight); nx.parent=cur.k; nx.has_parent=true;
        open.push(nx);
      }
    }
  }
  if (!found) return {};

  // backtrack
  std::vector<octomap::OcTreeKey> chain; chain.push_back(gK);
  for (octomap::OcTreeKey cur=gK; !(cur.k[0]==sK.k[0] && cur.k[1]==sK.k[1] && cur.k[2]==sK.k[2]); ){
    auto it = parent.find(pack(cur)); if (it==parent.end()) break; cur = it->second; chain.push_back(cur);
  }
  std::reverse(chain.begin(), chain.end());

  // to world
  std::vector<Eigen::Vector3d> poly; poly.reserve(chain.size());
  for (auto& k : chain){ auto c = keyToCoord_any(k); poly.emplace_back(c.x(),c.y(),c.z()); }
  return poly;
}

static std::vector<Eigen::Vector3d> shortcut_octomap_any(const std::vector<Eigen::Vector3d>& P,
                                                         double inflation, double step){
  if (P.size()<=2) return P;
  std::vector<Eigen::Vector3d> Q; size_t i=0;
  while(i<P.size()-1){
    size_t j=P.size()-1;
    for (; j>i+1; --j) if (losFreeInflated_any(P[i], P[j], inflation, step)) break;
    Q.push_back(P[i]); i=j;
  }
  Q.push_back(P.back()); return Q;
}

static std::vector<octomap::OcTreeKey> neighbors_any(const octomap::OcTreeKey& key, int connectivity){
  std::vector<octomap::OcTreeKey> out; out.reserve(26);
  for (int dz=-1; dz<=1; ++dz)
    for (int dy=-1; dy<=1; ++dy)
      for (int dx=-1; dx<=1; ++dx) {
        if (dx==0 && dy==0 && dz==0) continue;
        int man = std::abs(dx)+std::abs(dy)+std::abs(dz);
        if ((connectivity==6  && man!=1) ||
            (connectivity==18 && man>2)) continue;
out.emplace_back(key.k[0]+dx*stride,
                 key.k[1]+dy*stride,
                 key.k[2]+dz*stride);      }
  return out;
}

// remove "=1.0" here
static double h_key_any(const octomap::OcTreeKey& a,
                        const octomap::OcTreeKey& b,
                        double wz) {
double dx=(a.k[0]-b.k[0]) * stride;
double dy=(a.k[1]-b.k[1]) * stride;
double dz=(a.k[2]-b.k[2]) * stride * wz;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}

// remove "=0.5" here
static bool losFreeInflated_any(const Eigen::Vector3d& A,
                                const Eigen::Vector3d& B,
                                double inflation,
                                double step_m,
                                double occ_th) {
  Eigen::Vector3d d = B - A; double L = d.norm();
  if (L < 1e-9) return true;
  Eigen::Vector3d u = d / L;
  for (double s=0.0; s<=L; s+=step_m){
    Eigen::Vector3d p = A + s*u;
    octomap::OcTreeKey k;
    if (!coordToKeyChecked_any(octomap::point3d(p.x(),p.y(),p.z()), k)) return false;
    if (!isInflatedFree_any(k, inflation, occ_th)) return false;
  }
  return true;
}


static std::vector<Eigen::Vector3d> resample_by_arclen(const std::vector<Eigen::Vector3d>& P, int M){
  if (P.empty()) return {};
  if ((int)P.size()==1) return std::vector<Eigen::Vector3d>(M, P[0]);
  std::vector<double> s(P.size(),0.0);
  for (size_t i=1;i<P.size();++i) s[i]=s[i-1]+(P[i]-P[i-1]).norm();
  double L=s.back(); if (L<1e-9) return std::vector<Eigen::Vector3d>(M,P.front());
  std::vector<Eigen::Vector3d> R; R.reserve(M);
  for (int k=0;k<M;++k){
    double tk=(M==1)?0.0:(double)k/(M-1); double target=tk*L;
    size_t j=1; while(j<s.size() && s[j]<target) ++j;
    if (j>=s.size()) { R.push_back(P.back()); continue; }
    double t=(target-s[j-1])/std::max(1e-12, s[j]-s[j-1]);
    R.push_back((1.0-t)*P[j-1]+t*P[j]);
  }
  return R;
}

static void ctrl_from_polyline(const std::vector<Eigen::Vector3d>& poly, int N, std::vector<double>& x3){
  auto R = resample_by_arclen(poly, N);
  for (int i=0;i<N;++i){ x3[i*3+0]=R[i].x(); x3[i*3+1]=R[i].y(); x3[i*3+2]=R[i].z(); }
}


static double eval_gi_row_7D(int i, const std::vector<double>& x, std::vector<double>* grad_opt) {
  PerConCtx pc; pc.con_idx = i;

    if (grad_opt) {
        grad_opt->assign(x.size(), 0.0);
        return collision_con_7D_i(x, *grad_opt, &pc);
    } else {
        std::vector<double> dummy;
        return collision_con_7D_i(x, dummy, &pc);
    }
}

// ====== Per-component FD check for quaternion gradients ======
static void check_fd_quat_percomp_7D(int i,
                                     const Params& par7,
                                     std::vector<double> x,
                                     double eps = 1e-6)
{
    const int dim = par7.dim;   // should be 7
    const int ncp = par7.num_ctrl_pts;
    if (dim != 7 || (int)x.size() != ncp*dim) {
        std::cerr << "[FD] bad sizes\n";
        return;
    }
    if (i < 0 || i >= par7.g_lastS.n_rows) {
        std::cerr << "[FD] row i out of range\n";
        return;
    }

    // Analytic gradient
    std::vector<double> grad;
    double gi0 = eval_gi_row_7D(i, x, &grad);

    /*std::cout << "\n[FD] Checking per-component quaternion gradients at row " << i
              << " (gi=" << gi0 << ")\n";
    std::cout << "ctrl\tcomp\tanalytic\tFD\tabs diff\trel diff\n";
    */
    // Loop over all interior control points
    for (int ci = 1; ci < ncp-1; ++ci) {
        for (int j = 3; j <= 6; ++j) { // quaternion components qx,qy,qz,qw
            int idx = ci*dim + j;

            // central difference
            auto xp = x, xm = x;
            xp[idx] += eps;
            xm[idx] -= eps;
            double gip = eval_gi_row_7D(i, xp, nullptr);
            double gim = eval_gi_row_7D(i, xm, nullptr);
            double fd = (gip - gim) / (2.0 * eps);

            double ga = grad[idx];
            double ad = std::abs(ga - fd);
            double rd = (std::abs(fd) > 1e-14) ? ad/std::abs(fd) : NAN;

            /*std::cout << ci << "\tq" << (j-2)   // label q1,q2,q3,q4
                      << "\t" << ga
                      << "\t" << fd
                      << "\t" << ad
                      << "\t" << rd << "\n";*/
        }
    }
}
// ================= Main (looped optimization with SLSQP + bounds) =================
int main(int argc, char** argv) {
  ros::init(argc, argv, "bspline_opt_node");
  ros::NodeHandle nh("~");

  // ---- Load params (3D) ----
  g_par.dim = 3;
  nh.param("num_ctrl_pts", g_par.num_ctrl_pts, 10);
  nh.param("N_samples",    g_par.N_samples,    20);

  nh.param("C_len",        g_par.C_len,        0.1);
  nh.param("C_goal",       g_par.C_goal,       8.0);

  nh.param("C_geom",      g_par.C_speed,          0.1);
  nh.param("C_acc",        g_par.C_acc,            0.02);
  nh.param("C_jerk",       g_par.C_jerk,           0.005);

  nh.param("d_safe_xy", g_par.d_safe_xy, 0.7);
  nh.param( "d_safe_z",  g_par.d_safe_z, 0.5);

 // A* params
  int connectivity, max_expand; double inflation, z_weight, shortcut_step, bbox_margin;
  nh.param("astar/connectivity", connectivity, 26);
  nh.param("astar/max_expand",   max_expand,   2000000);
  nh.param("astar/inflation",    inflation,    0.4);
  nh.param("astar/z_weight",     z_weight,     1.0);
  nh.param("astar/shortcut_step",shortcut_step,0.05);
  nh.param("astar/bbox_margin",  bbox_margin,  0.5);

  // Endpoints (truncate 7D → 3D)
  std::vector<double> x0_full, xf_full;
nh.getParam("x0", x0_full);
nh.getParam("xf", xf_full);
if ((int)x0_full.size() < 3 || (int)xf_full.size() < 3) {
  ROS_ERROR("x0/xf must have at least 3 elements each (YAML is 7D)");
  return 1;
}
xf_full[0] = -xf_full[0];
xf_full[1] = -xf_full[1];
g_par.x0_3D = std::vector<double>(x0_full.begin(), x0_full.begin()+3);
g_par.xf_3D = std::vector<double>(xf_full.begin(), xf_full.begin()+3);
g_par.x0 = std::vector<double>(x0_full.begin(), x0_full.begin()+x0_full.size());
g_par.xf = std::vector<double>(xf_full.begin(), xf_full.begin()+xf_full.size());

// Bounds (truncate lb7/ub7 → first 3 entries)
std::vector<double> lb7, ub7;
nh.getParam("lb7", lb7);
nh.getParam("ub7", ub7);
if ((int)lb7.size() < 3 || (int)ub7.size() < 3) {
  ROS_ERROR("lb7/ub7 must have at least 3 elements (YAML is 7D)");
  return 1;
}
g_par.lb7 = std::vector<double>(lb7.begin(), lb7.begin()+lb7.size());
g_par.ub7 = std::vector<double>(ub7.begin(), ub7.begin()+ub7.size());
for (int d=0; d<3; ++d) {
  g_par.lb3[d] = lb7[d];
  g_par.ub3[d] = ub7[d];
}

  // Publishers / subscribers
  ros::Publisher path_pub_init_visual  = nh.advertise<nav_msgs::Path>("visual_solution_path_init", 1, true);
  ros::Publisher path_pub_init  = nh.advertise<nav_msgs::Path>("solution_path_init", 1, true);

  ros::Publisher path_pub  = nh.advertise<nav_msgs::Path>("/solution_path", 1, true);
  ros::Publisher path_pub_visual  = nh.advertise<nav_msgs::Path>("/viual_solution_path", 1, true);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 1, true);
  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
    "/rtabmap/octomap_obstacles", 1, boost::bind(&cloudCallback, _1, &g_par, &cloud_pub));
  ros::Publisher dist_pub = nh.advertise<std_msgs::Float32MultiArray>("/path_distances",1,true);
  ros::Subscriber octo_sub =
    nh.subscribe<octomap_msgs::Octomap>("/rtabmap/octomap_binary",
                                        1, octomapCb);
                                      
  ros::Subscriber x0_sub = nh.subscribe<omni_firmware::FullPose>("/pose_full", 1, x0Cb);
  ros::Subscriber grid_sub = nh.subscribe("/rtabmap/octomap_grid", 1, gridMapCallback);
  
  ros::ServiceServer set_goal_srv =
  nh.advertiseService("set_goal", setGoalCb);

  g_par.dim = 3;
  // Initial controls: straight line between x0 and xf
  const int nd    = g_par.dim;
  const int n_ctrl_3 = g_par.num_ctrl_pts * nd;
  g_par7 = g_par;
  g_par7.dim = 7;
  const int n_ctrl_7 = g_par7.num_ctrl_pts * g_par7.dim;

 // one slack per sampled row
int n_slack = g_par7.N_samples * (g_par7.num_ctrl_pts - 3); // typical #rows
// better: use g_par7.g_lastS.n_rows after sampling
int nvars = n_ctrl_7 + g_par7.g_lastS.n_rows;

  std::vector<double> x(n_ctrl_3, 0.0);
  for (int i=0; i<g_par.num_ctrl_pts; ++i) {
    double s = double(i) / double(g_par.num_ctrl_pts - 1);
    for (int d=0; d<3; ++d) x[i*nd + d] = (1.0 - s)*g_par.x0_3D[d] + s*g_par.xf_3D[d] + noise(gen);
  }


  // Bounds (fix endpoints exactly; interior bounded by lb3/ub3)
  std::vector<double> lb(n_ctrl_3, -1e9), ub(n_ctrl_3, +1e9);
  for (int i=1; i<g_par.num_ctrl_pts-1; ++i) {
    for (int d=0; d<3; ++d) {
      lb[i*nd + d] = g_par.lb3[d];
      ub[i*nd + d] = g_par.ub3[d];
      lb[i*nd + d] = g_par.lb3[d];
      ub[i*nd + d] = g_par.ub3[d];
    }
  }
  bool slacks = false;
  if (slacks){nvars = n_ctrl_7;}
  std::vector<double> lb7_ctrl(nvars, -1e9);
  std::vector<double> ub7_ctrl(nvars, +1e9); 

  //for (int d=0; d<3; ++d) {
  //  lb[0*nd + d] = ub[0*nd + d] = g_par.x0_3D[d];
  //  lb[(g_par.num_ctrl_pts-1)*nd + d] = ub[(g_par.num_ctrl_pts-1)*nd + d] = g_par.xf_3D[d];
  //}

  // --- Expand into full-length bounds for each control point ---  
  if (slacks){
  // bounds for slack vars (≥ 0)
for (int i=n_ctrl_7; i<nvars; ++i) {
    lb7_ctrl[i] = 0.0;
    ub7_ctrl[i] = 1e9;
}} 
  for (int i=1; i<g_par7.num_ctrl_pts-1; ++i) {
    for (int d=0; d<7; ++d) {
      lb7_ctrl[i*7 + d] = lb7[d];
      ub7_ctrl[i*7 + d] = ub7[d];
   }
  }
  

  // Fix endpoints exactly to x0/xf
  //for (int d=0; d<7; ++d) {
  //  lb7_ctrl[0*7 + d] = ub7_ctrl[0*7 + d] = g_par.x0[d];
  //  lb7_ctrl[(g_par7.num_ctrl_pts-1)*7 + d] =
  //    ub7_ctrl[(g_par7.num_ctrl_pts-1)*7 + d] = g_par.xf[d];
  //}

   // Optimizer: SLSQP (gradient-based SQP)
  //nlopt::opt opt3(nlopt::LD_SLSQP, n_ctrl_3);
  //opt3.set_lower_bounds(lb);
  //opt3.set_upper_bounds(ub);
  //opt3.set_min_objective(objective_3D, &g_par);

  // Termination criteria
  //opt3.set_xtol_rel(1e-5);
  //opt3.set_ftol_rel(1e-5);
  //opt3.set_maxeval(2000);

  // Optimizer: SLSQP (gradient-based SQP)
  nlopt::opt local(nlopt::LD_SLSQP, nvars);
  nlopt::opt opt7(nlopt::AUGLAG, nvars);
  opt7.set_local_optimizer(local);
  //nlopt::opt opt7(nlopt::LD_SLSQP, n_ctrl_7);

  opt7.set_lower_bounds(lb7_ctrl);
  opt7.set_upper_bounds(ub7_ctrl);

  opt7.set_min_objective(objective_7D, &g_par7);

  // Termination criteria
  opt7.set_xtol_rel(1e-5);
  opt7.set_ftol_rel(1e-5);
  opt7.set_maxeval(2000);

  // Allocate constraint contexts once; we rebind each loop
  int max_constraints = g_par.num_ctrl_pts * g_par.N_samples; // safe upper bound
  std::vector<PerConCtx> cctx_3D(max_constraints);
  std::vector<PerConCtx> cctx_7D(max_constraints);

  ros::Rate loop_rate(1); // optimize/publish ~1 Hz
  int sequence_counter = 0;
    std::vector<double> x_initial = x;

////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  while (ros::ok()) {
    ros::spinOnce();
    if (!g_x0_ready.load() && service_called) {
      loop_rate.sleep();
      continue;
    }

    if (!start_plan) {
      loop_rate.sleep();
      continue;
    }

    if (start_plan){
      if (g_par.points.size() > 10){
  
      const int nd = g_par.dim;            // 3
    std::vector<double> new_x(n_ctrl_3, 0.0);
    if (haveTree()){

      Eigen::Vector3d p0(g_par.x0_3D[0], g_par.x0_3D[1], g_par.x0_3D[2]);
      Eigen::Vector3d pf(g_par.xf_3D[0], g_par.xf_3D[1], g_par.xf_3D[2]);
      auto poly = astar_on_octomap_with_edt(p0, pf, connectivity, max_expand, z_weight, inflation, bbox_margin);

      //std::cout << poly.size() << std::endl;
      if (!poly.empty()){
        poly = shortcut_octomap_any(poly, inflation, shortcut_step);
        ctrl_from_polyline(poly, g_par.num_ctrl_pts, new_x);
        for (int d=0; d<3; ++d){ new_x[d]=g_par.x0_3D[d]; new_x[(g_par.num_ctrl_pts-1)*3 + d]=g_par.xf_3D[d]; }
      } else {
        ROS_WARN("[A*] reinit: no path; using straight line.");
        for (int i=0;i<g_par.num_ctrl_pts;++i){
          double s=(double)i/(g_par.num_ctrl_pts-1);
          for (int d=0;d<3;++d) new_x[i*3+d]=(1-s)*g_par.x0_3D[d]+s*g_par.xf_3D[d];
        }
      }
    } 
  else {
    for (int i=0;i<g_par.num_ctrl_pts;++i){
      double s=(double)i/(g_par.num_ctrl_pts-1);
      for (int d=0;d<3;++d) new_x[i*3+d]=(1-s)*g_par.x0_3D[d]+s*g_par.xf_3D[d];
      ROS_WARN("No Octomap Tree found; using straight line.");
    }
  }
        //x_initial.swap(new_x);
        x_initial = new_x;

      nav_msgs::Path path_msg_init;
      nav_msgs::Path path_msg_init_visual;

      path_msg_init.header.stamp = ros::Time::now();
      path_msg_init.header.frame_id = "map";
      path_msg_init.header.seq = 0;
      std::vector<double> P_init = x_initial;
      Sampled Sopt_init = sample_with_map(P_init, g_par);


      path_msg_init_visual = path_msg_init;

      geometry_msgs::PoseStamped ps;
      for (int i=0; i<Sopt_init.n_rows; ++i) {
        ps.header = path_msg_init.header;
        ps.pose.position.x = -Sopt_init.T[i*nd + 0];
        ps.pose.position.y = -Sopt_init.T[i*nd + 1];
        ps.pose.position.z = Sopt_init.T[i*nd + 2];
        ps.pose.orientation.x = 0;
        ps.pose.orientation.y = 0;
        ps.pose.orientation.z = 0;
        ps.pose.orientation.w = 1;
        path_msg_init.poses.push_back(ps);
      }

      geometry_msgs::PoseStamped ps_vs;
      for (int i=0; i<Sopt_init.n_rows; ++i) {
        ps_vs.header = path_msg_init.header;
        ps_vs.pose.position.x = Sopt_init.T[i*nd + 0];
        ps_vs.pose.position.y = Sopt_init.T[i*nd + 1];
        ps_vs.pose.position.z = Sopt_init.T[i*nd + 2];
        ps_vs.pose.orientation.x = 0;
        ps_vs.pose.orientation.y = 0;
        ps_vs.pose.orientation.z = 0;
        ps_vs.pose.orientation.w = 1;
        path_msg_init_visual.poses.push_back(ps_vs);
      }
      path_pub_init.publish(path_msg_init);
      path_pub_init_visual.publish(path_msg_init_visual);


      // if not the last sample, use forward direction
      std::array<double,3> nearest_;
      double min_min_dist = 100000;
      double temp_dist;
      for (int i = 0; i < Sopt_init.n_rows; ++i) {
          double xw = Sopt_init.T[(i+1)*g_par7.dim + 0];
          double yw = Sopt_init.T[(i+1)*g_par7.dim + 1];
          double zw = Sopt_init.T[(i+1)*g_par7.dim + 2];

          nearest_ = nearest_point3d(g_par7.points, xw, yw, zw);
      
      double dx = xw - nearest_[0];
      double dy = yw - nearest_[1];
      double dz = zw - nearest_[2];
      double d2 = dx*dx + dy*dy + dz*dz;

      temp_dist = std::sqrt(d2);
      if (temp_dist < min_min_dist) {
          min_min_dist = temp_dist;
      }
      }
      ROS_INFO("Published A* path, with minimum distance %f.", min_min_dist);
    
      }
      else {
        ros::spinOnce();
        loop_rate.sleep();
        continue;
      }
      std::vector<double> Pw;
      double minf;
      
      // Compute ensured 3D path
      // Re-sample current x to know how many constraints to add
      {
        std::vector<double> P = x_initial;
        enforce_endpoints(P, g_par);
        g_par.g_lastS = sample_with_map(P, g_par);
        g_par.g_have_samples = true;
      }
      // Re-install per-sample collision constraints
      //opt3.remove_inequality_constraints();
      //for (int i=0; i<g_par.g_lastS.n_rows; ++i) {
      //  cctx_3D[i].con_idx = i;
      //  opt3.add_inequality_constraint(collision_con_3D_i, &cctx_3D[i], 1e-8);
      //}

      // Optimize with warm start
      minf = 0.0;
      g_par7.points = g_par.points;

 
      nav_msgs::Path path_msg_full;
      nav_msgs::Path path_msg_full_visual;

      path_msg_full.header.stamp = ros::Time::now();
      path_msg_full.header.frame_id = "map";
      path_msg_full.header.seq = sequence_counter;
      x = x_initial;
      path_msg_full_visual = path_msg_full;
      // ==================   Compute ensured 7D path ==================
      // Compute initial solution 7D
      // Build 7D control vector from 3D path + orientation interpolation
      const int n_ctrl7 = g_par7.num_ctrl_pts * g_par7.dim;
      std::vector<double> x7(n_ctrl7, 0.0);

      // Extract quaternion from 7D state arrays
      Quat q0 = q_from_array(&g_par.x0[3]); // qx,qy,qz,qw start at index 3
      Quat qf = q_from_array(&g_par.xf[3]);


      // Fill all control points
      for (int i=0; i<g_par7.num_ctrl_pts; ++i) {
        double s = double(i) / double(g_par7.num_ctrl_pts - 1);

        // Position from 3D optimized path
        x7[i*7 + 0] = x[i*3 + 0];
        x7[i*7 + 1] = x[i*3 + 1];
        x7[i*7 + 2] = x[i*3 + 2];

        // Orientation interpolation
        Quat qi = q_slerp(q0, qf, s);
        q_to_array(qi, &x7[i*7 + 3]); // writes qx,qy,qz,qw into x7
      }
      g_par7.x_init = x7;
      {
        std::vector<double> P = x7;
        enforce_endpoints(P, g_par7);
        g_par7.g_lastS = sample_with_map(P, g_par7);
        g_par7.g_have_samples = true;
      }
      // Re-install per-sample collision constraints
      opt7.remove_inequality_constraints();
      for (int i=0; i<g_par7.g_lastS.n_rows; ++i) {
        cctx_7D[i].con_idx = i;
        opt7.add_inequality_constraint(collision_con_7D_i, &cctx_7D[i], 1e-8);
      }
      //int test_row = std::max(0, g_par7.g_lastS.n_rows/2);
      //check_fd_quat_percomp_7D(test_row, g_par7, x7);   // Optimize with warm start
      minf = 0.0;
      try {
        nlopt::result res = opt7.optimize(x7, minf);
        ROS_INFO_STREAM("NLopt (SLSQP) result=" << res << ", f=" << minf);
      } catch (std::exception& e) {
        ROS_WARN_STREAM("NLopt exception: " << e.what());
      }

      g_par7.resolution = 0.05;
      // Sample final path and publish with sequence
      Pw = x7;
      enforce_endpoints(Pw, g_par7);
      Sampled Sopt_7D = sample_with_map(Pw, g_par7);

      path_msg_full.poses.reserve(Sopt_7D.n_rows);

      
      geometry_msgs::PoseStamped ps_7D;
      // Publish 7D full path (pos + quat)
      for (int i = 0; i < Sopt_7D.n_rows; ++i) {
        ps_7D.header = path_msg_full.header;
        ps_7D.pose.position.x = -Sopt_7D.T[i*7 + 0];
        ps_7D.pose.position.y = -Sopt_7D.T[i*7 + 1];
        ps_7D.pose.position.z = Sopt_7D.T[i*7 + 2];
        ps_7D.pose.orientation.x = Sopt_7D.T[i*7 + 3];
        ps_7D.pose.orientation.y = Sopt_7D.T[i*7 + 4];
        ps_7D.pose.orientation.z = Sopt_7D.T[i*7 + 5];
        ps_7D.pose.orientation.w = Sopt_7D.T[i*7 + 6];
        path_msg_full.poses.push_back(ps_7D);
      }

      geometry_msgs::PoseStamped ps_7D_visual;
      // Publish 7D full path (pos + quat)
      for (int i = 0; i < Sopt_7D.n_rows; ++i) {
        ps_7D_visual.header = path_msg_full.header;
        ps_7D_visual.pose.position.x = Sopt_7D.T[i*7 + 0];
        ps_7D_visual.pose.position.y = Sopt_7D.T[i*7 + 1];
        ps_7D_visual.pose.position.z = Sopt_7D.T[i*7 + 2];
        ps_7D_visual.pose.orientation.x = Sopt_7D.T[i*7 + 3];
        ps_7D_visual.pose.orientation.y = Sopt_7D.T[i*7 + 4];
        ps_7D_visual.pose.orientation.z = Sopt_7D.T[i*7 + 5];
        ps_7D_visual.pose.orientation.w = Sopt_7D.T[i*7 + 6];
        path_msg_full_visual.poses.push_back(ps_7D_visual);
      }

      // ---- Compute per-point distances using nearest_point3d ----
      const double inv_dxy = 1.0 / std::max(1e-9, g_par7.d_safe_xy);
      const double inv_dz  = 1.0 / std::max(1e-9, g_par7.d_safe_z);

  
                                         
      std_msgs::Float32MultiArray dist_msg;
      dist_msg.data.resize(Sopt_7D.n_rows);
      double min_distance = 10000000;
      for (int i = 0; i < Sopt_7D.n_rows; ++i) {
        double xw = Sopt_7D.T[i*g_par7.dim + 0];
        double yw = Sopt_7D.T[i*g_par7.dim + 1];
        double zw = Sopt_7D.T[i*g_par7.dim + 2];
        const double* row = &Sopt_7D.T[i*g_par7.dim];
        Quat q = q_from_array(&row[3]);

      // if not the last sample, use forward direction
      std::array<double,3> nearest;
      if (i+1 < Sopt_7D.n_rows) {
          double x_next = Sopt_7D.T[(i+1)*g_par7.dim + 0];
          double y_next = Sopt_7D.T[(i+1)*g_par7.dim + 1];
          double z_next = Sopt_7D.T[(i+1)*g_par7.dim + 2];

          nearest = nearest_point3d_forward(g_par7.points, xw, yw, zw,
                                            x_next, y_next, z_next);
      } else {
          // last point → just use global nearest
          nearest = nearest_point3d(g_par7.points, xw, yw, zw);
      }
      // remove this later
      std::array<double,3> di_world{ xw - nearest[0], yw - nearest[1], zw - nearest[2] };
      const auto di_body = rotate_by_quat(q, di_world);
      //std::cout << "hello 3" << std::endl;

      const double sx = inv_dxy * di_body[0];
      const double sy = inv_dxy * di_body[1];
      const double sz = inv_dz  * di_body[2];
      const double dist2 = sx*sx + sy*sy + sz*sz;
      std::cout << "g: " << 1- dist2 << std::endl;
      // remove this later
      
      double dx = xw - nearest[0];
      double dy = yw - nearest[1];
      double dz = zw - nearest[2];
      double d2 = dx*dx + dy*dy + dz*dz;

      dist_msg.data[i] = std::sqrt(d2);
      if (dist_msg.data[i] < min_distance) {
          min_distance = dist_msg.data[i];
      }
      }

      // publish alongside path
      dist_msg.layout.dim.resize(1);
      dist_msg.layout.dim[0].label = "path_points";
      dist_msg.layout.dim[0].size = Sopt_7D.n_rows;
      dist_msg.layout.dim[0].stride = Sopt_7D.n_rows;

      dist_pub.publish(dist_msg);

      path_pub.publish(path_msg_full);
      path_pub_visual.publish(path_msg_full_visual);
      start_plan = false;
      service_called = false;
      ROS_INFO("Published optimized path, with minimum distance %f.", min_distance);
      sequence_counter++;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


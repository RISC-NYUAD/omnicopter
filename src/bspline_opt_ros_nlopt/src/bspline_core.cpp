#include "bspline_core.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros/ros.h>
#include "utils.cpp"

#include <iomanip>
#include <limits>


// Global cloud + kd-tree
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::ConstPtr g_cloud_const;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdtree;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                   Params* par,
                   ros::Publisher* cloud_pub) {
  par->points.clear();
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    par->points.push_back({*iter_x, *iter_y, *iter_z});
  }
  //ROS_INFO_STREAM("Received cloud with " << par->points.size() << " points.");
  cloud_pub->publish(*msg);

// Convert ROS message to pcl::PointCloud
  g_cloud->clear();
  pcl::fromROSMsg(*msg, *g_cloud);
  
  // Update kd-tree
  g_kdtree.setInputCloud(g_cloud);
  //std::cout << "KdTree size = " << g_kdtree.getInputCloud()->size() << std::endl;

}
// ========================== Quadratic Gram matrices ==========================
Eigen::Matrix4d gram_speed() {
  Eigen::Matrix4d M;
  M <<  9.0/5,  -9.0/10, -3.0/5,  -3.0/10,
       -9.0/10,  6.0/5,   3.0/10, -3.0/5,
       -3.0/5,   3.0/10,  6.0/5,  -9.0/10,
       -3.0/10, -3.0/5,  -9.0/10,  9.0/5;
  return M;
}

Eigen::Matrix4d gram_acc() {
  Eigen::Matrix4d M;
  M << 12, -18,  0,  6,
      -18, 36, -18,  0,
        0,-18, 36, -18,
        6,  0, -18, 12;
  return M;
}

Eigen::Matrix4d gram_jerk() {
  Eigen::Matrix4d M;
  M <<  36, -108,  108, -36,
      -108,  324, -324, 108,
       108, -324,  324,-108,
       -36,  108, -108,  36;
  return M;
}

QuadForm assemble_form(const Params& par,
                       const std::vector<std::array<int,4>>& segments,
                       const Eigen::Matrix4d& M)
{
  const int n = par.num_ctrl_pts * par.dim;
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(segments.size()*4*4*3); // rough

  for (auto seg : segments) {
    for (int a=0;a<4;++a){
      for (int b=0;b<4;++b){
        double val = M(a,b);
        for (int d=0; d<3; ++d){
          int ia = seg[a]*par.dim + d;
          int ib = seg[b]*par.dim + d;
          triplets.emplace_back(ia, ib, val);
        }
      }
    }
  }

  QuadForm Q;
  Q.nvars = n;
  Q.H.resize(n,n);
  Q.H.setFromTriplets(triplets.begin(), triplets.end());
  return Q;
}

// ========================== Segments ==========================
std::vector<std::array<int,4>> build_segments(int num_ctrl_pts) {
  std::vector<std::array<int,4>> segs;
  if (num_ctrl_pts < 4) return segs;
  for (int s = 0; s + 3 < num_ctrl_pts; s += 3) {
    segs.push_back({s, s+1, s+2, s+3});
    if (s+3 == num_ctrl_pts - 1) break;
  }
  if (!segs.empty()) {
    auto &last = segs.back();
    if (last[3] != num_ctrl_pts-1)
      segs.push_back({num_ctrl_pts-4, num_ctrl_pts-3, num_ctrl_pts-2, num_ctrl_pts-1});
  }
  return segs;
}

void cubic_basis(double t, double b[4]) {
  const double u = 1.0 - t;
  b[0] = u*u*u;
  b[1] = 3.0*u*u*t;
  b[2] = 3.0*u*t*t;
  b[3] = t*t*t;
}

// ========================== Sampling ==========================
Sampled sample_with_map(const std::vector<double>& P, const Params& par) {
  Sampled S;
  const auto segs = build_segments(par.num_ctrl_pts);
  const int n_segments = (int)segs.size();
  S.n_rows = n_segments * par.N_samples;
  S.T.assign(S.n_rows * 7, 0.0);
  S.map.resize(S.n_rows);
  S.rpy.resize(S.n_rows);   // allocate storage for RPY
  uint nd;
  if (par.dim >3) nd = 7; else nd = 3;
  // initial orientation from x0 (already quaternion)
  Quat q_prev = q_from_array(&par.x0[3]);
  //std::cout << q_prev.x << " " << q_prev.y << " " << q_prev.z << " " << q_prev.w << std::endl; 

  // keep last delta RPY (start with zeros)
  double dR_prev = 0.0, dP_prev = 0.0, dY_prev = 0.0;

  int row = 0;
  for (int s = 0; s < n_segments; ++s) {
    const auto &id = segs[s];
    for (int k = 0; k < par.N_samples; ++k) {
      const double t = (par.N_samples==1) ? 0.0 : double(k)/double(par.N_samples-1);
      double b[4]; cubic_basis(t, b);

      // --- Position ---
      for (int d=0; d<3; ++d) {
        double p = 0.0;
        p += b[0]*P[id[0]*par.dim + d];
        p += b[1]*P[id[1]*par.dim + d];
        p += b[2]*P[id[2]*par.dim + d];
        p += b[3]*P[id[3]*par.dim + d];
        S.T[row*nd + d] = p;
      }
      if (par.dim >3){
      // --- Orientation delta (absolute from spline) ---
      double dR = 0.0, dP = 0.0, dY = 0.0;
      for (int kk=0; kk<4; ++kk) {
        dR += b[kk]*P[id[kk]*par.dim + 3];
        dP += b[kk]*P[id[kk]*par.dim + 4];
        dY += b[kk]*P[id[kk]*par.dim + 5];
      }

      // incremental change = current - previous
      double dR_inc = dR - dR_prev;
      double dP_inc = dP - dP_prev;
      double dY_inc = dY - dY_prev;

      // update quaternion
      Quat dq = quat_from_rpy(dR_inc, dP_inc, dY_inc);
      Quat q  = q_mul(q_prev, dq);
      //q_normalize(q);

      // save result
      q_to_array(q, &S.T[row*nd + 3]);

      // save rpy (absolute deltas, not increments)
      S.rpy[row] = {dR+par.RPY0[0], dP+par.RPY0[1], dY+par.RPY0[2]};
      // update trackers
      q_prev = q;
      dR_prev = dR;
      dP_prev = dP;
      dY_prev = dY;
      } else {
        q_to_array(q_prev, &S.T[row*nd + 3]);
      }
      // bookkeeping
      S.map[row].seg = s;
      for (int kk=0; kk<4; ++kk) {
        S.map[row].cidx[kk] = id[kk];
        S.map[row].w[kk]    = b[kk];
      }

      ++row;
    }
  }
  return S;
}

// ========================== Utilities ==========================
std::array<double,3> nearest_point3d(const std::vector<std::array<double,3>>& pts,
                                     double x, double y, double z) {
  double best = std::numeric_limits<double>::infinity();
  std::array<double,3> best_p{0,0,0};
  for (const auto& p : pts) {
    const double dx = x - p[0], dy = y - p[1], dz = z - p[2];
    const double d2 = dx*dx + dy*dy + dz*dz;
    if (d2 < best) { best = d2; best_p = p; }
  }
  return best_p;
}
std::array<double,3> nearest_point3d_pcl(Params& par,
                                         double x, double y, double z) 
{
//par.kdtree.setInputCloud(par.cloud);
    pcl::PointXYZ searchPoint(x, y, z);

    std::vector<int> indices(1);
    std::vector<float> sqr_distances(1);
    if (g_kdtree.nearestKSearch(searchPoint, 1, indices, sqr_distances) > 0) {

        const auto& p = (*g_cloud)[indices[0]];
        return {p.x, p.y, p.z};
    }

    
    // Fallback: no result
    return {0.0, 0.0, 0.0};
}


// Return the nearest occupied voxel center
std::array<double,3> nearest_point3d_EDT(const std::shared_ptr<DynamicEDTOctomap>& edt,
                                         double x, double y, double z, double resolution) {
     octomap::point3d query(x, y, z);
    float dist = edt->getDistance(query);

    if (dist <= 1e-6) {
        // already in an occupied voxel
        return {x, y, z};
    }


    // current position
    double cx = x, cy = y, cz = z;

    // get resolution from the underlying OcTree
    double step = resolution;

    while (dist > step * 0.5) {
        float bestDist = dist;
        double bestx = cx, besty = cy, bestz = cz;

        // check 26 neighbors in continuous space
        for (int dx=-1; dx<=1; ++dx) {
            for (int dy=-1; dy<=1; ++dy) {
                for (int dz=-1; dz<=1; ++dz) {
                    if (dx==0 && dy==0 && dz==0) continue;

                    double nx = cx + dx*step;
                    double ny = cy + dy*step;
                    double nz = cz + dz*step;

                    float nd = edt->getDistance(octomap::point3d(nx, ny, nz));
                    if (nd < bestDist) {
                        bestDist = nd;
                        bestx = nx; besty = ny; bestz = nz;
                    }
                }
            }
        }

        if (bestDist >= dist) {
            // no improvement
            break;
        }

        cx = bestx; cy = besty; cz = bestz;
        dist = bestDist;
    }

    return {cx, cy, cz};
}

std::array<double,3> nearest_point3d_forward(const std::vector<std::array<double,3>>& pts,
                                     double x,double y,double z,
                                     double x_next,double y_next,double z_next){
  double best = std::numeric_limits<double>::infinity();
  std::array<double,3> best_p{0,0,0};
  // motion direction
    const double dxm = x_next - x;
    const double dym = y_next - y;
    const double dzm = z_next - z;

  for (const auto& p : pts) {
    
        const double dx = x - p[0];
        const double dy = y - p[1];
        const double dz = z - p[2];

        // check if obstacle is forward
        const double dot = dx*dxm + dy*dym + dz*dzm;
        if (dot <= 0.0) continue;  // obstacle behind, ignore

        const double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < best) {
            best = d2;
            best_p = p;
        }
  }
  return best_p;
}

std::array<double,3> normalize3(std::array<double,3> v){
  double n = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) + 1e-12;
  v[0]/=n; v[1]/=n; v[2]/=n; return v;
}

std::array<double,3> cross3(const std::array<double,3>& a,
                            const std::array<double,3>& b){
  return { a[1]*b[2]-a[2]*b[1],
           a[2]*b[0]-a[0]*b[2],
           a[0]*b[1]-a[1]*b[0] };
}



void enforce_endpoints(std::vector<double>& P, const Params& par) {
  for (int d=0; d<par.dim; ++d) {
    if (d < 3){
      P[0*par.dim + d] = par.x0[d];
      P[(par.num_ctrl_pts-1)*par.dim + d] = par.xf[d];} else {
      P[0*par.dim + d] = 0.0;
      }
  }
}

void ensure_samples_from_x(const std::vector<double>& x) {
  std::vector<double> P(x.begin(), x.end());
  enforce_endpoints(P, g_par);
  g_par.g_lastS = sample_with_map(P, g_par);
  g_par.g_have_samples = true;
}

void maybe_build_forms(Params& par) {
  if (par.forms_ready) return;
  const auto segs = build_segments(par.num_ctrl_pts);
  par.Qspeed = assemble_form(par, segs, gram_speed());
  par.Qacc   = assemble_form(par, segs, gram_acc());
  par.Qjerk  = assemble_form(par, segs, gram_jerk());
  par.forms_ready = true;
}


// ---- tiny helpers ----
static inline Eigen::Vector3d v3(double x, double y, double z) { return Eigen::Vector3d(x,y,z); }

static inline void orthonormal_basis_from_axis(const Eigen::Vector3d& axis,
                                               Eigen::Vector3d& u, Eigen::Vector3d& v)
{
  Eigen::Vector3d a = axis.normalized();
  Eigen::Vector3d tmp = (std::fabs(a.x()) < 0.9) ? v3(1,0,0) : v3(0,1,0);
  u = (tmp - tmp.dot(a)*a).normalized();
  v = a.cross(u);
}

// Nearest point in raw cloud (brute force; replace with your KD-tree if you have one)
static inline Eigen::Vector3d nearest_point3d_e(const std::vector<std::array<double,3>>& pts,
                                              const Eigen::Vector3d& p)
{
  double best = std::numeric_limits<double>::infinity();
  Eigen::Vector3d q(0,0,0);
  for (const auto& t : pts) {
    Eigen::Vector3d r(t[0],t[1],t[2]);
    double d2 = (r - p).squaredNorm();
    if (d2 < best) { best = d2; q = r; }
  }
  return q;
}

// Heuristic, spline-aware helical escape before optimization.
// P is control vector (length = num_ctrl_pts * 3) for a 3D path.
// par: uses par.x0_3D, par.xf_3D, par.lb3/ub3, par.num_ctrl_pts, par.N_samples, par.points, par.d_safe_xy/z.
void helical_escape_preproject(std::vector<double>& P, Params& par,
                               int outer_iters = 4,
                               double base_radius = 0.05,   // meters
                               double radius_growth = 0.05, // meters/turn
                               double eta = 0.6,            // control step size
                               double angle_step = M_PI/10) // ~18°
{
  const int nd = 3;
  const int n_ctrl = par.num_ctrl_pts * nd;

  // Axis to spiral around (global start->goal)
  Eigen::Vector3d axis = v3(par.xf_3D[0]-par.x0_3D[0],
                            par.xf_3D[1]-par.x0_3D[1],
                            par.xf_3D[2]-par.x0_3D[2]).normalized();
  Eigen::Vector3d U, V; orthonormal_basis_from_axis(axis, U, V);

  // Safety radius to enforce
  const double d_safe = std::max(par.d_safe_xy, par.d_safe_z) - 0.5;

  for (int it=0; it<outer_iters; ++it) {

    // 1) sample with map (so we know row -> ctrl weights)
    Sampled S = sample_with_map(P, par);
    const int R = S.n_rows;

    // 2) accumulate control updates
    std::vector<Eigen::Vector3d> dC(par.num_ctrl_pts, Eigen::Vector3d::Zero());
    std::vector<double> wsum(par.num_ctrl_pts, 0.0);

    // grow radius each outer iter; also phase-shift angle so we sweep directions
    const double radius = base_radius + it * radius_growth;
    const double theta0 = it * 0.37; // irrational-ish offset to avoid repeating directions

    for (int r=0; r<R; ++r) {
      const double* row = &S.T[r*nd];
      Eigen::Vector3d Pi(row[0], row[1], row[2]);

      // nearest obstacle
      if (par.points.empty()) continue; // nothing to avoid
      Eigen::Vector3d Q = nearest_point3d_e(par.points, Pi);
      const double dist = (Pi - Q).norm();

      if (dist >= d_safe) continue; // already safe

      // Build a helical offset in the plane orthogonal to axis
      // angle depends on row index so adjacent rows push in different directions
      const double theta = theta0 + r * angle_step;

      // magnitude: try to exceed the deficit to d_safe
      const double deficit = std::max(0.0, d_safe - dist);
      const double mag = std::max(deficit, radius); // at least radius, grows with iters

      Eigen::Vector3d delta = mag * (std::cos(theta)*U + std::sin(theta)*V);

      // Optionally bias by direct repulsion a bit:
      // delta += 0.3 * ( (Pi - Q).normalized() * deficit );

      // Push back to controls via 4 local B-spline weights
      const auto& rm = S.map[r];
      for (int k=0; k<4; ++k) {
        int c = rm.cidx[k];
        if (c<=0 || c>=par.num_ctrl_pts-1) continue; // keep endpoints fixed
        double w = rm.w[k];
        dC[c]   += (eta * w) * delta;
        wsum[c] += w;
      }
    }

    // 3) apply updates + clamp to bounds for interior controls
    for (int c=1; c<par.num_ctrl_pts-1; ++c) {
      if (wsum[c] <= 1e-12) continue;
      Eigen::Vector3d upd = dC[c] / wsum[c];
      P[c*nd+0] += upd.x();
      P[c*nd+1] += upd.y();
      P[c*nd+2] += upd.z();

      // bounds
      P[c*nd+0] = std::min(std::max(P[c*nd+0], par.lb3[0]), par.ub3[0]);
      P[c*nd+1] = std::min(std::max(P[c*nd+1], par.lb3[1]), par.ub3[1]);
      P[c*nd+2] = std::min(std::max(P[c*nd+2], par.lb3[2]), par.ub3[2]);
    }

    // 4) keep endpoints exact and loop
    enforce_endpoints(P, par);
  }
}


// ================= Objective with analytic gradient =================
double objective_7D(const std::vector<double>& x, std::vector<double>& grad, void* data) {
  (void)data; // using global state


  maybe_build_forms(g_par7);

  
  
  const int nd = g_par7.dim;
  const int nd7 = 7;
  const int n_ctrl = g_par7.num_ctrl_pts * nd;
  const int n_rows = g_par7.g_lastS.n_rows;
  const int n_slack = (int)x.size() - n_ctrl;         // one slack per row
  bool slacks = n_slack >0;

// Controls with endpoints enforced for sampling and smoothness
  std::vector<double> P(x.begin(), x.begin()+n_ctrl);
  enforce_endpoints(P, g_par7);
// Sample once per objective eval (cached for constraints)
  g_par7.g_lastS = sample_with_map(P, g_par7);
  g_par7.g_have_samples = true;


  // If gradient requested: prepare it to cover controls + slacks
  if (!grad.empty()) {
    grad.assign(x.size(), 0.0);  // IMPORTANT: zero everything once, including slack tail
  }
  // === Cost ===
  double J = 0.0;

  // 1) Closed-form smoothness: P^T (Σ w_k H_k) P
  //    Use sparse products: g += 2 * w_k * H_k * P
  Eigen::Map<const Eigen::VectorXd> Pv(P.data(), P.size());

  Eigen::VectorXd g_smooth = Eigen::VectorXd::Zero(P.size());
  if (g_par7.C_speed != 0.0 && g_par7.Qspeed.H.nonZeros() > 0) {
    J += g_par7.C_speed * (Pv.transpose() * (g_par7.Qspeed.H * Pv)).value();
    if (!grad.empty()) g_smooth += 2.0 * g_par7.C_speed * (g_par7.Qspeed.H * Pv);
  }
  if (g_par7.C_acc != 0.0 && g_par7.Qacc.H.nonZeros() > 0) {
    J += g_par7.C_acc * (Pv.transpose() * (g_par7.Qacc.H * Pv)).value();
    if (!grad.empty()) g_smooth += 2.0 * g_par7.C_acc * (g_par7.Qacc.H * Pv);
  }
  if (g_par7.C_jerk != 0.0 && g_par7.Qjerk.H.nonZeros() > 0) {
    J += g_par7.C_jerk * (Pv.transpose() * (g_par7.Qjerk.H * Pv)).value();
    if (!grad.empty()) g_smooth += 2.0 * g_par7.C_jerk * (g_par7.Qjerk.H * Pv);
  }

  // 2) (Optional) geometric length over samples already supported by your code
  //    Keep it via C_len to preserve prior behavior.
  if (g_par7.C_len != 0.0) {
    for (int i=0; i<n_rows-1; ++i) {
      const double* pi  = &g_par7.g_lastS.T[i*nd7];
      const double* pj  = &g_par7.g_lastS.T[(i+1)*nd7];
      const double dx = pj[0]-pi[0], dy = pj[1]-pi[1], dz = pj[2]-pi[2];
      J += g_par7.C_len * (dx*dx + dy*dy + dz*dz);
    }
  }

  // 3) Goal on last sample
  {
    const double* pend = &g_par7.g_lastS.T[(n_rows-1)*nd7];
    for (int d=0; d<3; ++d) {
      const double diff = pend[d] - g_par7.xf[d];
      J += g_par7.C_goal * diff*diff;
    }
  }

 // 4) Penalty for deviation from initial guess (only x,y,z of each CP)
if (!g_par7.x_init.empty() && g_par7.C_init != 0.0) {
  for (int i = 0; i < g_par7.num_ctrl_pts; ++i) {
    if (i == 0 || i == g_par7.num_ctrl_pts-1) continue; // skip fixed endpoints
    int base = i * nd;
    for (int d = 0; d < 3; ++d) {  // only x,y,z
      double diff = P[base + d] - g_par7.x_init[base + d];
      J += g_par7.C_init * diff * diff;
      if (!grad.empty()) {
        grad[base + d] += 2.0 * g_par7.C_init * diff;
      }
    }
  }
}

// 5) Orientation *delta* penalty (no goal orientation):
//    Encourage small per-CP deltas [dR,dP,dY] to regularize orientation changes.
if (g_par7.C_theta != 0.0) {
  double J_theta = 0.0;
  for (int i = 0; i < g_par7.num_ctrl_pts; ++i) {
    const int base = i * nd;
    const double dR = P[base + 3];
    const double dP = P[base + 4];
    const double dY = P[base + 5];
    J_theta += (dR * dR + dP * dP + dY * dY);
    if (!grad.empty()) {
      grad[base + 3] += 2.0 * g_par7.C_theta * dR;
      grad[base + 4] += 2.0 * g_par7.C_theta * dP;
      grad[base + 5] += 2.0 * g_par7.C_theta * dY;
    }
  }
  J += g_par7.C_theta * J_theta; // note: J_theta already sums squares; weight applied once here
}

    Eigen::VectorXd g_slack = Eigen::VectorXd::Zero(P.size());
if (slacks){
// 6) Slack penalty (one slack per row)
  for (int i=0;i<n_slack;++i){
    double z = x[n_ctrl + i];
    J += g_par7.C_slack * z * z;
    if (!grad.empty()) {
      grad[n_ctrl + i] += 2.0 * g_par7.C_slack * z;
    }
  } }
      
  // === Gradient ===
  if (!grad.empty()) {
    // (a) smoothness (already in control space)
    for (int i=0;i<n_ctrl;++i) grad[i] += g_smooth[i];

   

    // (b) geometric length term (sample-space chain to control space)
    if (g_par7.C_len != 0.0) {
      for (int i=0; i<n_rows-1; ++i) {
        const double* pi  = &g_par7.g_lastS.T[i*nd7];
        const double* pj  = &g_par7.g_lastS.T[(i+1)*nd7];
        const double gx = 2.0 * g_par7.C_len * (pj[0]-pi[0]);
        const double gy = 2.0 * g_par7.C_len * (pj[1]-pi[1]);
        const double gz = 2.0 * g_par7.C_len * (pj[2]-pi[2]);

        // sample i contribution: -[gx,gy,gz]
        const auto& rmi = g_par7.g_lastS.map[i];
        for (int k=0; k<4; ++k) {
          int ci = rmi.cidx[k]; double w = rmi.w[k];
          if (ci==0 || ci==(g_par7.num_ctrl_pts-1)) continue;
          grad[ci*nd+0] += -w * gx;
          grad[ci*nd+1] += -w * gy;
          grad[ci*nd+2] += -w * gz;
        }
        // sample i+1 contribution: +[gx,gy,gz]
        const auto& rmj = g_par7.g_lastS.map[i+1];
        for (int k=0; k<4; ++k) {
          int cj = rmj.cidx[k]; double w = rmj.w[k];
          if (cj==0 || cj==(g_par7.num_ctrl_pts-1)) continue;
          grad[cj*nd+0] += +w * gx;
          grad[cj*nd+1] += +w * gy;
          grad[cj*nd+2] += +w * gz;
        }
      }
    }


    // (c) goal on last sample (chain to controls)
    {
      const int i = n_rows-1;
      const double* pend = &g_par7.g_lastS.T[i*nd7];
      const double gx = 2.0 * g_par7.C_goal * (pend[0]-g_par7.xf[0]);
      const double gy = 2.0 * g_par7.C_goal * (pend[1]-g_par7.xf[1]);
      const double gz = 2.0 * g_par7.C_goal * (pend[2]-g_par7.xf[2]);

      const auto& rm = g_par7.g_lastS.map[i];
      for (int k=0; k<4; ++k) {
        int c = rm.cidx[k]; double w = rm.w[k];

        if (c==0 || c==(g_par7.num_ctrl_pts-1)) continue;
        grad[c*nd+0] += w * gx;
        grad[c*nd+1] += w * gy;
        grad[c*nd+2] += w * gz;
      }
    }

    // zero gradients at endpoints to respect fixed endpoints
    for (int d=0; d<nd; ++d) {
      grad[0*nd + d] = 0.0;
      if (d<3)
      grad[(g_par7.num_ctrl_pts-1)*nd + d] = 0.0;
    }

    //std::cout << "J:" << J << std::endl;
  
  }

  return J;
}
// ================= Collision constraint with analytic gradient =================
// ================= Collision constraint with softmin =================

double collision_con_7D_i(const std::vector<double>& x,
                          std::vector<double>& grad,
                          void* d)
{
  const double mult = 1;
  const int dim_ctrl = g_par7.dim;   // 6 : [x,y,z, dR,dP,dY]
  const int dim_samp = 7;            // 7 : [x,y,z, qx,qy,qz,qw]
  auto* pc = reinterpret_cast<PerConCtx*>(d);
  const int i = pc->con_idx;

  const int n_ctrl  = g_par7.num_ctrl_pts * dim_ctrl;
  const int n_slack = (int)x.size() - n_ctrl;
  const bool slacks = (n_slack > 0);

  // Controls for sampling: fix endpoint positions only
  std::vector<double> P(x.begin(), x.begin()+n_ctrl);
  for (int dd=0; dd<dim_ctrl; ++dd){
    if (dd < 3){
      P[0*dim_ctrl + dd] = g_par7.x0[dd];
      P[(g_par7.num_ctrl_pts-1)*dim_ctrl + dd] = g_par7.xf[dd];
    } else {
      P[0*dim_ctrl + dd] = 0.0;
    }
  }

  // Sample once: rows are [x y z qx qy qz qw], S.rpy[row] holds ABS R,P,Y
  Sampled S = sample_with_map(P, g_par7);

  // Scales
  const double ax = 1.0 / std::max(1e-9, g_par7.d_safe_xy);
  const double az = 1.0 / std::max(1e-9, g_par7.d_safe_z);
  const double Dx = ax*ax, Dy = ax*ax, Dz = az*az;

  // Row i data
  const double* row = &S.T[i*dim_samp];
  const double xw = row[0], yw = row[1], zw = row[2];
  const Quat   q  = q_from_array(&row[3]);
  const auto   rpy_i = S.rpy[i];
  
  // Nearest obstacle (world) and vectors
  const auto pn = nearest_point3d_pcl(g_par7, xw, yw, zw);
  const std::array<double,3> v{ xw - pn[0], yw - pn[1], zw - pn[2] };
  if (std::abs(v[0]) +std::abs(v[1]) +std::abs(v[2])  < 1e-12 ) const_cast<std::array<double,3>&>(v) = {1e-6, 0 ,0};

  // Rotate into body using SAME convention as value path
  const auto b = rotate_by_quat(q, v); // b = R(q) v

  // Value
  const double sx = ax * b[0], sy = ax * b[1], sz = az * b[2];
  const double dist2 = sx*sx + sy*sy + sz*sz;
  const double zeta = slacks ? x[n_ctrl + i] : 0.0;
  const double gi = mult*(1.0 - dist2 - zeta );
  if (grad.empty()) return gi;
  grad.assign(x.size(), 0.0);

  // For gradients: s = D * b
  const std::array<double,3> s{ Dx*b[0], Dy*b[1], Dz*b[2] };

  // ---------- Position gradient in world: ∂g/∂p_w = -2 R^T (D b) ----------
  // Rebuild R(q) (same as you had)
  const double xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
  const double xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
  const double wxq = q.w*q.x, wyq = q.w*q.y, wzq = q.w*q.z;
  const double r00 = 1.0 - 2.0*(yy + zz);
  const double r01 = 2.0*(xy - wzq);
  const double r02 = 2.0*(xz + wyq);
  const double r10 = 2.0*(xy + wzq);
  const double r11 = 1.0 - 2.0*(xx + zz);
  const double r12 = 2.0*(yz - wxq);
  const double r20 = 2.0*(xz - wyq);
  const double r21 = 2.0*(yz + wxq);
  const double r22 = 1.0 - 2.0*(xx + yy);

  const double gpx = -2.0 * ( r00*s[0] + r10*s[1] + r20*s[2] );
  const double gpy = -2.0 * ( r01*s[0] + r11*s[1] + r21*s[2] );
  const double gpz = -2.0 * ( r02*s[0] + r12*s[1] + r22*s[2] );

  const auto& rm_i = S.map[i];
  for (int k=0; k<4; ++k){
    const int    ci = rm_i.cidx[k];
    const double w   = rm_i.w[k];
    if (ci==0 || ci==(g_par7.num_ctrl_pts)) continue;
    grad[ci*dim_ctrl + 0] += mult*w * gpx;
    grad[ci*dim_ctrl + 1] += mult*w * gpy;
    grad[ci*dim_ctrl + 2] += mult*w * gpz;
  }

  // ---------------- ORIENTATION (LOCAL; VECTOR FORM) ----------------
  // Helpers
  auto cross = [](const std::array<double,3>& a, const std::array<double,3>& b){
    return std::array<double,3>{
      a[1]*b[2] - a[2]*b[1],
      a[2]*b[0] - a[0]*b[2],
      a[0]*b[1] - a[1]*b[0]
    };
  };
  auto dot = [](const std::array<double,3>& a, const std::array<double,3>& b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  };
  const std::array<double,3> qv{ q.x, q.y, q.z };

  // b = v + 2 q_w (q×v) + 2 q×(q×v)
  // ∂g/∂q_w  = -4 ((q×v)·s)
  const std::array<double,3> qxv = cross(qv, v);
  const double a_qw = -4.0 * dot(qxv, s);

  // ∂g/∂q_vec = -4 [ q_w (v×s) + v(q·s) + s(v·q) - 2 q (v·s) ]
  const std::array<double,3> vxs = cross(v, s);
  const double q_dot_s = dot(qv, s);
  const double v_dot_q = dot(v, qv);
  const double v_dot_s = dot(v, s);

  std::array<double,3> avec{
    q.w * vxs[0] + v[0]*q_dot_s + s[0]*v_dot_q - 2.0*qv[0]*v_dot_s,
    q.w * vxs[1] + v[1]*q_dot_s + s[1]*v_dot_q - 2.0*qv[1]*v_dot_s,
    q.w * vxs[2] + v[2]*q_dot_s + s[2]*v_dot_q - 2.0*qv[2]*v_dot_s
  };
  avec[0] *= -4.0; avec[1] *= -4.0; avec[2] *= -4.0;

  // Map to ABSOLUTE R,P,Y at this row via your dq_dRPY at (R,P,Y)
  Quat dq_dR_abs, dq_dP_abs, dq_dY_abs;
  dq_dRPY(rpy_i[0], rpy_i[1], rpy_i[2], dq_dR_abs, dq_dP_abs, dq_dY_abs);
  
 
  auto dot_q = [&](const Quat& dq_theta){
    return avec[0]*dq_theta.x + avec[1]*dq_theta.y + avec[2]*dq_theta.z + a_qw*dq_theta.w;
  };
  const double dgi_dR_abs = dot_q(dq_dR_abs);
  const double dgi_dP_abs = dot_q(dq_dP_abs);
  const double dgi_dY_abs = dot_q(dq_dY_abs);

  // Distribute to control ΔRPY via ROW weights ONLY (S.rpy[i] = Σ w_i(c) Δθ_c + RPY0)
  std::vector<double> w_i_vec(g_par7.num_ctrl_pts, 0.0);
  for (int k=0; k<4; ++k) w_i_vec[rm_i.cidx[k]] += rm_i.w[k];
  for (int c=0; c<g_par7.num_ctrl_pts; ++c){
    if (c==0 || c==(g_par7.num_ctrl_pts)) continue;
    const double w = w_i_vec[c];
    if (std::abs(w) < 1e-15) continue;
    grad[c*dim_ctrl + 3] += mult*w * dgi_dR_abs;
    grad[c*dim_ctrl + 4] += mult*w * dgi_dP_abs;
    grad[c*dim_ctrl + 5] += mult*w * dgi_dY_abs;
    
  }
  
  // Slack
  if (slacks) grad[n_ctrl + i] = -mult*1.0;

// --------------- (1) pos vs ang sensitivity imbalance ----------------
const double pos_n = std::sqrt(gpx*gpx + gpy*gpy + gpz*gpz);
const double ang_n = std::sqrt(dgi_dR_abs*dgi_dR_abs + dgi_dP_abs*dgi_dP_abs + dgi_dY_abs*dgi_dY_abs);

// Only report if the constraint is near active or violated
const double near_thr = 1e-4;
if (gi < 0.0 || std::abs(gi) < near_thr) {
    double ratio;
    if (pos_n > 0.0) ratio = ang_n / std::max(1e-16, pos_n);
    else             ratio = (ang_n > 0.0 ? 1e16 : 1.0);

    if (ratio < 1e-6 || ratio > 1e6) {
        std::fprintf(stderr,
            "[con#%d] pos/ang imbalance: pos=%.3e ang=%.3e ratio=%.3e (gi=%.2e)\n",
            i, pos_n, ang_n, ratio, gi);
    }
}

// --------------- (2) active-but-flat (zero usable gradient) ----------
if ((gi < 0.0 || std::abs(gi) < 1e-6) && (pos_n < 1e-12 && ang_n < 1e-12)) {
    std::fprintf(stderr,
        "[con#%d] active-but-flat: |gi|=%.3e (pos_n=%.3e ang_n=%.3e)\n",
        i, std::abs(gi), pos_n, ang_n);
}

// --------------- (3) zero interior weights (no way to push) ----------
double sum_w_interior = 0.0;
for (int k = 0; k < 4; ++k) {
    const int    ci = S.map[i].cidx[k];
    const double w  = S.map[i].w[k];
    if (ci > 0 && ci < g_par7.num_ctrl_pts - 1) sum_w_interior += w;
}
if ((gi < 0.0 || std::abs(gi) < near_thr) && std::abs(sum_w_interior) < 1e-12) {
    std::fprintf(stderr,
        "[con#%d] zero interior weights near active (gi=%.2e)\n",
        i, gi);
}

  return gi;
}

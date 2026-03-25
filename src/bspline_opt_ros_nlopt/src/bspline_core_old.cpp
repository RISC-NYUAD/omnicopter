#include "bspline_core.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros/ros.h>
#include "utils.cpp"


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
  S.T.assign(S.n_rows * par.dim, 0.0);
  S.map.resize(S.n_rows);
  Quat q0, q1, q2, q3;
  int row=0;
  for (int s=0; s<n_segments; ++s) {
    const auto &id = segs[s];
    if (par.dim == 7){
    q0 = q_from_array(&P[id[0]*par.dim + 3]);
    q1 = q_from_array(&P[id[1]*par.dim + 4]);
    q2 = q_from_array(&P[id[2]*par.dim + 5]);
    q3 = q_from_array(&P[id[3]*par.dim + 6]);}
    for (int k=0;k<par.N_samples;++k) {
      const double t = (par.N_samples==1) ? 0.0 : double(k)/double(par.N_samples-1);
      double b[4]; cubic_basis(t,b);
      for (int d=0; d<3; ++d) {
        double p = 0.0;
        p += b[0]*P[id[0]*par.dim + d];
        p += b[1]*P[id[1]*par.dim + d];
        p += b[2]*P[id[2]*par.dim + d];
        p += b[3]*P[id[3]*par.dim + d];
        S.T[row*par.dim + d] = p;
      }
      if (par.dim == 7){
      // orientation
      Quat q01 = q_slerp(q0,q1,t);
      Quat q12 = q_slerp(q1,q2,t);
      Quat q23 = q_slerp(q2,q3,t);
      Quat r1  = q_slerp(q01,q12,t);
      Quat r2  = q_slerp(q12,q23,t);
      Quat q   = q_slerp(r1,r2,t);
      q_to_array(q, &S.T[row*par.dim + 3]);}

      S.map[row].seg = s;
      for (int kk=0;kk<4;++kk){ 
        S.map[row].cidx[kk]=id[kk]; 
        S.map[row].w[kk]=b[kk]; 
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
    P[0*par.dim + d] = par.x0[d];
    if (d < 3)
      P[(par.num_ctrl_pts-1)*par.dim + d] = par.xf[d];
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
double objective_3D(const std::vector<double>& x, std::vector<double>& grad, void* data) {
  (void)data; // using global state

  maybe_build_forms(g_par);

  // Controls with endpoints enforced for sampling and smoothness
  std::vector<double> P(x.begin(), x.end());
  enforce_endpoints(P, g_par);

  // Sample once per objective eval (cached for constraints)
  g_par.g_lastS = sample_with_map(P, g_par);
  g_par.g_have_samples = true;
  g_par.g_lastS.closest_points.clear();
  for (int i = 0; i < g_par.g_lastS.n_rows; i++){
  	double* T = &g_par.g_lastS.T[i];
    	const auto p = nearest_point3d(g_par.points, T[0], T[1], T[2]);
    	g_par.g_lastS.closest_points.push_back({p[0],p[1],p[2]});
  }


 


  const int nd = g_par.dim; // 3
  const int n_ctrl = g_par.num_ctrl_pts * nd;
  const int n_rows = g_par.g_lastS.n_rows;

  // === Cost ===
  double J = 0.0;

  // 1) Closed-form smoothness: P^T (Σ w_k H_k) P
  //    Use sparse products: g += 2 * w_k * H_k * P
  Eigen::Map<const Eigen::VectorXd> Pv(P.data(), P.size());
  Eigen::VectorXd g_smooth = Eigen::VectorXd::Zero(P.size());

  if (g_par.C_speed != 0.0 && g_par.Qspeed.H.nonZeros() > 0) {
    J += g_par.C_speed * (Pv.transpose() * (g_par.Qspeed.H * Pv)).value();
    if (!grad.empty()) g_smooth += 2.0 * g_par.C_speed * (g_par.Qspeed.H * Pv);
  }
  if (g_par.C_acc != 0.0 && g_par.Qacc.H.nonZeros() > 0) {
    J += g_par.C_acc * (Pv.transpose() * (g_par.Qacc.H * Pv)).value();
    if (!grad.empty()) g_smooth += 2.0 * g_par.C_acc * (g_par.Qacc.H * Pv);
  }
  if (g_par.C_jerk != 0.0 && g_par.Qjerk.H.nonZeros() > 0) {
    J += g_par.C_jerk * (Pv.transpose() * (g_par.Qjerk.H * Pv)).value();
    if (!grad.empty()) g_smooth += 2.0 * g_par.C_jerk * (g_par.Qjerk.H * Pv);
  }

  // 2) (Optional) geometric length over samples already supported by your code
  //    Keep it via C_len to preserve prior behavior.
  if (g_par.C_len != 0.0) {
    for (int i=0; i<n_rows-1; ++i) {
      const double* pi  = &g_par.g_lastS.T[i*nd];
      const double* pj  = &g_par.g_lastS.T[(i+1)*nd];
      const double dx = pj[0]-pi[0], dy = pj[1]-pi[1], dz = pj[2]-pi[2];
      J += g_par.C_len * (dx*dx + dy*dy + dz*dz);
    }
  }

  // 3) Goal on last sample
  {
    const double* pend = &g_par.g_lastS.T[(n_rows-1)*nd];
    for (int d=0; d<3; ++d) {
      const double diff = pend[d] - g_par.xf[d];
      J += g_par.C_goal * diff*diff;
    }
  }

  // === Gradient ===
  if (!grad.empty()) {
    grad.assign(n_ctrl, 0.0);

    // (a) smoothness (already in control space)
    for (int i=0;i<n_ctrl;++i) grad[i] += g_smooth[i];

    // (b) geometric length term (sample-space chain to control space)
    if (g_par.C_len != 0.0) {
      for (int i=0; i<n_rows-1; ++i) {
        const double* pi  = &g_par.g_lastS.T[i*nd];
        const double* pj  = &g_par.g_lastS.T[(i+1)*nd];
        const double gx = 2.0 * g_par.C_len * (pj[0]-pi[0]);
        const double gy = 2.0 * g_par.C_len * (pj[1]-pi[1]);
        const double gz = 2.0 * g_par.C_len * (pj[2]-pi[2]);

        // sample i contribution: -[gx,gy,gz]
        const auto& rmi = g_par.g_lastS.map[i];
        for (int k=0; k<4; ++k) {
          int ci = rmi.cidx[k]; double w = rmi.w[k];
          if (ci==0 || ci==(g_par.num_ctrl_pts-1)) continue;
          grad[ci*nd+0] += -w * gx;
          grad[ci*nd+1] += -w * gy;
          grad[ci*nd+2] += -w * gz;
        }
        // sample i+1 contribution: +[gx,gy,gz]
        const auto& rmj = g_par.g_lastS.map[i+1];
        for (int k=0; k<4; ++k) {
          int cj = rmj.cidx[k]; double w = rmj.w[k];
          if (cj==0 || cj==(g_par.num_ctrl_pts-1)) continue;
          grad[cj*nd+0] += +w * gx;
          grad[cj*nd+1] += +w * gy;
          grad[cj*nd+2] += +w * gz;
        }
      }
    }

    // (c) goal on last sample (chain to controls)
    {
      const int i = n_rows-1;
      const double* pend = &g_par.g_lastS.T[i*nd];
      const double gx = 2.0 * g_par.C_goal * (pend[0]-g_par.xf[0]);
      const double gy = 2.0 * g_par.C_goal * (pend[1]-g_par.xf[1]);
      const double gz = 2.0 * g_par.C_goal * (pend[2]-g_par.xf[2]);

      const auto& rm = g_par.g_lastS.map[i];
      for (int k=0; k<4; ++k) {
        int c = rm.cidx[k]; double w = rm.w[k];
        if (c==0 || c==(g_par.num_ctrl_pts-1)) continue;
        grad[c*nd+0] += w * gx;
        grad[c*nd+1] += w * gy;
        grad[c*nd+2] += w * gz;
      }
    }

    // zero gradients at endpoints to respect fixed endpoints
    for (int d=0; d<nd; ++d) {
      grad[0*nd + d] = 0.0;
      grad[(g_par.num_ctrl_pts-1)*nd + d] = 0.0;
    }
  }
  return J;

}
// ================= Collision constraint with analytic gradient =================
// ================= Collision constraint with softmin =================

double collision_con_3D_i(const std::vector<double>& x,
                              std::vector<double>& grad,
                              void* d)
{
  const int nd = g_par.dim; // should be 3
  auto* pc = reinterpret_cast<PerConCtx*>(d);
  const int i = pc->con_idx;

  //if (!g_par.g_have_samples) {
    ensure_samples_from_x(x);
  //}
  if (i < 0 || i >= g_par.g_lastS.n_rows) {
    if (!grad.empty()) grad.assign(x.size(), 0.0);
    return 0.0;
  }

  const double* row = &g_par.g_lastS.T[i*nd];
  Eigen::Vector3d Pi(row[0], row[1], row[2]);

  // softmin over all obstacle points
  double alpha = 10.0; // smoothing factor; tune this
  std::vector<double> expvals;
  expvals.reserve(g_par.points.size());

  double maxexp = -1e20;
  for (const auto& p : g_par.points) {
  //for (const auto& p : g_par.g_lastS.closest_points) {
    Eigen::Vector3d pj(p[0], p[1], p[2]);
    double d2 = (Pi - pj).squaredNorm() + (g_par.d_safe_z-0.1);
    double val = -alpha * d2;
    if (val > maxexp) maxexp = val; // for numerical stability
    expvals.push_back(val);
  }
  /*
    for (int i=0; i< g_par.g_lastS.n_rows; i++) {
    double* T = &g_par.g_lastS.T[i];
    const auto p = nearest_point3d(g_par7.points, T[0], T[1], T[2]); 
    Eigen::Vector3d pj(p[0], p[1], p[2]);
    double d2 = (Pi - pj).squaredNorm() + (g_par.d_safe_z-0.1);
    double val = -alpha * d2;
    if (val > maxexp) maxexp = val; // for numerical stability
    expvals.push_back(val);
  }*/

  // compute softmin value
  double sumexp = 0.0;
  for (auto& v : expvals) sumexp += std::exp(v - maxexp);
  double logsumexp = maxexp + std::log(sumexp);
  double dsoft2 = -(1.0/alpha) * logsumexp;

  // constraint
  double gi = 1.0 - dsoft2;

  // gradient wrt controls
  if (!grad.empty()) {
    grad.assign(x.size(), 0.0);
    const int n_ctrl = g_par.num_ctrl_pts * nd;

    // weights w_j
    std::vector<double> w(g_par.points.size());
    double sumexp2 = 0.0;
    for (size_t j=0;j<g_par.points.size();++j) {
      w[j] = std::exp(expvals[j] - maxexp);
      sumexp2 += w[j];
    }
    for (size_t j=0;j<g_par.points.size();++j) w[j] /= sumexp2;

    // ∂dsoft2/∂Pi = sum_j w_j * ∂d_j^2/∂Pi
    Eigen::Vector3d gPi(0,0,0);
    for (size_t j=0;j<g_par.points.size();++j) {
      const auto& p = g_par.points[j];
      Eigen::Vector3d pj(p[0], p[1], p[2]);
      gPi += w[j] * 2.0*(Pi - pj);
    }
    // dgi/dPi = -∂dsoft2/∂Pi  std::cout << g_par.points.size() << std::endl;

    gPi *= -1.0;

    // chain rule to controls via Bezier weights
    const auto& rm = g_par.g_lastS.map[i];
    for (int k=0;k<4;++k) {
      int c = rm.cidx[k]; double wgt = rm.w[k];
      if (c==0 || c==(g_par.num_ctrl_pts-1)) continue;
      grad[c*nd+0] += wgt * gPi[0];
      grad[c*nd+1] += wgt * gPi[1];
      grad[c*nd+2] += wgt * gPi[2];
    }

    // zero endpoints
    for (int dpos=0; dpos<nd; ++dpos) {
      grad[0*nd + dpos] = 0.0;
      grad[(g_par.num_ctrl_pts-1)*nd + dpos] = 0.0;
    }
  }

  return gi;
}


// ================= Objective with analytic gradient =================
double objective_7D(const std::vector<double>& x, std::vector<double>& grad, void* data) {
  (void)data; // using global state

  maybe_build_forms(g_par7);

  
  
  const int nd = g_par7.dim; // 7
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
      const double* pi  = &g_par7.g_lastS.T[i*nd];
      const double* pj  = &g_par7.g_lastS.T[(i+1)*nd];
      const double dx = pj[0]-pi[0], dy = pj[1]-pi[1], dz = pj[2]-pi[2];
      J += g_par7.C_len * (dx*dx + dy*dy + dz*dz);
    }
  }

  // 3) Goal on last sample
  {
    const double* pend = &g_par7.g_lastS.T[(n_rows-1)*nd];
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


// quaternion penalties on control points (same logic as your original)
  auto unit_penalty = [&](const Quat& q)->std::pair<double, std::array<double,4>>{
    double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    double e = (n - 1.0);
    double J = e*e;
    double gcoef = (n>1e-12) ? (2.0*e / n) : 0.0;
    return {J, {gcoef*q.x, gcoef*q.y, gcoef*q.z, gcoef*q.w}};
  };
  auto ang2 = [&](Quat qa, Quat qb, std::array<double,4>& g_a, std::array<double,4>& g_b)->double{
    double d = q_dot(qa,qb);
    int sgn = 1;
    if (d < 0.0) { d = -d; sgn = -1; }
    d = std::min(1.0, std::max(0.0, d));
    double theta = 2.0*std::acos(d);
    double Jloc = theta*theta;
    double denom = std::sqrt(std::max(1e-12, 1.0 - d*d));
    double dJdd = 2.0*theta * (-2.0/denom);
    std::array<double,4> qb_eff{ double(sgn)*qb.x, double(sgn)*qb.y, double(sgn)*qb.z, double(sgn)*qb.w };
    g_a = { dJdd*qb_eff[0], dJdd*qb_eff[1], dJdd*qb_eff[2], dJdd*qb_eff[3] };
    g_b = { dJdd*qa.x, dJdd*qa.y, dJdd*qa.z, dJdd*qa.w };
    return Jloc;
  };
  // accumulate quaternion costs (no sampling)
  double J_qu = 0.0, J_qsmooth = 0.0;
  for (int i=0;i<g_par7.num_ctrl_pts;++i){
    Quat qi = q_from_array(&P[i*g_par7.dim + 3]);
    auto [Ju, gu] = unit_penalty(qi);
    J_qu += Ju;
    if (i+1<g_par7.num_ctrl_pts){
      Quat qj = q_from_array(&P[(i+1)*g_par7.dim + 3]);
      std::array<double,4> g_i{}, g_j{};
      double Js = ang2(qi,qj,g_i,g_j);
      J_qsmooth += Js;
    }
  }
  // goal orientation (last CP to xf)
  {
    Quat qf = q_from_array(&P[(g_par7.num_ctrl_pts-1)*g_par7.dim + 3]);
    Quat qg = q_from_array(&g_par7.xf[3]);
    std::array<double,4> g_f{}, g_dummy{};
    double Jg = ang2(qf,qg,g_f,g_dummy);
    J += g_par7.C_goal * Jg;
  }
  J += g_par7.C_quat*J_qu + g_par7.C_theta*J_qsmooth;  
  
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
    grad.assign(n_ctrl, 0.0);

    // (a) smoothness (already in control space)
    for (int i=0;i<n_ctrl;++i) grad[i] += g_smooth[i];

   

    // (b) geometric length term (sample-space chain to control space)
    if (g_par7.C_len != 0.0) {
      for (int i=0; i<n_rows-1; ++i) {
        const double* pi  = &g_par7.g_lastS.T[i*nd];
        const double* pj  = &g_par7.g_lastS.T[(i+1)*nd];
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
      const double* pend = &g_par7.g_lastS.T[i*nd];
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
// quat penalties (grad on interior controls only)
    for (int i=0;i<g_par7.num_ctrl_pts;++i){
      if (i==0 || i==g_par7.num_ctrl_pts-1) continue;
      Quat qi = q_from_array(&P[i*g_par7.dim + 3]);
      auto [Ju, gu] = unit_penalty(qi);
      grad[i*g_par7.dim + 3] += g_par7.C_quat * gu[0];
      grad[i*g_par7.dim + 4] += g_par7.C_quat * gu[1];
      grad[i*g_par7.dim + 5] += g_par7.C_quat * gu[2];
      grad[i*g_par7.dim + 6] += g_par7.C_quat * gu[3];
    }
    for (int i=0;i+1<g_par7.num_ctrl_pts;++i){
      std::array<double,4> gi{}, gj{};
      Quat qa = q_from_array(&P[i*g_par7.dim + 3]);
      Quat qb = q_from_array(&P[(i+1)*g_par7.dim + 3]);
      (void)ang2(qa,qb,gi,gj);
      if (i!=0 && i!=g_par7.num_ctrl_pts-1){
        grad[i*g_par7.dim + 3] += g_par7.C_theta * gi[0];
        grad[i*g_par7.dim + 4] += g_par7.C_theta * gi[1];
        grad[i*g_par7.dim + 5] += g_par7.C_theta * gi[2];
        grad[i*g_par7.dim + 6] += g_par7.C_theta * gi[3];
      }
      if ((i+1)!=0 && (i+1)!=g_par7.num_ctrl_pts-1){
        grad[(i+1)*g_par7.dim + 3] += g_par7.C_theta * gj[0];
        grad[(i+1)*g_par7.dim + 4] += g_par7.C_theta * gj[1];
        grad[(i+1)*g_par7.dim + 5] += g_par7.C_theta * gj[2];
        grad[(i+1)*g_par7.dim + 6] += g_par7.C_theta * gj[3];
      }
    }
    // zero gradients at endpoints to respect fixed endpoints
    for (int d=0; d<nd; ++d) {
      grad[0*nd + d] = 0.0;
      grad[(g_par7.num_ctrl_pts-1)*nd + d] = 0.0;
    }
  }
//std::cout << "J:" << J << std::endl;

  return J;
}
// ================= Collision constraint with analytic gradient =================
// ================= Collision constraint with softmin =================

double collision_con_7D_i(const std::vector<double>& x,
                              std::vector<double>& grad,
                              void* d)
{
  const int dim = g_par7.dim; // should be 7
  auto* pc = reinterpret_cast<PerConCtx*>(d);
  const int i = pc->con_idx;
  const int n_ctrl = g_par7.num_ctrl_pts * dim;          // <-- fixed
  const int n_slack = (int)x.size() - n_ctrl;
  bool slacks = n_slack >0;
  double alpha = 0.0;
//std::cout << "hello 0" << std::endl;

  // controls only
  std::vector<double> P(x.begin(), x.begin()+n_ctrl);
  for (int dd=0; dd<dim; ++dd){
    P[0*dim + dd] = g_par7.x0[dd];
    if (dd < 3)
      P[(g_par7.num_ctrl_pts-1)*dim + dd] = g_par7.xf[dd];
  }
//std::cout << "hello 1" << std::endl;
  // sample all (we use row i)
  auto S = sample_with_map(P, g_par7);
//std::cout << "hello 2" << std::endl;

  const double inv_dxy = 1.0 / std::max(1e-9, g_par7.d_safe_xy);
  const double inv_dz  = 1.0 / std::max(1e-9, g_par7.d_safe_z);

  const double* row = &S.T[i*dim];
  const double xw = row[0], yw = row[1], zw = row[2];
  Quat q = q_from_array(&row[3]);

  //const auto pn = nearest_point3d(g_par7.points, xw, yw, zw);
  const auto pn = nearest_point3d_pcl(g_par7,
                                         xw, yw, zw) ;
                                         
  std::array<double,3> di_world{ xw - pn[0], yw - pn[1], zw - pn[2] };
  const auto di_body = rotate_by_quat(q, di_world);
//std::cout << "hello 3" << std::endl;

  const double sx = inv_dxy * di_body[0];
  const double sy = inv_dxy * di_body[1];
  const double sz = inv_dz  * di_body[2];
  const double dist2 = sx*sx + sy*sy + sz*sz;

  // -------- slack --------
  const double zeta = slacks ? x[n_ctrl + i] : 0;
  double gi = 1.0 - dist2 - zeta; // <= 0
  // --- extra term: consecutive sample distance ---
  if (i+1 < S.n_rows) {
    const double* row_next = &S.T[(i+1)*dim];
    double dx = xw - row_next[0];
    double dy = yw - row_next[1];
    double dz = zw - row_next[2];
    double d_consec2 = dx*dx + dy*dy + dz*dz;
    gi -= alpha * d_consec2;  // add penalty
  }
  if (!grad.empty()){
    grad.assign(x.size(), 0.0);

    // derivative w.r.t. world position (through body-frame quadratic)
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

    const double wx = inv_dxy*inv_dxy, wy = inv_dxy*inv_dxy, wz = inv_dz*inv_dz;
    const double sdx = inv_dxy * di_body[0];
    const double sdy = inv_dxy * di_body[1];
    const double sdz = inv_dz  * di_body[2];
    const double vx = wx * sdx;
    const double vy = wy * sdy;
    const double vz = wz * sdz;
    // di_world, di_body already computed
    // W * di_body:
    const double Wdb_x = wx * di_body[0];
    const double Wdb_y = wy * di_body[1];
    const double Wdb_z = wz * di_body[2];

    // d(dist2)/d p_world
    const double gpx = 2.0 * ( r00*vx + r10*vy + r20*vz );
    const double gpy = 2.0 * ( r01*vx + r11*vy + r21*vz );
    const double gpz = 2.0 * ( r02*vx + r12*vy + r22*vz );

    // chain to control points via Bezier weights
    const auto& rm = S.map[i];
    for (int k=0;k<4;++k){
      int ci = rm.cidx[k];
      double w = rm.w[k];
      if (ci==0 || ci==(g_par7.num_ctrl_pts-1)) continue;
      int idx_x = ci*dim + 0;
      int idx_y = ci*dim + 1;
      int idx_z = ci*dim + 2;
      // gi = 1 - dist2 - zeta  => dgi/dp = - d(dist2)/dp
      grad[idx_x] += -w * gpx;
      grad[idx_y] += -w * gpy;
      grad[idx_z] += -w * gpz;
    }
    
    
        // gradient wrt consecutive point distance
    if (i+1 < S.n_rows) {
        const double* row_next = &S.T[(i+1)*dim];
        double dx = xw - row_next[0];
        double dy = yw - row_next[1];
        double dz = zw - row_next[2];
        double d_consec2 = dx*dx + dy*dy + dz*dz;

        // d/dP_i of (dx^2 + dy^2 + dz^2) = 2*(P_i - P_{i+1})
        double gpx_c = 2.0 * dx;
        double gpy_c = 2.0 * dy;
        double gpz_c = 2.0 * dz;

        const auto& rm = S.map[i];
        for (int k=0;k<4;++k){
            int ci = rm.cidx[k];
            double w = rm.w[k];
            if (ci==0 || ci==(g_par7.num_ctrl_pts-1)) continue;
            grad[ci*dim + 0] += -alpha * w * gpx_c;
            grad[ci*dim + 1] += -alpha * w * gpy_c;
            grad[ci*dim + 2] += -alpha * w * gpz_c;
        }

        // also propagate to P_{i+1} (negative sign)
        const auto& rm2 = S.map[i+1];
        for (int k=0;k<4;++k){
            int ci = rm2.cidx[k];
            double w = rm2.w[k];
            if (ci==0 || ci==(g_par7.num_ctrl_pts-1)) continue;
            grad[ci*dim + 0] += +alpha * w * gpx_c;
            grad[ci*dim + 1] += +alpha * w * gpy_c;
            grad[ci*dim + 2] += +alpha * w * gpz_c;
        }
    }
    const double dxw = di_world[0];
const double dyw = di_world[1];
const double dzw = di_world[2];

// Derivatives of R wrt q components (Hamilton, unit quaternion):
// q = (x,y,z,w). Below, we directly apply (∂R/∂qk) * d_world per component.
double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
// ---- wrt qx ----
const double dRdx_0 = 0.0*dxw +  2.0*q.y*dyw +  2.0*q.z*dzw;          // row0·dw
const double dRdx_1 = 2.0*q.y*dxw + (-4.0*q.x)*dyw + (-2.0*q.w)*dzw;
const double dRdx_2 = 2.0*q.z*dxw +  2.0*q.w*dyw + (-4.0*q.x)*dzw;
const double Jqx_x = dRdx_0;  // x component of (∂R/∂qx)*d_world
const double Jqx_y = dRdx_1;
const double Jqx_z = dRdx_2;

// ---- wrt qy ----
const double dRdy_0 = (-4.0*q.y)*dxw + 2.0*q.x*dyw +  2.0*q.w*dzw;
const double dRdy_1 =  2.0*q.x*dxw + 0.0*dyw      +  2.0*q.z*dzw;
const double dRdy_2 = (-2.0*q.w)*dxw + 2.0*q.z*dyw + (-4.0*q.y)*dzw;
const double Jqy_x = dRdy_0;
const double Jqy_y = dRdy_1;
const double Jqy_z = dRdy_2;

// ---- wrt qz ----
const double dRdz_0 = (-4.0*q.z)*dxw + (-2.0*q.w)*dyw + 2.0*q.x*dzw;
const double dRdz_1 =  2.0*q.w*dxw  + (-4.0*q.z)*dyw + 2.0*q.y*dzw;
const double dRdz_2 =  2.0*q.x*dxw  +  2.0*q.y*dyw   + 0.0*dzw;
const double Jqz_x = dRdz_0;
const double Jqz_y = dRdz_1;
const double Jqz_z = dRdz_2;

// ---- wrt qw ----
const double dRdw_0 = 0.0*dxw + (-2.0*q.z)*dyw +  2.0*q.y*dzw;
const double dRdw_1 = 2.0*q.z*dxw + 0.0*dyw    + (-2.0*q.x)*dzw;
const double dRdw_2 = (-2.0*q.y)*dxw + 2.0*q.x*dyw + 0.0*dzw;
const double Jqw_x = dRdw_0;
const double Jqw_y = dRdw_1;
const double Jqw_z = dRdw_2;

// dots with W*di_body
auto dotW = [&](double jx, double jy, double jz){
  return jx*Wdb_x + jy*Wdb_y + jz*Wdb_z;
};

const double dgi_dqx = -2.0 * dotW(Jqx_x, Jqx_y, Jqx_z);
const double dgi_dqy = -2.0 * dotW(Jqy_x, Jqy_y, Jqy_z);
const double dgi_dqz = -2.0 * dotW(Jqz_x, Jqz_y, Jqz_z);
const double dgi_dqw = -2.0 * dotW(Jqw_x, Jqw_y, Jqw_z);


// --- propagate through q = q_raw / ||q_raw|| ---
// reconstruct q_raw from the 4 contributors at row i
double qraw_x = 0.0, qraw_y = 0.0, qraw_z = 0.0, qraw_w = 0.0;
for (int k = 0; k < 4; ++k) {
  const int ci = rm.cidx[k];
  const double wk = rm.w[k];
  qraw_x += wk * P[ci*dim + 3];
  qraw_y += wk * P[ci*dim + 4];
  qraw_z += wk * P[ci*dim + 5];
  qraw_w += wk * P[ci*dim + 6];
}
const double qraw_n = std::sqrt(std::max(1e-18,
  qraw_x*qraw_x + qraw_y*qraw_y + qraw_z*qraw_z + qraw_w*qraw_w));

// J_norm = (I - q q^T) / ||q_raw||  (symmetric → J^T = J)
const double dq_x = dgi_dqx, dq_y = dgi_dqy, dq_z = dgi_dqz, dq_w = dgi_dqw;
const double q_dot_dq = q.x*dq_x + q.y*dq_y + q.z*dq_z + q.w*dq_w;
const double Jn_dq_x = (dq_x - q.x*q_dot_dq) / qraw_n;
const double Jn_dq_y = (dq_y - q.y*q_dot_dq) / qraw_n;
const double Jn_dq_z = (dq_z - q.z*q_dot_dq) / qraw_n;
const double Jn_dq_w = (dq_w - q.w*q_dot_dq) / qraw_n;
// Chain to contributing quaternion control points for sample i
for (int k=0;k<4;++k){
  const int ci = rm.cidx[k];
  const double wk = rm.w[k];
  if (ci==0 || ci==(g_par7.num_ctrl_pts-1)) continue;
  grad[ci*dim + 3] += wk * Jn_dq_x;
  grad[ci*dim + 4] += wk * Jn_dq_y;
  grad[ci*dim + 5] += wk * Jn_dq_z;
  grad[ci*dim + 6] += wk * Jn_dq_w;
}
    if (slacks){
    // wrt slack
    grad[n_ctrl + i] = -1.0;}
  }
  return gi;
}

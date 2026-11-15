#include "governed_allocator.hpp"
#include "model_fixture.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>

// ROS-independent rigid-body regression. This does not model contacts or every
// articulated Gazebo rotor effect. Motor RPM filtering and physical caps match
// the supplied plugin. CLI: CSV motor fraction gamma scenario mode.
int main(int argc,char** argv) {
 const std::string output=argc>1?argv[1]:"governed_offline.csv";
 const int motor=argc>2?std::stoi(argv[2]):6;
 const double fraction=argc>3?std::stod(argv[3]):0;
 const double gamma=argc>4?std::stod(argv[4]):0;
 const std::string scenario=argc>5?argv[5]:"hover";
 const std::string mode=argc>6?argv[6]:"governed";
 if(motor<0||motor>7||fraction<0||fraction>1||gamma<0||
    (scenario!="hover"&&scenario!="trajectory"&&scenario!="restore"&&scenario!="jitter")||
    (mode!="paper"&&mode!="governed"))return 2;
 omni::Options options;options.gamma=gamma;
 omni::GovernedAllocator allocator(fixture::allocation(),options);allocator.enabled=mode=="governed";
 const Eigen::Matrix3d J=fixture::inertia();
 const Eigen::Vector3d kp(15,15,18),kv(10,10,12),ki(6,6,8),kr=Eigen::Vector3d::Constant(8.5),kw=Eigen::Vector3d::Constant(6.5);
 Eigen::Vector3d p(0,0,1),velocity=Eigen::Vector3d::Zero(),omega=velocity,integral=velocity;
 Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
 omni::V8 lo=omni::V8::Zero(),hi=omni::V8::Constant(fixture::thrust_max),previous=lo,speed=lo;
 auto init=allocator.step(Eigen::Vector3d(0,0,fixture::mass*9.81),q.toRotationMatrix(),q.toRotationMatrix(),omega,J,kr,kw,lo,lo,hi);
 previous=init.u;speed=previous.array().sqrt();
 std::ofstream csv(output);csv<<std::setprecision(12);
 csv<<"t,fault,ex,ey,ez,attitude_error,target_tracking_error,omega_norm,accepted,fallback,status,step_ms,governor_ms,solver_ms,target_found,target_angle,force_error,moment_error";
 for(int i=0;i<8;i++)csv<<",u"<<i;
 csv<<",roll,pitch,yaw,target_roll,target_pitch,target_yaw\n";
 double t=0,max_position=0,max_step=0;int k=0,limited=0;
 const double jitter[5]={.005,.0025,.005,.0025,.005};
 while(t<80-1e-8){
  const double dt=scenario=="jitter"?jitter[k%5]:.004;
  Eigen::Vector3d reference(0,0,1),vref=Eigen::Vector3d::Zero(),aref=vref;
  Eigen::Matrix3d Rref=Eigen::Matrix3d::Identity();
  if(scenario=="trajectory"&&t>=15){
   const double tau=t-15,s=std::max(0.,std::min(1.,tau/5));
   const double h=s*s*s*(10+s*(-15+6*s));
   const double hd=s>0&&s<1?(30*s*s-60*s*s*s+30*s*s*s*s)/5:0;
   const double hdd=s>0&&s<1?(60*s-180*s*s+120*s*s*s)/25:0;
   const double freq[3]={.4,.4,.2},amp[3]={.3,.2,.1};
   for(int i=0;i<3;i++){
    const double sn=std::sin(freq[i]*tau),cs=std::cos(freq[i]*tau);
    reference[i]+=amp[i]*h*sn;
    vref[i]=amp[i]*(hd*sn+h*freq[i]*cs);
    aref[i]=amp[i]*(hdd*sn+2*hd*freq[i]*cs-h*freq[i]*freq[i]*sn);
   }
   Rref=(Eigen::AngleAxisd(.2*h*std::sin(.12*tau),Eigen::Vector3d::UnitZ())*
         Eigen::AngleAxisd(.2*h*std::sin(.2*tau),Eigen::Vector3d::UnitY())*
         Eigen::AngleAxisd(.1*h*std::sin(.16*tau),Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  hi.setConstant(fixture::thrust_max);
  const bool fault=t>=5 && !(scenario=="restore"&&t>=40);
  if(fault)hi[motor]*=fraction;
  const Eigen::Vector3d ep=reference-p,ev=vref-velocity;
  const Eigen::Vector3d proposed=(integral+ep.cwiseMax(-.5).cwiseMin(.5)*dt).cwiseMax(-1).cwiseMin(1);
  const Eigen::Vector3d force=fixture::mass*(aref+9.81*Eigen::Vector3d::UnitZ())+
     kp.cwiseProduct(ep.cwiseMax(-.5).cwiseMin(.5))+kv.cwiseProduct(ev.cwiseMax(-1).cwiseMin(1))+ki.cwiseProduct(proposed);
  const auto result=allocator.step(force,q.toRotationMatrix(),Rref,omega,J,kr,kw,previous,lo,hi);
  if(result.accepted)integral=proposed;
  if(!result.u.allFinite()||(result.u.array()<0).any()||(result.u.array()>hi.array()).any()){
   std::cerr<<"Motor bounds failed at "<<t<<'\n';return 3;
  }
  previous=result.u;speed=.6*speed+.4*result.u.array().sqrt().matrix();
  const omni::V8 actual=speed.array().square().matrix().cwiseMin(hi);
  const omni::V6 wrench=allocator.A*actual;
  const double reference_error=Eigen::AngleAxisd(q.toRotationMatrix().transpose()*Rref).angle();
  const double target_error=Eigen::AngleAxisd(q.toRotationMatrix().transpose()*result.attitude_target).angle();
  const Eigen::Vector3d euler=q.toRotationMatrix().eulerAngles(2,1,0);
  const Eigen::Vector3d target_euler=result.attitude_target.eulerAngles(2,1,0);
  csv<<t<<','<<fault<<','<<ep.x()<<','<<ep.y()<<','<<ep.z()<<','<<reference_error<<','<<target_error<<','<<omega.norm()<<','
     <<result.accepted<<','<<result.fallback<<','<<result.status<<','<<result.elapsed*1000<<','<<result.governor_seconds*1000<<','
     <<result.solver_seconds*1000<<','<<result.feasible_attitude_found<<','<<Eigen::AngleAxisd(result.attitude_target).angle()<<','
     <<result.force_tracking_error<<','<<result.moment_tracking_error;
  for(int i=0;i<8;i++)csv<<','<<result.u(i);
  csv<<','<<euler[2]<<','<<euler[1]<<','<<euler[0]<<','<<target_euler[2]<<','<<target_euler[1]<<','<<target_euler[0]<<'\n';
  const Eigen::Vector3d acceleration=q.toRotationMatrix()*wrench.head<3>()/fixture::mass-9.81*Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d domega=J.ldlt().solve(wrench.tail<3>()-omega.cross(J*omega));
  const Eigen::Vector3d delta=(omega+.5*dt*domega)*dt;
  if(delta.norm()>1e-12)q=(q*Eigen::Quaterniond(Eigen::AngleAxisd(delta.norm(),delta.normalized()))).normalized();
  omega+=dt*domega;p+=velocity*dt+.5*acceleration*dt*dt;velocity+=dt*acceleration;
  max_position=std::max(max_position,ep.norm());max_step=std::max(max_step,result.elapsed);
  if(!result.allocation_converged)limited++;
  if(!p.allFinite()||!omega.allFinite()||p.norm()>1000){std::cerr<<"Diverged at "<<t<<'\n';return 4;}
  t+=dt;k++;
 }
 std::cout<<"80 s "<<scenario<<" / "<<mode<<": max position error="<<max_position<<" m; max allocator step="<<max_step*1000
          <<" ms; nonconverged solves="<<limited<<"/"<<k<<"; final position="<<p.transpose()<<"; final body rate="<<omega.norm()<<" rad/s\n";
}

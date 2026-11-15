#include "model_fixture.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
// Independent rigid-body integration driving the actual C++ allocator.
// Not a Gazebo/ROS end-to-end test.
int main(int argc,char**argv) {
 const std::string output=argc>1?argv[1]:"offline_sim.csv";
 const int partial_motor=argc>2?std::stoi(argv[2]):5;
 const int stopped_motor=argc>3?std::stoi(argv[3]):6;
 if(partial_motor<0 || partial_motor>7 || stopped_motor<0 || stopped_motor>7) return 2;
 std::ofstream csv(output); csv << std::setprecision(10);
 csv << "t,phase,ex,ey,ez,attitude_error,accepted,status,violation,solve_ms,wrench_error";
 for(int i=0;i<8;i++) csv<<",u"<<i; csv<<"\n";
 omni::Options options; options.max_seconds=0.002;
 omni::FeasibleAllocator allocator(fixture::allocation(),options);
 const double dt=0.004; const Eigen::Matrix3d J=fixture::inertia();
 const Eigen::Vector3d kp(15,15,18),kd(10,10,12),kr(8.5,8.5,8.5),kw(6.5,6.5,6.5);
 Eigen::Vector3d p(0,0,1),velocity=Eigen::Vector3d::Zero(),omega=velocity,integral=velocity;
 Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
 omni::V8 lo=omni::V8::Zero(),hi=omni::V8::Constant(fixture::thrust_max),previous=lo,speed=lo;
 auto init=allocator.step(Eigen::Vector3d(0,0,fixture::mass*9.81),q.toRotationMatrix(),q.toRotationMatrix(),
                          omega,J,kr,kw,lo,lo,hi);
 previous=init.u; speed=previous.array().sqrt();
 double max_position=0,max_solve=0; int accepted=0, failures=0;
 for(int k=0;k<5000;k++) {
   const double t=k*dt;
   const double s=std::max(0.,std::min(1.,(t-2)/3));
   const double h=s*s*s*(10+s*(-15+6*s));
   const double hd=(s>0&&s<1)?(30*s*s-60*s*s*s+30*s*s*s*s)/3:0;
   const double hdd=(s>0&&s<1)?(60*s-180*s*s+120*s*s*s)/9:0;
   Eigen::Vector3d reference(0.4*h,0,1),vref(0.4*hd,0,0),aref(0.4*hdd,0,0);
   Eigen::Matrix3d Rref=Eigen::AngleAxisd(0.4*h,Eigen::Vector3d::UnitY()).toRotationMatrix();
   hi.setConstant(fixture::thrust_max); int phase=0;
   if(t>=6 && t<10){hi(partial_motor)/=3; phase=1;}
   if(t>=14){hi(stopped_motor)=0; phase=2;}
   Eigen::Vector3d ep=reference-p,ev=vref-velocity;
   const Eigen::Vector3d proposed=(integral+ep.cwiseMax(-0.5).cwiseMin(0.5)*dt).cwiseMax(-1).cwiseMin(1);
   Eigen::Vector3d force=fixture::mass*(aref+9.81*Eigen::Vector3d::UnitZ())+
       kp.cwiseProduct(ep.cwiseMax(-0.5).cwiseMin(0.5))+kd.cwiseProduct(ev.cwiseMax(-1).cwiseMin(1))+Eigen::Vector3d(6,6,8).cwiseProduct(proposed);
   auto result=allocator.step(force,q.toRotationMatrix(),Rref,omega,J,kr,kw,previous,lo,hi);
   if(result.accepted) integral=proposed;
   previous=result.u; speed=0.6*speed+0.4*result.u.array().sqrt().matrix();
   omni::V8 actual=speed.array().square().matrix().cwiseMin(hi);
   omni::V6 wrench=allocator.A*actual;
   Eigen::Vector3d acceleration=q.toRotationMatrix()*wrench.head<3>()/fixture::mass-
       9.81*Eigen::Vector3d::UnitZ();
   const Eigen::Vector3d domega=J.ldlt().solve(wrench.tail<3>()-omega.cross(J*omega));
   Eigen::Vector3d delta=(omega+0.5*dt*domega)*dt;
   if(delta.norm()>1e-12) q=(q*Eigen::Quaterniond(Eigen::AngleAxisd(delta.norm(),delta.normalized()))).normalized();
   omega+=dt*domega; p+=velocity*dt+0.5*acceleration*dt*dt; velocity+=dt*acceleration;
   max_position=std::max(max_position,ep.norm()); max_solve=std::max(max_solve,result.elapsed*1000);
   if(result.accepted)accepted++; else failures++;
   const double angle=Eigen::AngleAxisd(q.toRotationMatrix().transpose()*Rref).angle();
   csv<<t<<','<<phase<<','<<ep.x()<<','<<ep.y()<<','<<ep.z()<<','<<angle<<','<<result.accepted<<','
      <<result.status<<','<<result.violation<<','<<result.elapsed*1000<<','<<result.wrench_error;
   for(int i=0;i<8;i++)csv<<','<<result.u(i); csv<<'\n';
   if(!p.allFinite() || !omega.allFinite() || p.norm()>1000){std::cerr<<"Diverged at "<<t<<'\n'; return 1;}
 }
 std::cout<<"20 s rigid-body simulation: max position error="<<max_position<<" m; accepted="
          <<accepted<<"/5000; fallback="<<failures<<"; max step wall time="<<max_solve<<" ms\n";
 std::cout<<"Final position: "<<p.transpose()<<"; body rates: "<<omega.transpose()<<'\n';
}

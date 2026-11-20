#pragma once
// Isolated synchronous diagnostic weighting. No baseline snapshots or pair list.
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <map>
#include <tuple>
#include <cstring>

namespace online_direction {
using Matrix6=Eigen::Matrix<double,6,6>;
using Key=std::tuple<const void*,size_t,size_t>;
inline std::map<Key,Matrix6>& pendingCovariances(){static std::map<Key,Matrix6> x;return x;}
inline bool enabled(){
 const char* mode=std::getenv("PGO_DIRECTION_MODE");
 if(!mode||!std::strcmp(mode,"score"))return false;
 if(!std::strcmp(mode,"weak100"))return true;
 throw std::invalid_argument("PGO_DIRECTION_MODE must be score or weak100");
}
template<class T> inline void array(std::ostream& o,const T& m){
 o<<'[';bool comma=false;for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j){if(comma)o<<',';comma=true;o<<m(i,j);}o<<']';
}
struct Geometry {
 size_t supported=0;M3D h=M3D::Zero(),r=M3D::Identity(),world=M3D::Zero();
 V3D eigenvalues=V3D::Zero(),weights=V3D::Ones();
 Matrix6 covariance=Matrix6::Zero();
 double raw_rotation_orthogonality=0,projection_max_change=0,gauge_error=0,energy_error=0;
 std::string rejection;
};
inline bool covariance(const M3D& h,double score,const M3D& raw_r,Geometry& g){
 g.h=h;
 if(!h.allFinite()||!raw_r.allFinite()||!std::isfinite(score)||score<=0){g.rejection="invalid_geometry_or_score";return false;}
 Eigen::SelfAdjointEigenSolver<M3D> e(h);
 if(e.info()!=Eigen::Success||!e.eigenvalues().allFinite()||e.eigenvalues()(0)<-1e-10||e.eigenvalues()(2)<=1e-12){g.rejection="invalid_Htt_eigensystem";return false;}
 g.eigenvalues=e.eigenvalues();
 for(int i=0;i<3;++i)g.weights(i)=g.eigenvalues(i)<.1*g.eigenvalues(2)?.01:1.;
 g.world=score*e.eigenvectors()*g.weights.cwiseInverse().asDiagonal()*e.eigenvectors().transpose();
 Eigen::JacobiSVD<M3D> svd(raw_r,Eigen::ComputeFullU|Eigen::ComputeFullV);
 if(svd.info()!=Eigen::Success||!svd.singularValues().allFinite()||svd.singularValues().minCoeff()<.5){g.rejection="invalid_covariance_frame";return false;}
 M3D sign=M3D::Identity();sign(2,2)=(svd.matrixU()*svd.matrixV().transpose()).determinant();
 g.r=svd.matrixU()*sign*svd.matrixV().transpose();
 g.raw_rotation_orthogonality=(raw_r.transpose()*raw_r-M3D::Identity()).norm();
 g.projection_max_change=(g.r-raw_r).cwiseAbs().maxCoeff();
 g.covariance=Matrix6::Identity()*score;
 const M3D body=g.r.transpose()*g.world*g.r;
 g.covariance.bottomRightCorner<3,3>()=(body+body.transpose())*.5;
 if(!g.covariance.allFinite()||(g.covariance-g.covariance.transpose()).norm()>1e-10||Eigen::LLT<Matrix6>(g.covariance).info()!=Eigen::Success){g.rejection="non_SPD_covariance";return false;}
 if((g.covariance.topLeftCorner<3,3>()-score*M3D::Identity()).norm()>1e-14||g.covariance.topRightCorner<3,3>().norm()!=0||g.covariance.bottomLeftCorner<3,3>().norm()!=0){g.rejection="rotation_or_cross_block_changed";return false;}
 return true;
}
inline void audit(const char* stage,size_t source,size_t target,double score,const Geometry& g,size_t source_n,size_t target_n,const M4F* delta=nullptr){
 const char* path=std::getenv("PGO_DIRECTION_AUDIT_PATH");
 if(!path)throw std::runtime_error("weak100 requires PGO_DIRECTION_AUDIT_PATH");
 std::ofstream f(path,std::ios::app);if(!f)throw std::runtime_error("Cannot append direction audit");
 f<<std::setprecision(17)<<"{\"stage\":\""<<stage<<"\",\"source_id\":"<<source<<",\"target_id\":"<<target<<",\"mode\":\"weak100\",\"original_score\":"<<score
  <<",\"reason\":\""<<g.rejection<<"\",\"source_points\":"<<source_n<<",\"target_points\":"<<target_n<<",\"supported_planes\":"<<g.supported;
 if(delta){f<<",\"accepted_icp_delta_row_major\":";array(f,*delta);}
 if(g.h.allFinite()){f<<",\"Htt_world_row_major\":";array(f,g.h);}
 if(g.rejection.empty()){
  f<<",\"Htt_eigenvalues_ascending\":";array(f,g.eigenvalues);f<<",\"information_weights\":";array(f,g.weights);
  f<<",\"translation_covariance_world_row_major\":";array(f,g.world);
  f<<",\"covariance_frame_R_refined_SO3_row_major\":";array(f,g.r);
  f<<",\"covariance_rotxyz_transxyz_row_major\":";array(f,g.covariance);
  f<<",\"raw_rotation_orthogonality_error\":"<<g.raw_rotation_orthogonality<<",\"covariance_frame_projection_max_change\":"<<g.projection_max_change
   <<",\"gauge_rotation_invariance_max_error\":"<<g.gauge_error<<",\"quadratic_energy_coordinate_error\":"<<g.energy_error;
 }
 f<<"}\n";if(!f)throw std::runtime_error("Direction audit write failed");
}
inline bool prepare(const void* owner,size_t source_id,size_t target_id,double score,const CloudType::ConstPtr& aligned,const CloudType::ConstPtr& target,const M3D& raw_refined_r,const M4F& delta){
 Geometry g;
 if(aligned->empty()||target->empty()){g.rejection="empty_cloud";audit("rejected",source_id,target_id,score,g,aligned->size(),target->size(),&delta);return false;}
 pcl::KdTreeFLANN<PointType> tree;tree.setInputCloud(target);std::vector<int> ids(10);std::vector<float> squared(10);
 for(const auto& p:*aligned){
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)){g.rejection="nonfinite_source_point";break;}
  if(tree.nearestKSearch(p,10,ids,squared)!=10||squared.back()>.25)continue;
  V3D mean=V3D::Zero();for(int id:ids){const auto& q=(*target)[id];mean+=V3D(q.x,q.y,q.z);}mean/=10.;
  M3D c=M3D::Zero();for(int id:ids){const auto& q=(*target)[id];V3D v=V3D(q.x,q.y,q.z)-mean;c+=v*v.transpose();}c/=10.;
  Eigen::SelfAdjointEigenSolver<M3D> e(c);
  if(e.info()!=Eigen::Success||!e.eigenvalues().allFinite()||e.eigenvalues().sum()<1e-12||e.eigenvalues()(0)/e.eigenvalues().sum()>.05)continue;
  const V3D n=e.eigenvectors().col(0);bool valid=true;
  for(int id:ids){const auto& q=(*target)[id];if(std::abs(n.dot(V3D(q.x,q.y,q.z)-mean))>.1){valid=false;break;}}
  if(!valid||std::abs(n.dot(V3D(p.x,p.y,p.z)-mean))>.2)continue;
  g.h+=n*n.transpose();++g.supported;
 }
 if(g.rejection.empty()&&g.supported<30)g.rejection="fewer_than_30_reliable_planes";
 if(!g.rejection.empty()){audit("rejected",source_id,target_id,score,g,aligned->size(),target->size(),&delta);return false;}
 g.h/=double(g.supported);
 if(!covariance(g.h,score,raw_refined_r,g)){audit("rejected",source_id,target_id,score,g,aligned->size(),target->size(),&delta);return false;}
 // Runtime covariance-coordinate self-check; never changes the measurement.
 const M3D q=(Eigen::AngleAxisd(.73,V3D::UnitX())*Eigen::AngleAxisd(-.31,V3D::UnitY())*Eigen::AngleAxisd(.22,V3D::UnitZ())).toRotationMatrix();
 Geometry rotated;
 if(!covariance(q*g.h*q.transpose(),score,q*g.r,rotated))throw std::runtime_error("Gauge covariance self-check failed");
 g.gauge_error=(g.covariance-rotated.covariance).cwiseAbs().maxCoeff();
 V3D v(.012,-.018,.009),vb=g.r.transpose()*v;
 g.energy_error=std::abs(v.dot(g.world.ldlt().solve(v))-vb.dot(g.covariance.bottomRightCorner<3,3>().ldlt().solve(vb)));
 if(g.gauge_error>1e-9||g.energy_error>1e-9)throw std::runtime_error("Covariance coordinate numerical invariant failed");
 if(!pendingCovariances().emplace(Key(owner,source_id,target_id),g.covariance).second)throw std::runtime_error("Duplicate pending online covariance");
 audit("prepared",source_id,target_id,score,g,aligned->size(),target->size(),&delta);return true;
}
inline gtsam::SharedNoiseModel noise(const void* owner,const LoopPair& pair){
 if(!enabled())return gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones()*pair.score);
 auto& cache=pendingCovariances();const Key key(owner,pair.source_id,pair.target_id);auto it=cache.find(key);
 if(it==cache.end())throw std::runtime_error("Missing online covariance for accepted factor; frozen/scheduled factors are not allowed in weak100 mode");
 const Matrix6 c=it->second;
 if((c.topLeftCorner<3,3>()-pair.score*M3D::Identity()).norm()>1e-14)throw std::runtime_error("Stored covariance score mismatch");
 auto model=gtsam::noiseModel::Gaussian::Covariance(c);
 const Matrix6 actual=model->covariance();
 const double model_error=(actual-c).cwiseAbs().maxCoeff();
 if(!actual.allFinite()||model_error>1e-9||Eigen::LLT<Matrix6>(actual).info()!=Eigen::Success)throw std::runtime_error("Actual GTSAM noise covariance mismatch");
 const char* path=std::getenv("PGO_DIRECTION_AUDIT_PATH");std::ofstream f(path,std::ios::app);if(!f)throw std::runtime_error("Cannot append applied noise audit");
 f<<std::setprecision(17)<<"{\"stage\":\"applied\",\"source_id\":"<<pair.source_id<<",\"target_id\":"<<pair.target_id<<",\"original_score\":"<<pair.score<<",\"r_offset_row_major\":";array(f,pair.r_offset);
 f<<",\"t_offset\":";array(f,pair.t_offset);f<<",\"covariance_rotxyz_transxyz_row_major\":";array(f,c);
 f<<",\"actual_gtsam_covariance_row_major\":";array(f,actual);f<<",\"actual_gtsam_covariance_max_error\":"<<model_error;f<<"}\n";if(!f)throw std::runtime_error("Applied direction audit write failed");
 cache.erase(it);return model;
}
} // namespace online_direction

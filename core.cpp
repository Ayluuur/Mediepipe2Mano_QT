#include "core.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdexcept>
#include <tuple>

namespace m2m {
namespace {
constexpr double pi=3.14159265358979323846;
constexpr std::array<int,21> kpParents={-1,0,1,2,0,4,5,0,7,8,0,10,11,0,13,14,3,6,9,12,15};
constexpr std::array<int,21> mpToMano={0,13,14,15,20,1,2,3,16,4,5,6,17,10,11,12,19,7,8,9,18};
constexpr int chains[5][4]={{1,2,3,16},{4,5,6,17},{7,8,9,18},{10,11,12,19},{13,14,15,20}};
double alpha(double cutoff, double dt) { return 1/(1+1/(2*pi*cutoff)/dt); }
double median(std::vector<double> values) {
    std::sort(values.begin(),values.end());
    auto n=values.size();
    return n%2 ? values[n/2] : (values[n/2-1]+values[n/2])*.5;
}
Mat3 skew(const Vec3& v) {
    Mat3 s; s << 0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0; return s;
}
Matrix readMatrix(std::ifstream& in) {
    uint32_t rows=0,cols=0;
    in.read(reinterpret_cast<char*>(&rows),4); in.read(reinterpret_cast<char*>(&cols),4);
    if(!in || rows==0 || cols==0 || uint64_t(rows)*cols>10000000)
        throw std::runtime_error("Invalid MANO binary dimensions");
    Matrix m(rows,cols);
    in.read(reinterpret_cast<char*>(m.data()),std::streamsize(rows)*cols*8);
    if(!in || !m.allFinite()) throw std::runtime_error("Truncated or nonfinite MANO binary");
    return m;
}
void requireShape(const Matrix& m,int r,int c) {
    if(m.rows()!=r || m.cols()!=c) throw std::runtime_error("Unsupported MANO model shape");
}
}

Vec3 normalize(const Vec3& v) { double n=v.norm(); return n<1e-8 ? Vec3::Zero().eval() : (v/n).eval(); }
Mat3 rotationBetween(const Vec3& a,const Vec3& b) {
    Vec3 source=normalize(a),target=normalize(b);
    if(source.isZero(0) || target.isZero(0)) return Mat3::Identity();
    double cosine=std::clamp(source.dot(target),-1.,1.);
    if(cosine>1-1e-7) return Mat3::Identity();
    if(cosine<-1+1e-7) {
        Vec3 candidate=std::abs(source.x())>.9 ? Vec3(0,1,0) : Vec3(1,0,0);
        Vec3 axis=normalize(source.cross(candidate));
        return 2*axis*axis.transpose()-Mat3::Identity();
    }
    Mat3 s=skew(source.cross(target));
    return Mat3::Identity()+s+s*s/(1+cosine);
}
Mat3 palmFrame(const Matrix& world) {
    Vec3 across=normalize((world.row(5)-world.row(17)).transpose());
    Vec3 forward=normalize((world.row(9)-world.row(0)).transpose());
    Vec3 normal=normalize(across.cross(forward));
    if(normal.isZero(0)) return Mat3::Identity();
    across=normalize(forward.cross(normal));
    Mat3 f; f.col(0)=across; f.col(1)=forward; f.col(2)=normal;
    return Vec3(1,-1,-1).asDiagonal()*f;
}
Matrix scenePoints(const Matrix& points,const Eigen::Vector2d& wrist,double depth,const std::optional<Vec3>& translation) {
    Matrix out=points*Vec3(1,-1,-1).asDiagonal();
    out.rowwise()+=translation.value_or(Vec3((wrist.x()-.5)*300,(.5-wrist.y())*240,depth)).transpose();
    return out;
}
std::optional<double> palmScale(const Matrix& screen,const Matrix& world,const Matrix& reference,
    int width,int height,std::map<int,double>* corrections) {
    constexpr int screenPairs[][2]={{0,9},{5,17},{5,9},{9,13},{13,17}};
    constexpr int manoPairs[][2]={{0,4},{1,7},{1,4},{4,10},{10,7}};
    std::vector<double> ratios,weights;
    std::vector<int> indices;
    for(int i=0;i<5;++i) {
        Vec3 segment=(world.row(screenPairs[i][0])-world.row(screenPairs[i][1])).transpose();
        double length=segment.norm();
        if(!std::isfinite(length) || length<1e-8) continue;
        double projection=segment.head<2>().norm()/length;
        Eigen::Vector2d pixels=(screen.row(screenPairs[i][0])-screen.row(screenPairs[i][1])).head<2>().transpose();
        pixels.array()*=Eigen::Array2d(width,height);
        double observed=pixels.norm(),fixed=(reference.row(manoPairs[i][0])-reference.row(manoPairs[i][1])).norm();
        if(projection<.3 || !std::isfinite(observed) || observed<5 || !std::isfinite(fixed) || fixed<1) continue;
        ratios.push_back(fixed*projection/observed); weights.push_back(observed*observed); indices.push_back(i);
    }
    Vec3 a=(world.row(5)-world.row(0)).transpose(),b=(world.row(17)-world.row(0)).transpose();
    Vec3 normal=a.cross(b); double n=normal.norm();
    double facing=std::isfinite(n) && n>1e-10 ? std::abs(normal.z())/n : 0;
    if(ratios.empty()) return std::nullopt;
    if(ratios.size()<2 && (!corrections || !corrections->count(indices[0]) || weights[0]<225)) return std::nullopt;
    auto raw=ratios;
    if(corrections && facing<.8) for(size_t i=0;i<ratios.size();++i)
        if(corrections->count(indices[i])) ratios[i]*=corrections->at(indices[i]);
    std::vector<int> order;
    double total=0;
    for(int i=0;i<int(ratios.size());++i) { order.push_back(i); total+=weights[i]; }
    std::stable_sort(order.begin(),order.end(),[&](int i,int j){return ratios[i]<ratios[j];});
    double cumulative=0,scale=ratios[order.back()];
    for(int i:order) { cumulative+=weights[i]; if(cumulative>=total*.5) { scale=ratios[i]; break; } }
    if(corrections && facing>=.8) for(size_t i=0;i<raw.size();++i) {
        double correction=scale/raw[i],previous=corrections->count(indices[i])?corrections->at(indices[i]):correction;
        (*corrections)[indices[i]]=previous+.1*(correction-previous);
    }
    return scale;
}
std::optional<Vec3> screenTranslation(const Matrix& screen,const Matrix& keypoints,
    int width,int height,double depth,std::optional<double> mmPerPixel) {
    Eigen::Vector2d wrist=screen.row(0).head<2>().transpose();
    if(!wrist.allFinite()) return std::nullopt;
    if(!mmPerPixel) {
        constexpr int sp[][2]={{0,9},{5,17},{5,9},{9,13},{13,17}};
        constexpr int mp[][2]={{0,4},{1,7},{1,4},{4,10},{10,7}};
        std::vector<double> ratios;
        for(int i=0;i<5;++i) {
            Eigen::Vector2d delta=(screen.row(sp[i][0])-screen.row(sp[i][1])).head<2>().transpose();
            delta.array()*=Eigen::Array2d(width,height);
            double pixels=delta.norm(),model=(keypoints.row(mp[i][0])-keypoints.row(mp[i][1])).head<2>().norm();
            if(std::isfinite(pixels) && std::isfinite(model) && pixels>=5 && model>=1) ratios.push_back(model/pixels);
        }
        if(ratios.size()<2) return std::nullopt;
        mmPerPixel=median(ratios);
    }
    return Vec3((wrist.x()-.5)*width* *mmPerPixel,-(wrist.y()-.5)*height* *mmPerPixel,depth);
}
OneEuro::OneEuro(double cutoff,double b,double derivative,int window,std::optional<double> speed):
    minCutoff(cutoff),beta(b),derivativeCutoff(derivative),medianWindow(window),maxSpeed(speed) {
    if(!std::isfinite(cutoff) || cutoff<=0 || !std::isfinite(b) || b<0 || !std::isfinite(derivative) || derivative<=0)
        throw std::invalid_argument("Filter cutoffs must be positive and beta nonnegative");
    if(window<1 || window%2!=1) throw std::invalid_argument("median_window must be a positive odd integer");
    if(speed && (!std::isfinite(*speed) || *speed<=0)) throw std::invalid_argument("max_speed must be finite and positive");
}
Matrix OneEuro::operator()(const Matrix& input,double time) {
    if(!input.allFinite()) {
        if(value.size()==0) throw std::invalid_argument("Cannot initialize filter with nonfinite values");
        return value;
    }
    if(value.size()==0) {
        samples.assign(medianWindow,input);
        value=input; raw=input; derivative=Matrix::Zero(input.rows(),input.cols()); timestamp=time;
        return value;
    }
    double dt=std::clamp(time-timestamp,1./120.,.1);
    samples.push_back(input); if(samples.size()>size_t(medianWindow)) samples.pop_front();
    Matrix current=input;
    if(medianWindow>1) for(Eigen::Index k=0;k<input.size();++k) {
        std::vector<double> values;
        for(const auto& sample:samples) values.push_back(sample.data()[k]);
        current.data()[k]=median(values);
    }
    if(maxSpeed) {
        Matrix delta=current-value; double distance=delta.norm(),limit=*maxSpeed*dt;
        if(distance>limit) current=value+delta*(limit/distance);
    }
    derivative+=alpha(derivativeCutoff,dt)*((current-raw)/dt-derivative);
    for(Eigen::Index i=0;i<value.size();++i)
        value.data()[i]+=alpha(minCutoff+beta*std::abs(derivative.data()[i]),dt)*(current.data()[i]-value.data()[i]);
    raw=current; timestamp=time; return value;
}
double DepthEstimator::progress() const {
    return calibrated()?1:std::min(double(samples.size())/std::max(config.calibrationFrames,1),1.);
}
double DepthEstimator::update(const Matrix& screen,double time,int width,int height) {
    constexpr int segments[5][2]={{0,9},{5,17},{5,9},{9,13},{13,17}};
    std::vector<double> lengths;
    for(auto& s:segments) {
        Eigen::Vector2d delta=(screen.row(s[0])-screen.row(s[1])).head<2>().transpose();
        delta.x()*=width; delta.y()*=height; lengths.push_back(delta.norm());
    }
    return updateSize(median(lengths),time);
}
double DepthEstimator::updateSize(double size,double time,bool calibrate) {
    if(!std::isfinite(size) || size<5) return depth;
    if(!lastSeen || time-*lastSeen>.35) filter.reset();
    auto previous=lastSeen; lastSeen=time;
    double rawDepth=config.reference;
    if(!calibrated()) {
        if(calibrate) samples.push_back(size);
        if(calibrate && samples.size()>=size_t(config.calibrationFrames)) referenceSize=median(samples);
    } else rawDepth=config.reference+(config.reference*size/ *referenceSize-config.reference)*config.motionGain;
    rawDepth=std::clamp(rawDepth,config.minimum,config.maximum);
    if(previous && config.maxSpeed>0) {
        double limit=config.maxSpeed*std::clamp(time-*previous,1./120.,.1);
        rawDepth=std::clamp(rawDepth,depth-limit,depth+limit);
    }
    Matrix x(1,1); x(0,0)=rawDepth; depth=filter(x,time)(0,0); return depth;
}
std::string MultiHandDepthEstimator::calibrationStatus(double time) const {
    if(deadline) {
        double remaining=*deadline-time;
        if(remaining>0) {
            std::ostringstream stream; stream<<"Depth calibration in "<<std::fixed<<std::setprecision(1)<<remaining<<"s - hold palms at same depth";
            return stream.str();
        }
        return "Depth calibration: waiting for hands";
    }
    if(completedAt && time-*completedAt<2) return "Depth calibration complete";
    return "ENTER: calibrate depth after 3s";
}
std::map<int,double> MultiHandDepthEstimator::update(const std::map<int,double>& sizes,double time) {
    std::vector<double> valid;
    for(const auto& [side,size]:sizes) if(std::isfinite(size) && size>=5) valid.push_back(size);
    if(deadline && time>=*deadline && !valid.empty()) {
        referenceSize=median(valid); samples={*referenceSize};
        for(int side=0;side<2;++side) {
            auto& e=estimators[side]; auto found=sizes.find(side);
            e.referenceSize=found!=sizes.end() && std::isfinite(found->second) && found->second>=5 ? found->second : *referenceSize;
            e.samples={*e.referenceSize}; e.filter.reset(); e.lastSeen.reset(); e.depth=e.config.reference;
        }
        deadline.reset(); completedAt=time;
    }
    if(!referenceSize) {
        if(!valid.empty()) { samples.push_back(median(valid)); if(samples.size()>=size_t(calibrationFrames)) referenceSize=median(samples); }
        for(auto& e:estimators) { e.samples=samples; e.referenceSize=referenceSize; }
    }
    std::map<int,double> depths;
    for(const auto& [side,size]:sizes) depths[side]=estimators.at(side).updateSize(size,time,false);
    return depths;
}

ManoModel::ManoModel(const std::string& path) {
    std::ifstream in(path,std::ios::binary);
    char magic[8]{}; in.read(magic,8);
    if(std::string(magic,8)!="M2MANO02") throw std::runtime_error("Cannot read converted MANO model: "+path);
    basis=readMatrix(in); mean=readMatrix(in); regressor=readMatrix(in);
    weights=readMatrix(in); meshTemplate=readMatrix(in); faces=readMatrix(in);
    Matrix p=readMatrix(in); poseBasis=readMatrix(in); requireShape(poseBasis,778*3,135);
    requireShape(basis,45,45); requireShape(mean,1,45); requireShape(regressor,16,778);
    requireShape(weights,778,16); requireShape(meshTemplate,778,3); requireShape(p,1,16);
    if(faces.cols()!=3 || faces.minCoeff()<0 || faces.maxCoeff()>=778)
        throw std::runtime_error("Invalid MANO faces");
    parents[0]=-1;
    for(int i=1;i<16;++i) {
        parents[i]=int(p(0,i));
        if(parents[i]<0 || parents[i]>=i) throw std::runtime_error("Invalid MANO joint hierarchy");
    }
    joints=regressor*meshTemplate;
    // Python __init__ uses zero absolute pose, NOT the PCA mean pose.
    update(Matrix::Zero(16,3));
}
void ManoModel::setPose(const Eigen::VectorXd& pca,bool keypointsOnly) {
    if(pca.size()<1 || pca.size()>45 || !pca.allFinite()) throw std::invalid_argument("Invalid PCA pose");
    Matrix values=pca.transpose()*basis.topRows(pca.size())+mean;
    Matrix pose=Matrix::Zero(16,3);
    for(int j=1;j<16;++j) for(int k=0;k<3;++k) pose(j,k)=values(0,(j-1)*3+k);
    update(pose,keypointsOnly);
}
void ManoModel::setAbsolutePose(const Matrix& pose,bool keypointsOnly) {
    requireShape(pose,16,3);
    if(!pose.allFinite()) throw std::invalid_argument("Invalid absolute pose");
    update(pose,keypointsOnly);
}
void ManoModel::update(const Matrix& pose,bool keypointsOnly) {
    Eigen::VectorXd feature(135);
    std::array<Eigen::Matrix4d,16> transforms;
    for(int j=0;j<16;++j) {
        Vec3 r=pose.row(j).transpose();
        double theta=std::max(r.norm(),std::numeric_limits<double>::epsilon());
        Vec3 axis=r/theta;
        Mat3 R=std::cos(theta)*Mat3::Identity()+(1-std::cos(theta))*axis*axis.transpose()+std::sin(theta)*skew(axis);
        Eigen::Matrix4d local=Eigen::Matrix4d::Identity();
        local.topLeftCorner<3,3>()=R;
        if(j) for(int r=0;r<3;++r) for(int c=0;c<3;++c)
            feature[(j-1)*9+r*3+c]=R(r,c)-(r==c?1.:0.);
        local.topRightCorner<3,1>()=j==0 ? joints.row(0).transpose().eval() : (joints.row(j)-joints.row(parents[j])).transpose().eval();
        transforms[j]=j==0 ? local : (transforms[parents[j]]*local).eval();
    }
    jointTransforms=transforms;
    keypoints.resize(21,3);
    for(int j=0;j<16;++j) {
        jointTransforms[j].topRightCorner<3,1>()*=1000;
        keypoints.row(j)=jointTransforms[j].topRightCorner<3,1>().transpose();
    }
    for(int j=0;j<16;++j) {
        Eigen::Vector4d joint; joint << joints.row(j).transpose(),0;
        transforms[j].col(3)-=(transforms[j]*joint).eval();
    }
    constexpr int tips[]={333,444,672,555,744};
    if(!keypointsOnly) vertices.resize(778,3);
    for(int i=0;i<(keypointsOnly?5:778);++i) {
        int v=keypointsOnly?tips[i]:i;
        Eigen::Matrix4d t=Eigen::Matrix4d::Zero();
        for(int j=0;j<16;++j) t+=weights(v,j)*transforms[j];
        Eigen::Vector4d p;
        p.head<3>()=meshTemplate.row(v).transpose()+poseBasis.middleRows(v*3,3)*feature;
        p[3]=1;
        Vec3 point=(t*p).head<3>()*1000;
        if(keypointsOnly) keypoints.row(16+i)=point.transpose();
        else vertices.row(v)=point.transpose();
    }
    if(!keypointsOnly) for(int i=0;i<5;++i) keypoints.row(16+i)=vertices.row(tips[i]);
}
Matrix ManoModel::keypointsBatch(const Matrix& poses) const {
    if(poses.cols()<1 || poses.cols()>45 || !poses.allFinite())
        throw std::invalid_argument("Invalid batch PCA poses");
    const int count=int(poses.rows());
    Matrix angles=(poses*basis.topRows(poses.cols())).rowwise()+mean.row(0);
    Matrix features(count,135), output(count,63);
    std::vector<std::array<Eigen::Matrix4d,16>> transforms(count);
    // Joint-major traversal shares rest geometry across all independent poses.
    for(int j=0;j<16;++j) for(int b=0;b<count;++b) {
        Vec3 r=Vec3::Zero();
        if(j) r=angles.block<1,3>(b,(j-1)*3).transpose();
        double theta=std::max(r.norm(),std::numeric_limits<double>::epsilon());
        Vec3 axis=r/theta;
        Mat3 rotation=std::cos(theta)*Mat3::Identity()+(1-std::cos(theta))*axis*axis.transpose()+std::sin(theta)*skew(axis);
        Eigen::Matrix4d local=Eigen::Matrix4d::Identity();
        local.topLeftCorner<3,3>()=rotation;
        local.topRightCorner<3,1>()=j==0?joints.row(0).transpose().eval():(joints.row(j)-joints.row(parents[j])).transpose().eval();
        transforms[b][j]=j==0?local:(transforms[b][parents[j]]*local).eval();
        output.block<1,3>(b,j*3)=transforms[b][j].topRightCorner<3,1>().transpose()*1000;
        if(j) for(int row=0;row<3;++row) for(int col=0;col<3;++col)
            features(b,(j-1)*9+row*3+col)=rotation(row,col)-(row==col?1.:0.);
    }
    for(int b=0;b<count;++b) for(int j=0;j<16;++j) {
        Eigen::Vector4d rest; rest<<joints.row(j).transpose(),0;
        transforms[b][j].col(3)-=(transforms[b][j]*rest).eval();
    }
    constexpr int tips[]={333,444,672,555,744};
    Matrix tipBasis(15,135);
    for(int i=0;i<5;++i) tipBasis.middleRows(i*3,3)=poseBasis.middleRows(tips[i]*3,3);
    // One matrix product for all pose corrective offsets in the batch.
    Matrix offsets=features*tipBasis.transpose();
    for(int i=0;i<5;++i) for(int b=0;b<count;++b) {
        Eigen::Matrix4d transform=Eigen::Matrix4d::Zero();
        for(int j=0;j<16;++j) transform+=weights(tips[i],j)*transforms[b][j];
        Eigen::Vector4d point;
        point.head<3>()=(meshTemplate.row(tips[i])+offsets.block<1,3>(b,i*3)).transpose(); point[3]=1;
        output.block<1,3>(b,(16+i)*3)=(transform*point).head<3>().transpose()*1000;
    }
    return output;
}
KeypointsToMano::KeypointsToMano(const std::string& path,bool left,int iterations,double smoothing,
                               int nPose,bool mirroredInput):
    model(path),left(left),mirroredInput(mirroredInput),iterations(iterations),smoothing(smoothing) {
    if(nPose<1 || nPose>45 || !(smoothing>=0 && smoothing<=1))
        throw std::invalid_argument("Invalid pose dimension or smoothing");
    if(mirroredInput) inputBasis(0,0)=-1;
    Matrix encoder=model.basis.topRows(nPose).transpose();
    Eigen::JacobiSVD<Matrix> svd(encoder,Eigen::ComputeThinU|Eigen::ComputeThinV);
    poseEncoder=svd.solve(Matrix::Identity(45,45));
    neutralPose=poseEncoder*(-model.mean.transpose()); pose=neutralPose;
    restWrist=model.keypoints.row(0).transpose();
    canonicalRest=model.keypoints.rowwise()-restWrist.transpose();
    for(int j=1;j<21;++j) lengths[j]=(model.keypoints.row(j)-model.keypoints.row(kpParents[j])).norm();
}
void KeypointsToMano::reset() {
    pose=neutralPose; hasPose=false; fitErrorMm=0;
    localToMano.setIdentity(); localToCamera.setIdentity();
    model.setAbsolutePose(Matrix::Zero(16,3));
}
Matrix KeypointsToMano::target(const Matrix& world) {
    requireShape(world,21,3);
    if(!world.allFinite()) throw std::runtime_error("Nonfinite landmarks");
    Matrix ordered(21,3);
    for(int i=0;i<21;++i) ordered.row(mpToMano[i])=world.row(i);
    for(int i=1;i<21;++i) if((ordered.row(i)-ordered.row(kpParents[i])).norm()<1e-8)
        throw std::invalid_argument("Landmarks contain a zero-length bone");
    Matrix centered=(world.rowwise()-world.row(0))*inputBasis;
    Vec3 index=centered.row(5).transpose(),pinky=centered.row(17).transpose();
    if(pinky.cross(index).norm()<1e-4*pinky.norm()*index.norm())
        throw std::invalid_argument("Palm landmarks are collinear");
    // IK normalization differs intentionally from the interaction normalize helper.
    auto unit=[](const Vec3& v)->Vec3 { return v/std::max(v.norm(),1e-8); };
    Vec3 normal=unit(pinky.cross(index)), x=unit(index), z=unit(x.cross(normal));
    Mat3 axes; axes.col(0)=x; axes.col(1)=normal; axes.col(2)=z;
    Mat3 toLocal=axes; localToCamera=toLocal.transpose();
    Matrix local=centered*toLocal, mapped=Matrix::Zero(21,3);
    for(int i=0;i<21;++i) mapped.row(mpToMano[i])=local.row(i);
    Matrix retargeted=Matrix::Zero(21,3); retargeted.row(0)=mapped.row(0);
    for(int j=1;j<21;++j) {
        Vec3 direction=(mapped.row(j)-mapped.row(kpParents[j])).transpose();
        double n=direction.norm(); direction=n<1e-8?Vec3(1,0,0):(direction/n).eval();
        retargeted.row(j)=retargeted.row(kpParents[j])+direction.transpose()*lengths[j];
    }
    Matrix source(4,3),dest(4,3);
    for(int i=0;i<4;++i) { source.row(i)=retargeted.row(1+3*i)-retargeted.row(0); dest.row(i)=canonicalRest.row(1+3*i); }
    Mat3 covariance=source.transpose()*dest;
    Eigen::JacobiSVD<Mat3> svd(covariance,Eigen::ComputeFullU|Eigen::ComputeFullV);
    Mat3 u=svd.matrixU(),vt=svd.matrixV().transpose(),rotation=u*vt;
    if(rotation.determinant()<0) { u.col(2)*=-1; rotation=u*vt; }
    localToMano=rotation;
    Matrix out=(retargeted.rowwise()-retargeted.row(0))*rotation;
    for(auto& chain:chains) {
        Eigen::RowVector3d offset=canonicalRest.row(chain[0])-out.row(chain[0]);
        for(int j:chain) out.row(j)+=offset;
    }
    Eigen::RowVector3d translation=restWrist.transpose()-out.row(0); out.rowwise()+=translation;
    return out;
}
Eigen::VectorXd KeypointsToMano::directionSeed(const Matrix& target) const {
    std::array<Mat3,16> rotations;
    for(auto& r:rotations) r.setIdentity();
    Eigen::VectorXd angles=Eigen::VectorXd::Zero(45);
    for(const auto& chain:chains) for(int k=0;k<3;++k) {
        int joint=chain[k],child=chain[k+1],parent=kpParents[joint];
        Vec3 a=(canonicalRest.row(child)-canonicalRest.row(joint)).transpose(); a.normalize();
        Vec3 b=rotations[parent].transpose()*(target.row(child)-target.row(joint)).transpose(); b.normalize();
        Vec3 axis=a.cross(b); double sine=axis.norm(),cosine=std::clamp(a.dot(b),-1.,1.);
        Vec3 rotvec=Vec3::Zero();
        if(sine>1e-8) rotvec=axis/sine*std::atan2(sine,cosine);
        else if(cosine<0) {
            Eigen::Index index; a.cwiseAbs().minCoeff(&index);
            axis=a.cross(Vec3::Unit(index)); rotvec=axis.normalized()*pi;
        }
        Mat3 local=Mat3::Identity();
        if(rotvec.norm()>0) local=Eigen::AngleAxisd(rotvec.norm(),rotvec.normalized()).toRotationMatrix();
        rotations[joint]=rotations[parent]*local;
        Eigen::AngleAxisd aa(local);
        angles.segment<3>((joint-1)*3)=aa.axis()*aa.angle();
    }
    return poseEncoder*(angles-model.mean.transpose());
}
void KeypointsToMano::setJacobianWorkers(int count) {
    if(count<0 || count>45) throw std::invalid_argument("jacobian_workers must be in [0,45]");
    jacobianWorkers=count;
    jacobianActive=0; jacobianPeak=0;
    derivativeWorkers.clear(); derivativeModels.clear();
    if(count<=1) return;
    for(int i=0;i<count;++i) {
        derivativeModels.push_back(std::make_unique<ManoModel>(model));
        derivativeWorkers.push_back(std::make_unique<Worker>());
    }
}
Eigen::VectorXd KeypointsToMano::solve(const Matrix& world) {
    Matrix goal=target(world);
    auto cost=[&](const Eigen::VectorXd& p) {
        model.setPose(p,true);
        return (model.keypoints-goal).squaredNorm()+2*(p-neutralPose).squaredNorm();
    };
    Eigen::VectorXd seed=directionSeed(goal);
    double seedCost=cost(seed),previousCost=cost(pose);
    Eigen::VectorXd params=seedCost<previousCost?seed:pose;
    double damping=20;
    for(int iteration=0;iteration<iterations;++iteration) {
        model.setPose(params,true); Matrix baseline=model.keypoints;
        Matrix difference=baseline-goal;
        Eigen::VectorXd residual=Eigen::Map<Eigen::VectorXd>(difference.data(),63);
        double currentCost=residual.squaredNorm()+2*(params-neutralPose).squaredNorm();
        Matrix jacobian(63,params.size());
        auto scalar=[&](int first,int last) {
            for(int k=first;k<last;++k) {
                Eigen::VectorXd perturbed=params; perturbed[k]+=1e-5;
                model.setPose(perturbed,true);
                Matrix derivative=(model.keypoints-baseline)/1e-5;
                jacobian.col(k)=Eigen::Map<Eigen::VectorXd>(derivative.data(),63);
            }
        };
        Matrix perturbations=params.transpose().replicate(params.size(),1);
        perturbations.diagonal().array()+=1e-5;
        auto evaluate=[&](const ManoModel& local,int first,int last) {
            Matrix samples=local.keypointsBatch(perturbations.middleRows(first,last-first));
            Eigen::Map<const Eigen::RowVectorXd> flatBaseline(baseline.data(),63);
            jacobian.middleCols(first,last-first)=((samples.rowwise()-flatBaseline)/1e-5).transpose();
        };
        if(jacobianWorkers==0) scalar(0,int(params.size()));
        else if(derivativeWorkers.empty()) evaluate(model,0,int(params.size()));
        else {
            std::vector<std::future<void>> jobs;
            int count=std::min(int(derivativeWorkers.size()),int(params.size()));
            for(int i=0;i<count;++i) jobs.push_back(derivativeWorkers[i]->submit([&,i,count] {
                int active=jacobianActive.fetch_add(1)+1;
                int peak=jacobianPeak.load();
                while(active>peak && !jacobianPeak.compare_exchange_weak(peak,active)) {}
                try {
                    evaluate(*derivativeModels[i],int(params.size())*i/count,int(params.size())*(i+1)/count);
                } catch(...) {
                    jacobianActive.fetch_sub(1);
                    throw;
                }
                jacobianActive.fetch_sub(1);
            }));
            std::exception_ptr failure;
            for(auto& job:jobs) try { job.get(); } catch(...) { failure=std::current_exception(); }
            if(failure) std::rethrow_exception(failure);
        }
        Matrix jtj=jacobian.transpose()*jacobian;
        Eigen::VectorXd rhs=jacobian.transpose()*residual+2*(params-neutralPose);
        bool accepted=false; double improvement=0;
        for(int trial=0;trial<32;++trial) {
            Matrix system=jtj; system.diagonal().array()+=damping+2;
            Eigen::LLT<Matrix> factor(system);
            Eigen::VectorXd delta=factor.solve(rhs);
            if(factor.info()!=Eigen::Success || !delta.allFinite())
                delta=system.completeOrthogonalDecomposition().solve(rhs);
            Eigen::VectorXd candidate=params-delta;
            double candidateCost=cost(candidate);
            if(std::isfinite(candidateCost) && candidateCost<currentCost) {
                params=candidate; improvement=currentCost-candidateCost;
                damping=std::max(damping/5,1e-12); accepted=true; break;
            }
            damping*=5;
        }
        if(!accepted || improvement/63<1e-8) break;
    }
    if(hasPose) params=smoothing*params+(1-smoothing)*pose;
    pose=params; hasPose=true; model.setPose(pose);
    fitErrorMm=std::sqrt((model.keypoints-goal).squaredNorm()/21);
    return pose;
}
SolveResult KeypointsToMano::cameraOriented() const {
    return {cameraVertices(),cameraKeypoints(),getSkeleton()};
}
Matrix KeypointsToMano::cameraVertices() const {
    Mat3 orientation=localToMano.transpose()*localToCamera*inputBasis;
    return (model.vertices.rowwise()-model.keypoints.row(0))*orientation;
}
Matrix KeypointsToMano::cameraKeypoints() const {
    Mat3 orientation=localToMano.transpose()*localToCamera*inputBasis;
    return (model.keypoints.rowwise()-model.keypoints.row(0))*orientation;
}
Matrix KeypointsToMano::getFaces(bool cameraOriented) const {
    Matrix faces=model.faces;
    if(cameraOriented && mirroredInput) faces=faces.rowwise().reverse().eval();
    return faces;
}
SkeletonPose KeypointsToMano::getSkeleton(const std::string& space) const {
    if(space!="camera" && space!="model") throw std::invalid_argument("Expected camera or model space");
    if(!hasPose) throw std::logic_error("Solve a valid frame before reading skeleton data");
    SkeletonPose snapshot;
    snapshot.side=left?"left":"right"; snapshot.space=space;
    snapshot.parents=model.parents; snapshot.transforms=model.jointTransforms;
    snapshot.restPositions=model.joints*1000; snapshot.fitErrorMm=fitErrorMm;
    snapshot.mirroredLocalAxes=space=="camera" && mirroredInput;
    if(space=="camera") {
        Mat3 basis=inputBasis*(localToMano.transpose()*localToCamera).transpose();
        for(auto& t:snapshot.transforms) {
            t.topLeftCorner<3,3>()=(basis*t.topLeftCorner<3,3>()*inputBasis).eval();
            t.topRightCorner<3,1>()=(basis*(t.topRightCorner<3,1>()-model.keypoints.row(0).transpose())).eval();
        }
        snapshot.restPositions=((snapshot.restPositions.rowwise()-snapshot.restPositions.row(0))*basis.transpose()).eval();
    }
    return snapshot;
}

HandState::HandState(const std::string& model,int side,int iterations,double smoothing,PositionFilterConfig position,bool mirroredInput):
    converter(model,side==0,iterations,smoothing,45,mirroredInput),side(side),
    positionFilter(position.minCutoff,position.beta*300,position.derivativeCutoff,position.medianWindow,position.maxSpeed/300),
    scenePositionFilter(position.minCutoff,position.beta,position.derivativeCutoff,position.medianWindow,position.maxSpeed),
    sceneReferenceKeypoints(converter.model.keypoints) {}
void HandState::beginTrack() {
    ikTimings.clear();
    nextIkSubmitAt=-inf;
    sceneMmPerPixel.reset(); scenePositionFilter.reset(); sceneTranslation.reset();
    filter.reset(); positionFilter.reset(); result.reset();
    if(!future.valid()) converter.reset(); else discard=true;
}
void HandState::deactivate() {
    ikTimings.clear();
    nextIkSubmitAt=-inf;
    sceneMmPerPixel.reset(); scenePositionFilter.reset(); sceneTranslation.reset();
    lastSeen=-inf; screenWrist.reset(); displayWrist.reset(); pending.reset(); result.reset();
    candidate=-1; candidateFrames=0; filter.reset(); positionFilter.reset();
    if(future.valid()) discard=true;
}
Matrix HandState::update(const Detection& d,double time) {
    if(time-lastSeen>.35) beginTrack();
    Matrix filtered=filter(d.world,time); pending=filtered; lastSeen=time; screenWrist=d.wrist();
    Matrix wrist(1,2); wrist.row(0)=d.wrist().transpose(); displayWrist=positionFilter(wrist,time).row(0).transpose();
    return filtered;
}
bool HandState::collect() {
    if(!future.valid() || future.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return false;
    ++ikCompletes;
    try { auto snapshot=future.get(); if(!discard) {
        if(std::isfinite(snapshot.ikSeconds) && snapshot.ikSeconds>0) {
            ikTimings.push_back(snapshot.ikSeconds);
            if(ikTimings.size()>20) ikTimings.pop_front();
        }
        result=std::move(snapshot); ++resultVersion;
    } }
    catch(const std::exception& e) { std::cerr << (side==0?"left":"right") << " hand IK failed: " << e.what() << '\n'; }
    if(discard) converter.reset();
    discard=false;
    return true;
}
std::optional<SkeletonPose> HandState::getSkeleton(const std::string& space) const {
    if(space!="camera" && space!="scene") throw std::invalid_argument("Expected camera or scene space");
    if(!result) return std::nullopt;
    if(space=="camera") return result->skeleton;
    if(!sceneTranslation) return std::nullopt;
    return result->skeleton.toScene(*sceneTranslation);
}
bool HandState::submit(double time,double maxFps) {
    if(future.valid() || !pending) return false;
    if(maxFps>0 && time<nextIkSubmitAt) return false;
    Matrix latest=std::move(*pending); pending.reset();
    if(maxFps>0) nextIkSubmitAt=time+1/maxFps;
    ++ikSubmits;
    double minimumCycleSeconds=maxFps>0?1/maxFps:0;
    future=solverWorker.submit([this,latest=std::move(latest),minimumCycleSeconds] {
        auto start=std::chrono::steady_clock::now();
        converter.solve(latest);
        if(minimumCycleSeconds>0) {
            auto minimumCycle=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(minimumCycleSeconds));
            std::this_thread::sleep_until(start+minimumCycle);
        }
        double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        auto snapshot=converter.cameraOriented(); snapshot.ikSeconds=seconds; return snapshot;
    });
    return true;
}
void HandState::updateScenePosition(const Detection& d,double depth,int width,int height,double time,std::optional<double> mmPerPixel) {
    if(!result) return;
    if(mmPerPixel) {
        if(std::isfinite(*mmPerPixel) && *mmPerPixel>0) sceneMmPerPixel=mmPerPixel;
        else {
            mmPerPixel=sceneMmPerPixel;
            if(!mmPerPixel) { if(sceneTranslation) sceneTranslation->z()=depth; return; }
        }
    }
    auto position=screenTranslation(d.screen,result->keypoints,width,height,depth,mmPerPixel);
    if(position) {
        Matrix xy(1,2); xy.row(0)=position->head<2>().transpose();
        position->head<2>()=scenePositionFilter(xy,time).row(0).transpose(); sceneTranslation=position;
    } else if(sceneTranslation) sceneTranslation->z()=depth;
}
std::map<int,Detection> HandednessResolver::resolve(std::vector<Detection> raw,
    const std::array<std::unique_ptr<HandState>,2>& states,double time) {
    std::map<int,Detection> resolved;
    resolvedOrder.clear();
    std::set<int> usedDetections,usedStates;
    std::vector<std::tuple<double,int,int>> pairs;
    std::set<int> activeSides;
    for(int side=0;side<2;++side) {
        auto& s=*states[side];
        if(s.screenWrist && time-s.lastSeen>=trackTimeout) s.deactivate();
        if(s.screenWrist && time-s.lastSeen<trackTimeout) activeSides.insert(side);
    }
    for(int i=0;i<int(raw.size());++i) for(int side=0;side<2;++side) {
        auto& s=*states[side];
        if(activeSides.count(side)) pairs.emplace_back((raw[i].wrist()-*s.screenWrist).norm(),i,side);
    }
    std::sort(pairs.begin(),pairs.end());
    for(auto [distance,index,side]:pairs) {
        if(distance>.30) break;
        if(usedDetections.count(index) || usedStates.count(side)) continue;
        auto& d=raw[index];
        d.stabilized=d.rawSide!=side; resolved[side]=d;
        resolvedOrder.push_back(side);
        usedDetections.insert(index); usedStates.insert(side);
    }
    std::map<int,Detection> candidates;
    std::vector<int> candidateOrder;
    for(int i=0;i<int(raw.size());++i) {
        if(usedDetections.count(i)) continue;
        auto d=raw[i]; int side=d.rawSide;
        if(resolved.count(side) || activeSides.count(side)) continue;
        if(raw.size()==1 && !activeSides.empty()) continue;
        if(!candidates.count(side)) { candidates[side]=d; candidateOrder.push_back(side); }
        else if(d.score>candidates[side].score) candidates[side]=d;
    }
    std::map<int,Candidate> next;
    for(int side:candidateOrder) {
        auto& d=candidates.at(side);
        auto previous=pending.find(side);
        bool continuous=previous!=pending.end() && time-previous->second.time<trackTimeout && (d.wrist()-previous->second.wrist).norm()<=.30;
        int count=continuous?previous->second.count+1:1;
        if(count>=confirmFrames) { d.stabilized=false; resolved[side]=d; resolvedOrder.push_back(side); }
        else next[side]={d.wrist(),count,time};
    }
    pending=std::move(next);
    return resolved;
}
void DepthButton::update(const std::map<int,Vec3>& input) {
    std::map<int,Vec3> points;
    for(const auto& [side,p]:input) if(p.allFinite()) points[side]=p;
    std::set<int> nextContacts;
    for(const auto& [side,point]:points) {
        Vec3 local=point-center;
        double margin=tipRadius+(contacts.count(side)?3:0);
        bool inside=std::abs(local.x())<=width/2+margin && std::abs(local.y())<=height/2+margin;
        bool hit;
        if(contacts.count(side)) hit=inside && -margin<=local.z() && local.z()<=travel+10+margin;
        else {
            hit=inside && std::abs(local.z())<=tipRadius;
            if(previous.count(side)) {
                Vec3 start=previous.at(side)-center; double front=-tipRadius;
                if(start.z()<front && front<=local.z()) {
                    double fraction=(front-start.z())/(local.z()-start.z());
                    Vec3 crossing=start+fraction*(local-start);
                    hit=hit || (std::abs(crossing.x())<=width/2+tipRadius && std::abs(crossing.y())<=height/2+tipRadius);
                }
            }
        }
        if(hit) nextContacts.insert(side);
    }
    bool nextPressed=!nextContacts.empty();
    if(nextPressed && !pressed) ++pressCount;
    pressed=nextPressed; contacts=std::move(nextContacts); previous=std::move(points);
}
bool Pinch::update(const Matrix& world,double time,const Ball& ball,double tolerance,int nextSequence,
    const std::optional<Vec3>& scenePoint) {
    lastSeen=time; distance=(world.row(4)-world.row(8)).norm();
    if(scenePoint) point=scenePoint;
    handFrame=palmFrame(world);
    if(!pinching && scenePoint && distance<=enter) {
        pinching=true; sequence=nextSequence; controls=ball.contains(*point,tolerance); return true;
    }
    if(pinching && distance>=exit) release();
    return false;
}
void BallController::update(std::array<Pinch,2>& states) {
    std::vector<int> controllers;
    for(int s=0;s<2;++s) if(states[s].pinching && states[s].controls) controllers.push_back(s);
    std::sort(controllers.begin(),controllers.end(),[&](int a,int b){return states[a].sequence<states[b].sequence;});
    std::vector<std::pair<int,int>> next;
    for(int s:controllers) next.emplace_back(s,states[s].sequence);
    if(next!=signature) {
        signature=next;
        if(!controllers.empty()) {
            auto& first=states[controllers[0]];
            center=ball.center; rotation=ball.rotation; scale=ball.scale;
            firstPoint=*first.point; firstFrame=first.handFrame;
            if(controllers.size()==2) {
                twoVector=*states[controllers[1]].point-firstPoint; twoDistance=std::max(twoVector.norm(),1e-6);
            }
        }
    }
    if(controllers.size()==1) {
        auto& first=states[controllers[0]];
        ball.center=center+*first.point-firstPoint; ball.rotation=first.handFrame*firstFrame.transpose()*rotation;
    } else if(controllers.size()==2) {
        auto& first=states[controllers[0]];
        Vec3 vector=*states[controllers[1]].point-*first.point;
        double newScale=std::clamp(scale*std::pow(std::max(vector.norm(),1e-6)/twoDistance,sensitivity),minimum,maximum);
        Mat3 delta=rotationBetween(twoVector,vector);
        ball.center=*first.point+delta*((center-firstPoint)*(newScale/scale));
        ball.rotation=delta*rotation; ball.scale=newScale;
    }
}
} // namespace m2m

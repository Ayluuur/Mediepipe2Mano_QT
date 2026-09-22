#include "core.h"
#include "skeleton_json.h"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <iostream>
#include <stdexcept>
using namespace m2m;
int checks=0; double worstMesh=0;
Matrix matrix(QJsonValue value) {
    auto a=value.toArray(); bool nested=a[0].isArray();
    Matrix m(nested?a.size():1,nested?a[0].toArray().size():a.size());
    for(int r=0;r<m.rows();++r) for(int c=0;c<m.cols();++c)
        m(r,c)=nested?a[r].toArray()[c].toDouble():a[c].toDouble();
    return m;
}
void require(bool ok,const std::string& name) {
    if(!ok) throw std::runtime_error(name); ++checks;
}
void near(const Matrix& a,const Matrix& b,double tol,const std::string& name) {
    require(a.rows()==b.rows() && a.cols()==b.cols() && a.allFinite(),name+" shape/finite");
    double error=(a-b).cwiseAbs().maxCoeff();
    require(error<=tol,name+" error="+std::to_string(error));
    if(name=="mesh") worstMesh=std::max(worstMesh,error);
}
void skeleton(const SkeletonPose& s,const QJsonObject& expected) {
    auto json=skeletonToJson(s);
    require(json.keys()==expected.keys(),"JSON schema matches Python");
    auto roundtrip=QJsonDocument::fromJson(QJsonDocument(json).toJson()).object();
    near(matrix(roundtrip["positions"]),s.positions(),1e-12,"JSON positions roundtrip");
    require(s.side==expected["side"].toString().toStdString(),"side");
    require(s.space==expected["space"].toString().toStdString(),"space");
    require(s.mirroredLocalAxes==expected["mirrored_local_axes"].toBool(),"mirror axes");
    require(std::abs(s.fitErrorMm-expected["fit_error_mm"].toDouble())<1e-5,"fit error");
    near(s.positions(),matrix(expected["positions"]),1e-4,"positions");
    near(s.restPositions,matrix(expected["rest_positions"]),1e-7,"rest positions");
    near(s.displacements(),matrix(expected["displacements"]),1e-4,"displacements");
    near(s.localPositions(),matrix(expected["local_positions"]),1e-4,"local positions");
    near(s.localAxisAngles(),matrix(expected["local_axis_angles"]),1e-5,"axis angles");
    auto local=s.localTransforms(); auto rotations=s.rotations(); auto localRotations=s.localRotations();
    Matrix quats=s.quaternions(),localQuats=s.localQuaternions();
    for(int j=0;j<16;++j) {
        require(s.names[j]==expected["names"].toArray()[j].toString().toStdString(),"joint name");
        require(s.parents[j]==expected["parents"].toArray()[j].toInt(),"parent");
        near(s.transforms[j],matrix(expected["transforms"].toArray()[j]),1e-4,"transform");
        near(local[j],matrix(expected["local_transforms"].toArray()[j]),1e-4,"local transform");
        near(rotations[j],matrix(expected["rotations"].toArray()[j]),1e-5,"rotation");
        near(localRotations[j],matrix(expected["local_rotations"].toArray()[j]),1e-5,"local rotation");
        for(bool isLocal:{false,true}) {
            Eigen::Vector4d q=(isLocal?localQuats:quats).row(j).transpose();
            Eigen::Vector4d wanted=matrix(expected[isLocal?"local_quaternions":"quaternions"].toArray()[j]).transpose();
            require(std::min((q-wanted).norm(),(q+wanted).norm())<1e-5,"quaternion (sign equivalent)");
        }
        require(std::abs(rotations[j].determinant()-1)<1e-9,"SO(3)");
        if(j) near(s.transforms[s.parents[j]]*local[j],s.transforms[j],1e-9,"hierarchy");
    }
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        require(argc==2,"Usage: skeleton_parity_tests project-root");
        QString root=QString::fromLocal8Bit(argv[1]); QFile file(root+"/tests/skeleton_reference.json");
        require(file.open(QIODevice::ReadOnly),"Missing skeleton fixtures");
        auto data=QJsonDocument::fromJson(file.readAll()).object();
        auto path=[&](bool left) {return (root+"/models/MANO_"+(left?"LEFT":"RIGHT")+".bin").toStdString();};
        for(bool left:{false,true}) {
            ManoModel batch(path(left)), scalar(path(left));
            Matrix originalVertices=batch.vertices, originalKeypoints=batch.keypoints;
            for(int dimensions:{1,17,45}) for(int count:{0,1,8,45}) {
                Matrix poses(count,dimensions);
                for(int b=0;b<count;++b) for(int k=0;k<dimensions;++k)
                    poses(b,k)=.4*std::sin(.3*b+.7*k);
                auto samples=batch.keypointsBatch(poses);
                require(samples.rows()==count && samples.cols()==63,"batch shape");
                for(int b=0;b<count;++b) {
                    scalar.setPose(poses.row(b).transpose(),true);
                    Matrix expected=Eigen::Map<const Matrix>(scalar.keypoints.data(),1,63);
                    near(samples.row(b),expected,1e-9,"batched FK versus scalar");
                }
                near(batch.vertices,originalVertices,0,"batch does not mutate mesh");
                near(batch.keypoints,originalKeypoints,0,"batch does not mutate keypoints");
            }
        }
        for(auto value:data["models"].toArray()) {
            auto f=value.toObject(); ManoModel model(path(f["side"].toString()=="left"));
            model.setAbsolutePose(matrix(f["pose"]));
            near(model.vertices,matrix(f["vertices"]),1e-9,"FK mesh");
            near(model.keypoints,matrix(f["keypoints"]),1e-9,"FK keypoints");
            for(int j=0;j<16;++j) near(model.jointTransforms[j],matrix(f["transforms"].toArray()[j]),1e-9,"FK transform");
            Matrix vertices=model.vertices;
            model.setAbsolutePose(matrix(f["pose"]),true);
            near(model.keypoints,matrix(f["keypoints"]),1e-9,"tips-only keypoints");
            near(model.vertices,vertices,0,"tips-only preserves vertices");
        }
        bool rateLimitTested=false;
        for(auto value:data["hands"].toArray()) {
            auto f=value.toObject(); bool left=f["side"].toString()=="left",mirrored=f["mirrored"].toBool();
            KeypointsToMano ik(path(left),left,5,1,f["n_pose"].toInt(),mirrored);
            auto world=matrix(f["world"]);
            near(ik.solve(world).transpose(),matrix(f["pose"]),1e-5,"PCA");
            auto frame=ik.cameraOriented();
            near(frame.vertices,matrix(f["vertices"]),1e-4,"mesh");
            near(frame.keypoints,matrix(f["keypoints"]),1e-4,"keypoints");
            KeypointsToMano parallel(path(left),left,5,1,f["n_pose"].toInt(),mirrored);
            parallel.setJacobianWorkers(8);
            near(parallel.solve(world).transpose(),matrix(f["pose"]),1e-5,"parallel PCA");
            near(parallel.cameraVertices(),frame.vertices,1e-4,"single/multi batch mesh");
            KeypointsToMano scalar(path(left),left,5,1,f["n_pose"].toInt(),mirrored);
            scalar.setJacobianWorkers(0); scalar.solve(world);
            near(scalar.cameraVertices(),frame.vertices,1e-4,"scalar/single batch mesh");
            parallel.solve(world); // Warm state and persistent workers can be reused.
            require(parallel.peakJacobianConcurrency()>1,"Jacobian workers did not overlap");
            near(ik.getFaces(true),matrix(f["faces"]),0,"winding");
            if(f["n_pose"].toInt()==45) {
                double error=std::sqrt((frame.keypoints-matrix(f["target"])).squaredNorm()/21);
                require(error<(f["amount"].toDouble()==0?1e-7:3.),"flat/curled fit");
            }
            skeleton(frame.skeleton,f["camera"].toObject());
            skeleton(ik.getSkeleton("model"),f["model"].toObject());
            skeleton(frame.skeleton.toScene(Vec3(20,-30,100)),f["scene"].toObject());
            auto before=ik.getSkeleton();
            std::vector<Matrix> invalid={Matrix::Zero(20,3),Matrix::Constant(21,3,inf),world,world};
            invalid[2].row(2)=world.row(1);
            invalid[3].row(17)=world.row(0)+2*(world.row(5)-world.row(0));
            for(auto& bad:invalid) {
                bool rejected=false; try {ik.solve(bad);} catch(const std::exception&) {rejected=true;}
                require(rejected,"invalid input rejected");
                near(ik.getSkeleton().positions(),before.positions(),0,"invalid keeps snapshot");
            }
            frame.skeleton.transforms[0].setZero();
            near(ik.getSkeleton().positions(),before.positions(),0,"detached snapshot");
            ik.reset(); bool reset=false; try {ik.getSkeleton();} catch(const std::logic_error&) {reset=true;}
            require(reset,"reset invalidates skeleton");
            if(f["n_pose"].toInt()!=45) continue;
            HandState hand(path(left),left?0:1,5,1,{},mirrored);
            require(!hand.getSkeleton(),"initial async empty");
            hand.pending=world; hand.submit(); hand.future.wait(); hand.collect();
            require(hand.getSkeleton().has_value(),"async skeleton collected");
            require(hand.resultVersion==1,"one accepted MANO update");
            require(hand.result->ikSeconds>0 && hand.meanIkSeconds()>0,"worker measures IK duration");
            require(hand.ikTimings.size()==1,"one timing per accepted solve");
            hand.collect();
            require(hand.resultVersion==1,"reusing snapshot is not a MANO update");
            require(hand.ikTimings.size()==1,"repeated collection does not duplicate timing");
            near(hand.getSkeleton()->positions(),hand.result->keypoints.topRows(16),1e-9,"same-frame bones");
            require(!hand.getSkeleton("scene"),"missing scene translation");
            hand.sceneTranslation=Vec3(20,-30,100);
            skeleton(*hand.getSkeleton("scene"),f["scene"].toObject());
            hand.pending=world; hand.submit(); hand.beginTrack(); hand.future.wait(); hand.collect();
            require(!hand.getSkeleton(),"discard clears snapshot");
            require(hand.resultVersion==1,"discarded work is not a MANO update");
            require(hand.ikTimings.empty(),"track reset and discarded work clear timing");
            bool cleared=false; try {hand.converter.getSkeleton();} catch(const std::logic_error&) {cleared=true;}
            require(cleared,"discard resets solver");
            hand.pending=world; hand.submit(); hand.future.wait(); hand.collect();
            near(hand.result->vertices,matrix(f["vertices"]),1e-4,"reacquired mesh");
            require(hand.resultVersion==2,"reacquired result increments update count");
            hand.deactivate(); require(!hand.getSkeleton(),"loss clears skeleton");
            if(!rateLimitTested) {
                HandState limited(path(left),left?0:1,5,1,{},mirrored);
                limited.pending=world; require(limited.submit(10,120),"first rate-limited IK submits");
                limited.future.wait(); limited.collect();
                limited.pending=world; require(!limited.submit(10.001,200),"IK rate limit rejects early submit");
                require(limited.pending.has_value(),"rate limit preserves latest pending frame");
                require(limited.submit(10.009,120),"IK rate limit accepts due submit");
                limited.future.wait(); limited.collect();
                require(limited.ikSubmits==2 && limited.ikCompletes==2,"IK submit/complete counters");
                require(limited.meanIkSeconds()>=.0082,"IK worker cycle respects 120 FPS limit");
                rateLimitTested=true;
            }
        }
        std::cout<<"PASS "<<checks<<" skeleton/FK/IK checks; max mesh error "<<worstMesh<<" mm\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1;}
}

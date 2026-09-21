#include "core.h"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <iostream>
#include <stdexcept>
using namespace m2m;
Matrix matrix(QJsonValue v) {
    auto a=v.toArray();
    bool nested=a[0].isArray();
    Matrix m(nested?a.size():1,nested?a[0].toArray().size():a.size());
    for(int i=0;i<m.rows();++i) for(int j=0;j<m.cols();++j)
        m(i,j)=nested?a[i].toArray()[j].toDouble():a[j].toDouble();
    return m;
}
int checks=0; double worstMesh=0;
void near(const Matrix& a,const Matrix& b,double tolerance,const std::string& name) {
    if(a.rows()!=b.rows() || a.cols()!=b.cols() || !a.allFinite()) throw std::runtime_error(name+": invalid shape/value");
    double error=(a-b).cwiseAbs().maxCoeff();
    if(error>tolerance) throw std::runtime_error(name+": max error "+std::to_string(error));
    if(name=="camera vertices") worstMesh=std::max(worstMesh,error);
    ++checks;
}
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); ++checks; }
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        if(argc!=2) throw std::runtime_error("Usage: parity_tests project-root");
        QString root=QString::fromLocal8Bit(argv[1]);
        QFile file(root+"/tests/reference.json");
        if(!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Missing Python reference fixtures");
        auto reference=QJsonDocument::fromJson(file.readAll()).object();
        for(auto value:reference["hands"].toArray()) {
            auto h=value.toObject(); bool left=h["side"].toString()=="left";
            KeypointsToMano ik((root+"/models/MANO_"+(left?"LEFT":"RIGHT")+".bin").toStdString(),left);
            near(ik.model.vertices,matrix(h["rest_vertices"]),1e-9,"rest vertices");
            near(ik.model.keypoints,matrix(h["rest_keypoints"]),1e-9,"rest keypoints");
            for(auto frame:h["frames"].toArray()) {
                auto f=frame.toObject();
                if(f["reset"].toBool()) ik.reset();
                Matrix world=matrix(f["world"]);
                near(ik.solve(world).transpose(),matrix(f["pose"]),1e-5,"PCA pose");
                auto snapshot=ik.cameraOriented();
                near(snapshot.vertices,matrix(f["vertices"]),1e-4,"camera vertices");
                near(snapshot.keypoints,matrix(f["keypoints"]),1e-4,"camera keypoints");
                near(palmFrame(world),matrix(f["palm_frame"]),1e-12,"palm frame");
            }
        }
        OneEuro filter;
        for(auto value:reference["filter"].toArray()) {
            auto f=value.toObject(); near(filter(matrix(f["input"]),f["time"].toDouble()),matrix(f["output"]),1e-12,"One Euro");
        }
        DepthEstimator depth;
        for(auto value:reference["depth"].toArray()) {
            auto f=value.toObject();
            require(std::abs(depth.update(matrix(f["input"]),f["time"].toDouble(),640,480)-f["output"].toDouble())<1e-10,"Depth differs");
            require(std::abs(depth.progress()-f["progress"].toDouble())<1e-12,"Calibration progress differs");
        }
        for(auto value:reference["rotations"].toArray()) {
            auto f=value.toObject(); near(rotationBetween(matrix(f["a"]).transpose(),matrix(f["b"]).transpose()),matrix(f["output"]),1e-12,"rotation between");
        }
        Ball referenceBall(35,Vec3(0,0,200)); BallController referenceController(referenceBall,.35,5,.8);
        std::array<Pinch,2> referencePinches;
        for(auto value:reference["controller"].toArray()) {
            auto f=value.toObject(); auto inputs=f["inputs"].toArray();
            for(int side=0;side<2;++side) {
                auto input=inputs[side].toObject(); auto& s=referencePinches[side];
                s.pinching=s.controls=input["active"].toBool(); s.sequence=input["sequence"].toInt();
                s.point=matrix(input["point"]).transpose(); s.handFrame=matrix(input["frame"]);
            }
            referenceController.update(referencePinches);
            near(referenceBall.center.transpose(),matrix(f["center"]),1e-10,"Python ball center");
            near(referenceBall.rotation,matrix(f["rotation"]),1e-12,"Python ball rotation");
            require(std::abs(referenceBall.scale-f["scale"].toDouble())<1e-12,"Python scale/clamping differs");
        }
        std::array<std::unique_ptr<HandState>,2> handStates;
        HandednessResolver resolver(3);
        for(int side=0;side<2;++side) handStates[side]=std::make_unique<HandState>((root+"/models/MANO_"+(side==0?"LEFT":"RIGHT")+".bin").toStdString(),side,5,.7);
        for(auto value:reference["handedness"].toArray()) {
            auto f=value.toObject(); double time=f["time"].toDouble(); std::vector<Detection> raw;
            for(auto v:f["raw"].toArray()) {
                auto d=v.toObject(); Detection detection;
                detection.rawSide=d["side"].toString()=="left"?0:1; detection.score=d["score"].toDouble();
                detection.screen=Matrix::Zero(21,3); detection.screen(0,0)=d["x"].toDouble(); detection.screen(0,1)=.5;
                raw.push_back(detection);
            }
            auto resolved=resolver.resolve(raw,handStates,time); auto expected=f["resolved"].toArray();
            require(resolved.size()==size_t(expected.size()),"Handedness count differs");
            for(auto v:expected) {
                auto d=v.toObject(); int side=d["side"].toString()=="left"?0:1;
                require(resolved.count(side) && resolved.at(side).stabilized==d["locked"].toBool(),"Handedness lock differs");
                require(std::abs(resolved.at(side).wrist().x()-d["x"].toDouble())<1e-12,"Handedness association differs");
            }
            for(auto& [side,d]:resolved) { handStates[side]->screenWrist=d.wrist(); handStates[side]->lastSeen=time; }
        }
        auto& asynchronous=*handStates[0];
        auto inputFrame=reference["hands"].toArray()[0].toObject()["frames"].toArray()[0].toObject();
        asynchronous.pending=matrix(inputFrame["world"]); asynchronous.submit(); asynchronous.deactivate();
        asynchronous.future.wait(); asynchronous.collect();
        require(!asynchronous.result && !asynchronous.future.valid(),"Retired track accepted stale IK result");
        asynchronous.beginTrack(); asynchronous.pending=matrix(inputFrame["world"]); asynchronous.submit();
        asynchronous.future.wait(); asynchronous.collect();
        require(asynchronous.result.has_value(),"Reacquired track lost IK result");
        near(asynchronous.result->keypoints,matrix(inputFrame["keypoints"]),1e-4,"Async snapshot keypoints");
        // Behavioral checks exercise hysteresis, timeout, acquisition, and mode changes.
        Ball ball(35,Vec3(0,0,200)); Pinch pinch; Matrix world=Matrix::Zero(21,3);
        world.row(5)<<-.02,-.05,0; world.row(9)<<0,-.06,0; world.row(17)<<.02,-.05,0;
        world.row(4)<<0,0,0; world.row(8)<<.04,0,0;
        require(!pinch.update(world,0,ball,8,1,std::nullopt),"Acquired without IK snapshot");
        require(pinch.update(world,.03,ball,8,1,Vec3(0,0,200)) && pinch.controls,"Inside pinch not acquired");
        world(8,0)=.05; pinch.update(world,.06,ball,8,2,Vec3(0,0,200));
        require(pinch.pinching,"Hysteresis released too early");
        world(8,0)=.056; pinch.update(world,.09,ball,8,2,Vec3(0,0,200));
        require(!pinch.pinching,"Pinch did not release");
        world(8,0)=.04; pinch.update(world,.12,ball,8,3,Vec3(500,0,200));
        require(pinch.pinching && !pinch.controls,"Outside pinch acquired ball");
        pinch.update(world,.15,ball,8,4,Vec3(0,0,200));
        require(!pinch.controls,"Outside pinch acquired by moving inside");
        pinch.expire(.34); require(!pinch.pinching,"Missing timeout failed");
        std::array<Pinch,2> states;
        states[0].pinching=states[0].controls=true; states[0].sequence=1; states[0].point=Vec3(0,0,200);
        BallController controller(ball,.35,5,.8); controller.update(states);
        states[0].point=Vec3(20,10,210); controller.update(states);
        near(ball.center,Vec3(20,10,210),1e-12,"one hand translation");
        states[1].pinching=states[1].controls=true; states[1].sequence=2; states[1].point=Vec3(40,10,210);
        controller.update(states); near(ball.center,Vec3(20,10,210),1e-12,"two hand transition");
        states[1].point=Vec3(60,10,210); controller.update(states);
        require(std::abs(ball.scale-std::pow(2.,.8))<1e-12,"two hand scaling failed");
        states[1].release(); controller.update(states);
        auto oldCenter=ball.center; states[0].point=Vec3(25,10,210); controller.update(states);
        near(ball.center,oldCenter+Vec3(5,0,0),1e-12,"two to one hand transition");
        std::cout<<"PASS "<<checks<<" checks; maximum MANO vertex error = "<<worstMesh<<" mm\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
}

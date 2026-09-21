#include "core.h"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <open3d/geometry/TriangleMesh.h>
#include <iostream>
#include <stdexcept>
using namespace m2m;
namespace {
int checks=0;
double number(QJsonValue value) { return value.isNull()?std::numeric_limits<double>::quiet_NaN():value.toDouble(); }
Matrix matrix(QJsonValue value) {
    auto a=value.toArray(); bool nested=a[0].isArray();
    Matrix result(nested?a.size():1,nested?a[0].toArray().size():a.size());
    for(int i=0;i<result.rows();++i) for(int j=0;j<result.cols();++j)
        result(i,j)=number(nested?a[i].toArray()[j]:a[j]);
    return result;
}
void require(bool condition,const std::string& name) { if(!condition) throw std::runtime_error(name); ++checks; }
void near(const Matrix& actual,const Matrix& expected,double tolerance,const std::string& name) {
    require(actual.rows()==expected.rows() && actual.cols()==expected.cols() && actual.allFinite(),name+" invalid shape/value");
    double error=(actual-expected).cwiseAbs().maxCoeff();
    require(error<=tolerance,name+" max error "+std::to_string(error));
}
void scalar(double a,double b,const std::string& name) { require(std::isfinite(a) && std::abs(a-b)<1e-9,name); }
int side(const QString& s) { return s=="left"?0:1; }
QString sideName(int s) { return s==0?"left":"right"; }
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        if(argc!=2) throw std::runtime_error("Usage: sync_parity_tests root");
        QString root=QString::fromLocal8Bit(argv[1]);
        QFile file(root+"/tests/sync_reference.json");
        if(!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Missing sync reference fixtures");
        QJsonParseError error; auto document=QJsonDocument::fromJson(file.readAll(),&error);
        require(error.error==QJsonParseError::NoError,"Invalid reference JSON"); auto data=document.object();
        for(auto value:data["filters"].toArray()) {
            auto series=value.toObject(); auto o=series["options"].toObject();
            OneEuro filter(o["min_cutoff"].toDouble(1.2),o["beta"].toDouble(4),1,o["median_window"].toInt(1),
                o.contains("max_speed")?std::optional<double>(o["max_speed"].toDouble()):std::nullopt);
            for(auto f:series["frames"].toArray()) {
                auto frame=f.toObject(); if(frame["reset"].toBool()) filter.reset();
                near(filter(matrix(frame["input"]),frame["time"].toDouble()),matrix(frame["output"]),1e-10,"median/speed filter");
            }
        }
        for(int window:{0,2,-1}) { bool rejected=false; try { OneEuro filter(1.5,.005,1,window); } catch(const std::invalid_argument&) { rejected=true; } require(rejected,"Invalid median window accepted"); }
        DepthConfig dc; dc.reference=200; dc.maximum=600; MultiHandDepthEstimator depth(dc);
        for(auto value:data["depth_tracker"].toArray()) {
            auto f=value.toObject(); double t=f["time"].toDouble();
            if(f["request"].toBool()) depth.requestCalibration(t);
            auto input=f["sizes"].toObject(),expected=f["output"].toObject(); std::map<int,double> sizes;
            for(auto it=input.begin();it!=input.end();++it) sizes[side(it.key())]=number(it.value());
            auto actual=depth.update(sizes,t); require(actual.size()==size_t(expected.size()),"Depth output count");
            for(auto it=expected.begin();it!=expected.end();++it) scalar(actual.at(side(it.key())),it.value().toDouble(),"Shared/manual depth");
            for(int s=0;s<2;++s) scalar(depth.estimators[s].progress(),f["progress"].toArray()[s].toDouble(),"Depth calibration progress");
            require(depth.calibrationStatus(t)==f["status"].toString().toStdString(),"Calibration status at "+std::to_string(t));
        }
        DepthButton button(Vec3(-150,0,350),90,60,20,10);
        auto cap=open3d::geometry::TriangleMesh::CreateBox(90,60,10); cap->Translate(Vec3(-45,-30,0));
        for(auto value:data["button"].toArray()) {
            auto f=value.toObject(); auto input=f["points"].toObject(); std::map<int,Vec3> points;
            for(auto it=input.begin();it!=input.end();++it) points[side(it.key())]=matrix(it.value()).transpose();
            button.update(points);
            require(button.pressed==f["pressed"].toBool() && button.pressCount==f["count"].toInt(),"Button pressed/count");
            std::set<int> expected; for(auto v:f["contacts"].toArray()) expected.insert(side(v.toString()));
            require(button.contacts==expected,"Button contact owners");
            Matrix vertices(cap->vertices_.size(),3);
            for(size_t i=0;i<cap->vertices_.size();++i) vertices.row(i)=(cap->vertices_[i]+button.capOffset()).transpose();
            near(vertices,matrix(f["vertices"]),1e-12,"Button cap displacement");
            near(button.capColor().transpose(),matrix(f["color"]),1e-12,"Button color");
        }
        auto model=[&](int s){return (root+"/models/MANO_"+(s==0?"LEFT":"RIGHT")+".bin").toStdString();};
        std::array<std::unique_ptr<HandState>,2> states;
        for(int s=0;s<2;++s) states[s]=std::make_unique<HandState>(model(s),s,5,.7);
        HandednessResolver resolver(3);
        for(auto value:data["presence"].toArray()) {
            auto f=value.toObject(); std::vector<Detection> raw; double t=f["time"].toDouble();
            if(f["reset"].toBool()) for(auto& state:states) state->deactivate();
            for(auto v:f["raw"].toArray()) {
                auto input=v.toObject(); Detection d; d.rawSide=side(input["raw_side"].toString()); d.score=input["score"].toDouble();
                d.screen=Matrix::Zero(21,3); d.screen.row(0).head<2>()=matrix(input["wrist"]); raw.push_back(d);
            }
            auto resolved=resolver.resolve(raw,states,t);
            auto expected=f["resolved"].toObject();
            require(resolved.size()==size_t(expected.size()),"Presence output count");
            auto order=f["order"].toArray(); require(resolver.resolvedOrder.size()==size_t(order.size()),"Detection order count");
            for(int i=0;i<order.size();++i) require(resolver.resolvedOrder[i]==side(order[i].toString()),"Simultaneous hand acquisition order");
            for(const auto& [s,d]:resolved) {
                require(expected.contains(sideName(s)),"Presence side"); auto e=expected[sideName(s)].toObject();
                require(d.stabilized==e["label_was_stabilized"].toBool(),"Identity lock");
                near(d.wrist().transpose(),matrix(e["wrist"]),1e-12,"Spatial association");
                states[s]->screenWrist=d.wrist(); states[s]->lastSeen=t;
            }
            std::set<int> active; for(auto v:f["active"].toArray()) active.insert(side(v.toString()));
            for(int s=0;s<2;++s) require(states[s]->screenWrist.has_value()==bool(active.count(s)),"Track retirement");
        }
        for(auto value:data["palm_scale"].toArray()) {
            auto series=value.toObject(); Matrix reference=matrix(series["reference"]); std::map<int,double> corrections;
            for(auto v:series["frames"].toArray()) {
                auto f=v.toObject(); Matrix screen=matrix(f["screen"]);
                auto scale=palmScale(screen,matrix(f["world"]),reference,640,480,&corrections);
                require(scale.has_value()!=f["scale"].isNull(),"Palm scale validity");
                if(scale) scalar(*scale,number(f["scale"]),"Palm scale orientation/magnification");
                auto expected=f["corrections"].toObject(); require(corrections.size()==size_t(expected.size()),"Correction count");
                for(auto it=expected.begin();it!=expected.end();++it) scalar(corrections.at(it.key().toInt()),it.value().toDouble(),"Learned segment correction");
                auto translation=screenTranslation(screen,reference,640,480,75,scale);
                require(translation.has_value()!=f["translation"].isNull(),"Scene translation validity");
                if(translation) near(translation->transpose(),matrix(f["translation"]),1e-9,"Metric wrist translation");
            }
        }
        HandState hand(model(0),0,5,.7); auto scene=data["scene_position"].toObject();
        hand.result=SolveResult{Matrix::Zero(778,3),matrix(scene["reference"])};
        for(auto value:scene["frames"].toArray()) {
            auto f=value.toObject(); Detection d; d.screen=matrix(f["screen"]);
            hand.updateScenePosition(d,f["depth"].toDouble(),640,480,f["time"].toDouble(),number(f["scale"]));
            require(hand.sceneTranslation.has_value(),"Scene position missing");
            near(hand.sceneTranslation->transpose(),matrix(f["translation"]),1e-9,"Metric XY filter/fallback");
            scalar(*hand.sceneMmPerPixel,number(f["cached_scale"]),"Cached palm scale");
        }
        hand.sceneSegmentCorrections[1]=1.2; hand.beginTrack();
        require(!hand.sceneTranslation && !hand.sceneMmPerPixel && hand.sceneSegmentCorrections.at(1)==1.2,"Re-entry scale lifecycle");
        std::cout<<"PASS "<<checks<<" current-Python synchronization checks\n"; return 0;
    } catch(const std::exception& e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
}

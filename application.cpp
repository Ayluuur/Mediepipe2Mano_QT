#include "application.h"
#include "detector.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QTimer>
#include <open3d/Open3D.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif
namespace m2m {
namespace {
#ifdef _WIN32
double processCpuSeconds() {
    FILETIME creation{},exit{},kernel{},user{};
    if(!GetProcessTimes(GetCurrentProcess(),&creation,&exit,&kernel,&user)) return 0;
    ULARGE_INTEGER kernelTicks{},userTicks{};
    kernelTicks.LowPart=kernel.dwLowDateTime; kernelTicks.HighPart=kernel.dwHighDateTime;
    userTicks.LowPart=user.dwLowDateTime; userTicks.HighPart=user.dwHighDateTime;
    return double(kernelTicks.QuadPart+userTicks.QuadPart)/1e7;
}
unsigned int logicalProcessorCount() { return unsigned(std::max<DWORD>(1,GetActiveProcessorCount(ALL_PROCESSOR_GROUPS))); }
#else
double processCpuSeconds() { return 0; }
unsigned int logicalProcessorCount() { return 1; }
#endif
QJsonObject readJson(const QString& path) {
    QFile f(path);
    if(!f.open(QIODevice::ReadOnly)) throw std::runtime_error(("Cannot open configuration: "+path).toStdString());
    QJsonParseError error; auto doc=QJsonDocument::fromJson(f.readAll(),&error);
    if(error.error!=QJsonParseError::NoError || !doc.isObject()) throw std::runtime_error(("Invalid JSON: "+path+": "+error.errorString()).toStdString());
    return doc.object();
}
QJsonObject merge(QJsonObject base,const QJsonObject& extra) {
    for(auto it=extra.begin();it!=extra.end();++it)
        base[it.key()]=it.value().isObject() && base[it.key()].isObject()?QJsonValue(merge(base[it.key()].toObject(),it.value().toObject())):it.value();
    return base;
}
double number(const QJsonObject& c,const char* group,const char* key) {
    const QJsonValue value=c.value(group).toObject().value(key);
    if(!value.isDouble() || !std::isfinite(value.toDouble())) throw std::runtime_error(std::string("Missing/invalid number: ")+group+"."+key);
    return value.toDouble();
}
QString string(const QJsonObject& c,const char* group,const char* key) {
    const QJsonValue value=c.value(group).toObject().value(key);
    if(!value.isString()) throw std::runtime_error(std::string("Missing/invalid string: ")+group+"."+key);
    return value.toString();
}
bool boolean(const QJsonObject& c,const char* group,const char* key) {
    const QJsonValue value=c.value(group).toObject().value(key);
    if(!value.isBool()) throw std::runtime_error(std::string("Missing/invalid boolean: ")+group+"."+key);
    return value.toBool();
}
Vec3 vector3(const QJsonObject& c,const char* group,const char* key) {
    const QJsonValue value=c.value(group).toObject().value(key);
    if(!value.isArray() || value.toArray().size()!=3) throw std::runtime_error(std::string(group)+"."+key+" must contain three finite coordinates");
    Vec3 out; auto a=value.toArray();
    for(int i=0;i<3;++i) {
        if(!a[i].isDouble() || !std::isfinite(a[i].toDouble())) throw std::runtime_error("Nonfinite/invalid center coordinate");
        out[i]=a[i].toDouble();
    }
    return out;
}
PositionFilterConfig positionConfig(const QJsonObject& c) {
    PositionFilterConfig out;
    const QJsonValue value=c.value("tracking").toObject().value("position_filter");
    if(value.isNull() || value.isUndefined()) return out;
    if(!value.isObject()) throw std::runtime_error("tracking.position_filter must be an object");
    auto options=value.toObject();
    for(auto it=options.begin();it!=options.end();++it) {
        if(!it.value().isDouble() || !std::isfinite(it.value().toDouble())) throw std::runtime_error("Invalid position filter number");
        double n=it.value().toDouble(); auto key=it.key();
        if(key=="min_cutoff") out.minCutoff=n;
        else if(key=="beta") out.beta=n;
        else if(key=="derivative_cutoff") out.derivativeCutoff=n;
        else if(key=="max_speed") out.maxSpeed=n;
        else if(key=="median_window") {
            if(n!=std::floor(n) || n<1 || n>100000) throw std::runtime_error("Invalid median_window");
            out.medianWindow=int(n);
        } else throw std::runtime_error(("Unknown position filter option: "+key).toStdString());
    }
    OneEuro validate(out.minCutoff,out.beta,out.derivativeCutoff,out.medianWindow,out.maxSpeed);
    return out;
}
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
DepthConfig depthConfig(const QJsonObject& c) {
    DepthConfig d; d.enabled=boolean(c,"depth_estimation","enabled");
    d.reference=number(c,"depth_estimation","reference_depth");
    d.minimum=number(c,"depth_estimation","minimum_depth"); d.maximum=number(c,"depth_estimation","maximum_depth");
    d.calibrationFrames=int(number(c,"depth_estimation","calibration_frames"));
    d.minCutoff=number(c,"depth_estimation","min_cutoff"); d.beta=number(c,"depth_estimation","beta");
    d.maxSpeed=number(c,"depth_estimation","max_speed"); d.motionGain=number(c,"depth_estimation","motion_gain");
    return d;
}
using Mesh=open3d::geometry::TriangleMesh;
using Visualizer=open3d::visualization::Visualizer;
void setVertices(Mesh& mesh,const Matrix& vertices) {
    mesh.vertices_.resize(vertices.rows());
    for(int i=0;i<vertices.rows();++i) mesh.vertices_[i]=vertices.row(i).transpose();
    mesh.ComputeVertexNormals();
}
void transformMesh(Mesh& mesh,const std::vector<Vec3>& base,const Ball& ball) {
    mesh.vertices_.resize(base.size());
    for(size_t i=0;i<base.size();++i) mesh.vertices_[i]=ball.rotation*(base[i]*ball.scale)+ball.center;
    mesh.ComputeVertexNormals();
}
void configureView(Visualizer& v,const QString& mode) {
    auto& control=v.GetViewControl();
    Vec3 front=mode=="front"?Vec3(0,0,-1):Vec3(.28,-.16,-1).normalized().eval();
    control.SetFront(front); control.SetLookat(Vec3::Zero()); control.SetUp(Vec3(0,1,0)); control.SetZoom(.72);
    control.ChangeFieldOfView(60-control.GetFieldOfView()); control.SetConstantZNear(50); control.SetConstantZFar(1500);
}
void drawLandmarks(cv::Mat& image,const Matrix& screen) {
    constexpr int connections[][2]={{0,1},{1,2},{2,3},{3,4},{0,5},{5,6},{6,7},{7,8},
        {5,9},{9,10},{10,11},{11,12},{9,13},{13,14},{14,15},{15,16},{13,17},{0,17},{17,18},{18,19},{19,20}};
    std::array<std::optional<cv::Point>,21> points;
    auto valid=[](double x){return x>=0 && (x<=1 || std::abs(x-1)<=1e-9*std::max(std::abs(x),1.));};
    for(int i=0;i<21;++i) {
        double x=screen(i,0),y=screen(i,1);
        if(valid(x) && valid(y)) points[i]=cv::Point(std::min(int(std::floor(x*image.cols)),image.cols-1),std::min(int(std::floor(y*image.rows)),image.rows-1));
    }
    for(auto& pair:connections) if(points[pair[0]] && points[pair[1]])
        cv::line(image,*points[pair[0]],*points[pair[1]],cv::Scalar(224,224,224),2);
    for(auto& p:points) if(p) {
        cv::circle(image,*p,3,cv::Scalar(224,224,224),2);
        cv::circle(image,*p,2,cv::Scalar(0,0,255),2);
    }
}
void text(cv::Mat& image,const QString& label,cv::Point p,double size,cv::Scalar color,int thickness) {
    cv::putText(image,label.toStdString(),p,cv::FONT_HERSHEY_SIMPLEX,size,color,thickness,cv::LINE_AA);
}
}
QJsonObject loadConfig(const QString& root,const QString& mode,const QString& overridePath) {
    auto config=readJson(root+"/configs/"+mode+".json");
    if(!overridePath.isEmpty()) config=merge(config,readJson(overridePath));
    validateConfig(config,mode!="viewer"); return config;
}
void validateConfig(const QJsonObject& c,bool interaction) {
    auto integer=[&](const char* group,const char* key,int minimum,int maximum) {
        double n=number(c,group,key); require(n==std::floor(n) && n>=minimum && n<=maximum,"Configuration integer out of range");
    };
    integer("camera","index",0,1000); integer("camera","width",1,16384); integer("camera","height",1,16384);
    boolean(c,"camera","mirror");
    integer("detector","model_complexity",0,1); integer("detector","max_hands",1,2);
    integer("detector","xnnpack_threads",1,64);
    for(auto key:{"detector_fps","idle_detector_fps","render_fps","idle_render_fps"}) {
        double n=number(c,"performance",key); require(n>0 && n<=1000,"Performance FPS limits must be in (0,1000]");
    }
    require(number(c,"performance","idle_detector_fps")<=number(c,"performance","detector_fps"),
        "performance.idle_detector_fps must not exceed detector_fps");
    require(number(c,"performance","idle_render_fps")<=number(c,"performance","render_fps"),
        "performance.idle_render_fps must not exceed render_fps");
    for(auto key:{"min_detection_confidence","min_tracking_confidence"}) {
        double n=number(c,"detector",key); require(n>=0 && n<=1,"Detector confidence must be in [0,1]");
    }
    integer("mano","iterations",0,10000);
    integer("mano","jacobian_workers",0,45);
    double manoMaxFps=number(c,"mano","max_fps"); require(manoMaxFps>0 && manoMaxFps<=1000,"mano.max_fps must be in (0,1000]");
    require(string(c,"mano","executor")=="thread", "Native Qt mano.executor must be thread");
    require(string(c,"mano","jacobian_backend")=="thread", "Native Qt mano.jacobian_backend must be thread");
    double smoothing=number(c,"mano","pose_smoothing"); require(smoothing>=0 && smoothing<=1,"Pose smoothing must be in [0,1]");
    integer("tracking","handedness_confirm_frames",1,10000);
    positionConfig(c);
    QString map=string(c,"tracking","handedness_map"); require(map=="auto" || map=="direct" || map=="swapped","Invalid handedness mapping");
    auto d=depthConfig(c);
    integer("depth_estimation","calibration_frames",1,100000);
    require(d.minimum>=0 && d.minimum<d.maximum && d.reference>=d.minimum && d.reference<=d.maximum,"Invalid depth range/reference");
    require(d.motionGain>0 && d.minCutoff>0 && d.beta>=0,"Invalid depth filter/gain");
    QString view=string(c,"viewer","initial_view"); require(view=="front" || view=="depth","Invalid initial view");
    string(c,"viewer","window_name");
    if(interaction) {
        double enter=number(c,"pinch","enter_distance"),exit=number(c,"pinch","exit_distance");
        require(enter>0 && enter<exit,"Require 0 < pinch enter < exit");
        require(number(c,"pinch","grab_tolerance")>=0 && number(c,"pinch","missing_timeout")>=0,"Invalid pinch tolerance/timeout");
        vector3(c,"ball","center");
        require(number(c,"ball","radius")>0,"Invalid ball radius");
        vector3(c,"button","center");
        for(auto key:{"width","height","travel","tip_radius"}) require(number(c,"button",key)>0,"Button dimensions must be positive");
        require(number(c,"ball","minimum_scale")>0 && number(c,"ball","maximum_scale")>=number(c,"ball","minimum_scale"),"Invalid ball scale range");
        require(number(c,"ball","scale_sensitivity")>0,"Invalid ball sensitivity");
    }
}
int runApplication(const QString& root,const QJsonObject& c,bool interaction,int maximumFrames,bool synthetic,const QString& capturePath,bool hidden,const QString& videoPath,double benchmarkSeconds) {
    auto dconfig=depthConfig(c);
    auto pconfig=positionConfig(c);
    const int xnnpackThreads=int(number(c,"detector","xnnpack_threads"));
    const int jacobianWorkers=int(number(c,"mano","jacobian_workers"));
    const double manoMaxFps=number(c,"mano","max_fps");
    const double detectorFps=number(c,"performance","detector_fps");
    const double idleDetectorFps=number(c,"performance","idle_detector_fps");
    const double renderFps=number(c,"performance","render_fps");
    const double idleRenderFps=number(c,"performance","idle_render_fps");
    std::array<std::unique_ptr<HandState>,2> states;
    MultiHandDepthEstimator depthTracker(dconfig);
    auto& estimators=depthTracker.estimators;
    HandednessResolver resolver(int(number(c,"tracking","handedness_confirm_frames")));
    std::array<double,2> depths{};
    std::array<Pinch,2> pinches;
    std::array<std::shared_ptr<Mesh>,2> meshes,markers;
    std::array<bool,2> meshAdded{},markerAdded{};
    std::array<std::vector<Vec3>,2> markerBase;
    for(int side=0;side<2;++side) {
        states[side]=std::make_unique<HandState>((root+"/models/MANO_"+(side==0?"LEFT":"RIGHT")+".bin").toStdString(),side,
            int(number(c,"mano","iterations")),number(c,"mano","pose_smoothing"),pconfig,boolean(c,"camera","mirror"));
        states[side]->converter.setJacobianWorkers(jacobianWorkers);
        meshes[side]=std::make_shared<Mesh>();
        auto faces=states[side]->converter.getFaces(true);
        for(int f=0;f<faces.rows();++f) meshes[side]->triangles_.push_back(faces.row(f).cast<int>().transpose());
        // Preserve the source's call order: paint on the initially empty hand mesh.
        meshes[side]->PaintUniformColor(side==0?Vec3(.35,.65,1):Vec3(1,.65,.35));
        if(interaction) {
            pinches[side].enter=number(c,"pinch","enter_distance"); pinches[side].exit=number(c,"pinch","exit_distance");
            pinches[side].missingTimeout=number(c,"pinch","missing_timeout");
            markers[side]=Mesh::CreateSphere(5,12); markers[side]->ComputeVertexNormals();
            markers[side]->PaintUniformColor(side==0?Vec3(.2,.8,1):Vec3(1,.55,.2)); markerBase[side]=markers[side]->vertices_;
        }
    }
    Ball ball(interaction?number(c,"ball","radius"):35,interaction?vector3(c,"ball","center"):Vec3(0,0,35));
    std::unique_ptr<DepthButton> button;
    if(interaction) button=std::make_unique<DepthButton>(vector3(c,"button","center"),number(c,"button","width"),
        number(c,"button","height"),number(c,"button","travel"),number(c,"button","tip_radius"));
    BallController controller(ball,interaction?number(c,"ball","minimum_scale"):.35,
        interaction?number(c,"ball","maximum_scale"):2,interaction?number(c,"ball","scale_sensitivity"):.5);
    std::unique_ptr<Detector> detector;
    if(!synthetic) detector=std::make_unique<Detector>(root,int(number(c,"detector","model_complexity")),int(number(c,"detector","max_hands")),
        xnnpackThreads,
        number(c,"detector","min_detection_confidence"),number(c,"detector","min_tracking_confidence"));
    cv::VideoCapture camera;
    if(!synthetic) {
        if(videoPath.isEmpty()) {
            camera.open(int(number(c,"camera","index")));
            camera.set(cv::CAP_PROP_FRAME_WIDTH,number(c,"camera","width"));
            camera.set(cv::CAP_PROP_FRAME_HEIGHT,number(c,"camera","height")); camera.set(cv::CAP_PROP_BUFFERSIZE,1);
        } else camera.open(videoPath.toStdString());
        if(!camera.isOpened()) throw std::runtime_error("Cannot open configured camera/video");
    }
    open3d::visualization::VisualizerWithKeyCallback visualizer;
    if(!visualizer.CreateVisualizerWindow(string(c,"viewer","window_name").toStdString(),1920,1080,50,50,!synthetic && !hidden))
        throw std::runtime_error("Open3D could not create an OpenGL window");
    struct Cleanup {
        cv::VideoCapture& camera; Visualizer& visualizer;
        ~Cleanup(){camera.release();cv::destroyAllWindows();visualizer.DestroyVisualizerWindow();}
    } cleanup{camera,visualizer};
    auto& options=visualizer.GetRenderOption();
    options.background_color_=Vec3(.03,.03,.03); options.light_on_=true; options.mesh_show_back_face_=true;
    auto bounds=std::make_shared<open3d::geometry::PointCloud>();
    bounds->points_={Vec3(-230,-180,interaction?-80:-60),Vec3(230,180,interaction?80:60)};
    bounds->colors_={options.background_color_,options.background_color_}; visualizer.AddGeometry(bounds,true);
    QString viewMode=string(c,"viewer","initial_view"); configureView(visualizer,viewMode);
    std::shared_ptr<Mesh> ballMesh,ballFrame,buttonBase,buttonCap;
    std::vector<Vec3> ballBase,frameBase;
    std::vector<Vec3> buttonCapVertices;
    if(interaction) {
        ballMesh=Mesh::CreateSphere(ball.baseRadius,28); ballMesh->ComputeVertexNormals(); ballMesh->PaintUniformColor(Vec3(.95,.72,.15));
        ballFrame=Mesh::CreateCoordinateFrame(ball.baseRadius*1.45,Vec3::Zero()); ballBase=ballMesh->vertices_; frameBase=ballFrame->vertices_;
        transformMesh(*ballMesh,ballBase,ball); transformMesh(*ballFrame,frameBase,ball);
        visualizer.AddGeometry(ballMesh,false); visualizer.AddGeometry(ballFrame,false);
        auto box=[](double width,double height,double thickness) {
            auto mesh=Mesh::CreateBox(width,height,thickness);
            mesh->Translate(Vec3(-width/2,-height/2,0)); mesh->ComputeVertexNormals(); return mesh;
        };
        buttonCap=box(button->width,button->height,10);
        buttonCapVertices=buttonCap->vertices_;
        buttonBase=box(button->width+12,button->height+12,8);
        buttonBase->Translate(button->center+Vec3(0,0,button->travel+10)); buttonBase->PaintUniformColor(Vec3(.22,.25,.30));
        buttonCap->Translate(button->capOffset()); buttonCap->PaintUniformColor(button->capColor());
        visualizer.AddGeometry(buttonBase,false); visualizer.AddGeometry(buttonCap,false);
    }
    QElapsedTimer clock; clock.start(); int frames=0,nextSequence=1,detectedHands=0,meshFrames=0;
    std::array<unsigned long long,2> displayedVersions{};
    std::deque<double> captureTimings,mediapipeTimings,mainRenderTimings;
    struct Benchmark {
        bool started=false;
        double startedAt=0,cpuStartedAt=0,duration=0,captureTotal=0,mediapipeTotal=0,renderTotal=0;
        unsigned long long frames=0,detections=0;
        std::array<unsigned long long,2> ikVersions{},ikSamples{};
        std::array<double,2> ikTotal{};
        std::vector<double> mediapipeValues;
        std::array<std::vector<double>,2> ikValues;
    } benchmark;
    benchmark.duration=benchmarkSeconds;
    auto recordTiming=[](std::deque<double>& samples,double seconds) {
        samples.push_back(seconds); if(samples.size()>60) samples.pop_front();
    };
    auto meanMs=[](const std::deque<double>& samples) {
        double total=0; for(double value:samples) total+=value;
        return samples.empty()?0:1000*total/samples.size();
    };
    struct Packet {
        cv::Mat frame; std::vector<Detection> raw; double timestamp=0;
        double captureSeconds=0,mediapipeSeconds=0;
    };
    struct LiveRates {
        double timer=0,detector=0,render=0;
    } rates;
    unsigned long long timerCallbacks=0,detectorFrames=0,renderFrames=0;
    unsigned long long sampledTimer=0,sampledDetector=0,sampledRender=0;
    double ratesStarted=0;
    auto updateRates=[&](double now) {
        double elapsed=now-ratesStarted;
        if(elapsed<.5) return;
        rates.timer=(timerCallbacks-sampledTimer)/elapsed;
        rates.detector=(detectorFrames-sampledDetector)/elapsed;
        rates.render=(renderFrames-sampledRender)/elapsed;
        sampledTimer=timerCallbacks; sampledDetector=detectorFrames; sampledRender=renderFrames;
        ratesStarted=now;
    };
    QEventLoop loop;
    QObject deliveryContext;
    std::exception_ptr failure;
    bool stopping=false,renderDirty=true;
    std::function<void(Packet)> processPacket;
    std::function<void(bool)> scheduleCapture;
    // Declared after notification state and before camera/visualizer cleanup is destroyed.
    Worker captureWorker;
    auto readPacket=[&] {
        Packet packet;
        auto captureStarted=std::chrono::steady_clock::now();
        if(!camera.read(packet.frame)) return packet;
        packet.captureSeconds=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-captureStarted).count();
        packet.timestamp=clock.nsecsElapsed()/1e9;
        auto mediapipeStarted=std::chrono::steady_clock::now();
        if(boolean(c,"camera","mirror")) cv::flip(packet.frame,packet.frame,1);
        cv::Mat rgb; cv::cvtColor(packet.frame,rgb,cv::COLOR_BGR2RGB);
        packet.raw=detector->process(rgb);
        packet.mediapipeSeconds=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-mediapipeStarted).count();
        return packet;
    };
    auto requestCalibration=[&] { if(dconfig.enabled) depthTracker.requestCalibration(clock.nsecsElapsed()/1e9); };
    visualizer.RegisterMouseScrollCallback([&](Visualizer* vis,double,double y) { vis->GetViewControl().Scale(-y); renderDirty=true; return false; });
    for(int key:{257,335}) visualizer.RegisterKeyCallback(key,[&](Visualizer*) { requestCalibration(); return false; });
    QString cameraTitle=interaction?"Pinch Interaction":"MediaPipe Hands";
    processPacket=[&](Packet packet) {
        try {
            cv::Mat frame=std::move(packet.frame);
            if(synthetic) frame=cv::Mat::zeros(int(number(c,"camera","height")),int(number(c,"camera","width")),CV_8UC3);
            else if(frame.empty()) { stopping=true; loop.quit(); return; }
            bool mirror=boolean(c,"camera","mirror");
            double now=synthetic?clock.nsecsElapsed()/1e9:packet.timestamp;
            std::vector<Detection> raw=std::move(packet.raw);
            ++detectorFrames;
            if(!synthetic) {
                if(benchmark.duration>0 && !benchmark.started && !raw.empty()) {
                    benchmark.started=true; benchmark.startedAt=clock.nsecsElapsed()/1e9;
                    benchmark.cpuStartedAt=processCpuSeconds();
                    for(int side=0;side<2;++side) benchmark.ikVersions[side]=states[side]->resultVersion;
                    std::cout<<"BENCHMARK_STARTED seconds="<<benchmark.duration
                        <<" xnnpack_threads="<<xnnpackThreads
                        <<" jacobian_workers="<<jacobianWorkers<<'\n'<<std::flush;
                }
                recordTiming(captureTimings,packet.captureSeconds);
                recordTiming(mediapipeTimings,packet.mediapipeSeconds);
                detectedHands+=int(raw.size());
                if(benchmark.started) {
                    benchmark.captureTotal+=packet.captureSeconds;
                    benchmark.mediapipeTotal+=packet.mediapipeSeconds;
                    benchmark.mediapipeValues.push_back(packet.mediapipeSeconds);
                    benchmark.detections+=raw.size();
                }
            } else {
                // Explicit synthetic render mode, never substituted for a live detector.
                for(int side=0;side<2;++side) {
                    auto& state=*states[side];
                    state.result=SolveResult{state.converter.cameraVertices(),state.converter.cameraKeypoints()};
                    state.displayWrist=Eigen::Vector2d(side==0?.3:.7,.5);
                    state.lastSeen=now; depths[side]=dconfig.reference;
                }
            }
            QString mapping=string(c,"tracking","handedness_map");
            for(auto& d:raw) if(mapping=="swapped" || (mapping=="auto" && !mirror)) d.rawSide=1-d.rawSide;
            for(auto& s:states) s->collect();
            if(benchmark.started) for(int side=0;side<2;++side) {
                auto& state=*states[side];
                if(state.resultVersion>benchmark.ikVersions[side] && !state.ikTimings.empty()) {
                    benchmark.ikTotal[side]+=state.ikTimings.back();
                    benchmark.ikValues[side].push_back(state.ikTimings.back());
                    ++benchmark.ikSamples[side];
                    benchmark.ikVersions[side]=state.resultVersion;
                }
            }
            auto detections=resolver.resolve(raw,states,now);
            std::map<int,std::optional<double>> sceneScales;
            if(dconfig.enabled) {
                std::map<int,double> sizes;
                for(const auto& [side,d]:detections) {
                    auto& s=*states[side];
                    sceneScales[side]=palmScale(d.screen,d.world,s.sceneReferenceKeypoints,frame.cols,frame.rows,&s.sceneSegmentCorrections);
                    sizes[side]=sceneScales[side]?20/ *sceneScales[side]:std::numeric_limits<double>::quiet_NaN();
                }
                for(const auto& [side,z]:depthTracker.update(sizes,now)) depths[side]=z;
            }
            for(int side:resolver.resolvedOrder) {
                auto& d=detections.at(side);
                auto& state=*states[side]; Matrix filtered=state.update(d,now);
                if(dconfig.enabled) state.updateScenePosition(d,depths[side],frame.cols,frame.rows,now,
                    sceneScales[side].value_or(std::numeric_limits<double>::quiet_NaN()));
                if(interaction) {
                    std::optional<Vec3> point;
                    if(state.result && state.displayWrist) {
                        Matrix points=scenePoints(state.result->keypoints,*state.displayWrist,depths[side],state.sceneTranslation);
                        point=((points.row(16)+points.row(20))*.5).transpose();
                    }
                    if(pinches[side].update(filtered,now,ball,number(c,"pinch","grab_tolerance"),nextSequence,point)) ++nextSequence;
                }
                drawLandmarks(frame,d.screen);
                QString depthText=QString("Z=%1mm").arg(depths[side],0,'f',0);
                if(dconfig.enabled && !estimators[side].calibrated()) depthText+=QString(" CAL %1%").arg(estimators[side].progress()*100,0,'f',0);
                QString name=side==0?"LEFT":"RIGHT",label; cv::Scalar color;
                if(interaction) {
                    auto& pinch=pinches[side]; QString status=pinch.controls?"GRAB":pinch.pinching?"PINCH (outside)":"OPEN";
                    label=QString("%1 / MANO_%1 %2 %3mm %4").arg(name,status).arg(pinch.distance*1000,0,'f',1).arg(depthText);
                    color=pinch.controls?cv::Scalar(80,255,80):cv::Scalar(80,200,255);
                } else {
                    label=QString("%1 / MANO_%1 %2 %3").arg(name,d.stabilized?"(locked)":QString("%1%").arg(d.score*100,0,'f',0),depthText);
                    color=side==0?cv::Scalar(255,220,80):cv::Scalar(80,180,255);
                }
                text(frame,label,{int(d.screen(0,0)*frame.cols),int(d.screen(0,1)*frame.rows)-12},interaction?.52:.55,color,2);
            }
            if(raw.size()==1 && detections.size()==1) for(int side=0;side<2;++side) if(side!=detections.begin()->first) {
                double timestamp=states[detections.begin()->first]->lastSeen;
                if(timestamp-states[side]->lastSeen>.35) { states[side]->deactivate(); if(interaction) pinches[side].release(); }
            }
            for(int side=0;side<2;++side) {
                auto& s=*states[side];
                if(!detections.count(side)) s.pending.reset();
                if(interaction) pinches[side].expire(now);
                s.submit(now,manoMaxFps);
            }
            if(interaction) {
                Vec3 oldBallCenter=ball.center; Mat3 oldBallRotation=ball.rotation; double oldBallScale=ball.scale;
                bool oldButtonPressed=button->pressed;
                controller.update(pinches);
                std::map<int,Vec3> tips;
                for(const auto& [side,d]:detections) {
                    auto& s=*states[side];
                    if(s.result && s.displayWrist) tips[side]=scenePoints(s.result->keypoints,*s.displayWrist,depths[side],s.sceneTranslation).row(16).transpose();
                }
                button->update(tips);
                if(oldButtonPressed!=button->pressed) {
                    for(size_t i=0;i<buttonCapVertices.size();++i) buttonCap->vertices_[i]=buttonCapVertices[i]+button->capOffset();
                    buttonCap->PaintUniformColor(button->capColor()); visualizer.UpdateGeometry(buttonCap); renderDirty=true;
                }
                if(oldBallScale!=ball.scale || !oldBallCenter.isApprox(ball.center) || !oldBallRotation.isApprox(ball.rotation)) {
                    transformMesh(*ballMesh,ballBase,ball); transformMesh(*ballFrame,frameBase,ball);
                    visualizer.UpdateGeometry(ballMesh); visualizer.UpdateGeometry(ballFrame); renderDirty=true;
                }
                for(int side=0;side<2;++side) {
                    auto& pinch=pinches[side];
                    if(pinch.pinching && pinch.point) {
                        Ball translation(1,Vec3::Zero()); translation.center=*pinch.point; transformMesh(*markers[side],markerBase[side],translation);
                        if(!markerAdded[side]) { visualizer.AddGeometry(markers[side],false); markerAdded[side]=true; }
                        visualizer.UpdateGeometry(markers[side]); renderDirty=true;
                    } else if(markerAdded[side]) { visualizer.RemoveGeometry(markers[side],false); markerAdded[side]=false; renderDirty=true; }
                }
            }
            for(int side=0;side<2;++side) {
                auto& s=*states[side];
                if(now-s.lastSeen<resolver.trackTimeout && s.result && s.displayWrist) {
                    ++meshFrames;
                    if(synthetic || detections.count(side) || s.resultVersion!=displayedVersions[side] || !meshAdded[side]) {
                        displayedVersions[side]=s.resultVersion;
                        setVertices(*meshes[side],scenePoints(s.result->vertices,*s.displayWrist,depths[side],s.sceneTranslation));
                        if(!meshAdded[side]) { visualizer.AddGeometry(meshes[side],false); meshAdded[side]=true; }
                        visualizer.UpdateGeometry(meshes[side]); renderDirty=true;
                    }
                } else if(meshAdded[side]) { visualizer.RemoveGeometry(meshes[side],false); meshAdded[side]=false; renderDirty=true; }
            }
            double wallNow=clock.nsecsElapsed()/1e9;
            QString loopRates=QString("FPS Timer:%1 Detect:%2 Render:%3")
                .arg(rates.timer,0,'f',1).arg(rates.detector,0,'f',1).arg(rates.render,0,'f',1);
            auto ikRate=[&](int side) {
                auto& state=*states[side];
                double seconds=state.meanIkSeconds();
                if(now-state.lastSeen>=resolver.trackTimeout || !state.result || seconds<=0)
                    return QString("--");
                return QString("%1/%2 (%3ms)").arg(1/seconds,0,'f',1)
                    .arg(manoMaxFps,0,'g',6).arg(seconds*1000,0,'f',2);
            };
            QString ikRates=QString("IK solve/max FPS L:%1 R:%2").arg(ikRate(0),ikRate(1));
            QString timings=QString("Time ms Capture:%1 MediaPipe:%2 Render:%3")
                .arg(meanMs(captureTimings),0,'f',1).arg(meanMs(mediapipeTimings),0,'f',1)
                .arg(meanMs(mainRenderTimings),0,'f',2);
            int ikBaseline=0;
            double ikScale=std::min(.58,.58*(frame.cols-24)/
                cv::getTextSize(ikRates.toStdString(),cv::FONT_HERSHEY_SIMPLEX,.58,2,&ikBaseline).width);
            text(frame,ikRates,{12,28},ikScale,{80,255,255},1);
            int baselinePixels=0;
            double rateScale=std::min(.58,.58*(frame.cols-24)/
                cv::getTextSize(loopRates.toStdString(),cv::FONT_HERSHEY_SIMPLEX,.58,2,&baselinePixels).width);
            if(interaction) {
                text(frame,loopRates,{12,52},rateScale,{80,255,80},1);
                text(frame,timings,{12,76},.48,{255,210,80},1);
                text(frame,QString("Ball scale %1x  [1 front / 3 depth]").arg(ball.scale,0,'f',2),{12,102},.52,{220,220,220},1);
            } else {
                text(frame,loopRates,{12,52},rateScale,{80,255,80},1);
                text(frame,timings,{12,76},.48,{255,210,80},1);
                text(frame,QString("Open3D view: %1  [1 front / 3 depth]").arg(viewMode.toUpper()),{12,102},.52,{210,210,210},1);
            }
            if(dconfig.enabled) text(frame,QString::fromStdString(depthTracker.calibrationStatus(now)),
                {12,frame.rows-18},.52,{80,255,255},1);
            if(interaction) {
                QString status="READY";
                if(button->pressed) {
                    QStringList names; for(int side:button->contacts) names<<(side==0?"left":"right");
                    status="PRESSED "+names.join('/');
                }
                text(frame,QString("Button: %1  presses: %2").arg(status).arg(button->pressCount),
                    {12,126},.52,button->pressed?cv::Scalar(80,255,80):cv::Scalar(220,220,220),2);
            }
            int key=-1;
            if(!synthetic && !hidden) { cv::imshow(cameraTitle.toStdString(),frame); key=cv::waitKey(1)&0xff; }
            if(key==10 || key==13) requestCalibration();
            if(key=='1' || key=='3') { viewMode=key=='1'?"front":"depth"; configureView(visualizer,viewMode); renderDirty=true; }
            if(key=='q' || key==27) { stopping=true; loop.quit(); }
            ++frames;
            if(benchmark.started) ++benchmark.frames;
            if(maximumFrames>0 && frames>=maximumFrames) {
                if(renderDirty) { visualizer.UpdateRender(); renderDirty=false; }
                if(!capturePath.isEmpty()) {
                    visualizer.CaptureScreenImage(capturePath.toStdString(),true);
                    cv::imwrite((capturePath+".camera.png").toStdString(),frame);
                }
                stopping=true; loop.quit();
            }
            if(benchmark.started && wallNow-benchmark.startedAt>=benchmark.duration) { stopping=true; loop.quit(); }
            bool active=!raw.empty();
            for(const auto& state:states) if(now-state->lastSeen<resolver.trackTimeout) active=true;
            if(!stopping) scheduleCapture(active);
        } catch(...) { failure=std::current_exception(); stopping=true; loop.quit(); }
    };
    double nextDetectorAt=0;
    std::optional<bool> lastDetectorActive;
    scheduleCapture=[&](bool active) {
        if(stopping) return;
        double now=clock.nsecsElapsed()/1e9;
        if(lastDetectorActive && *lastDetectorActive!=active) nextDetectorAt=now;
        lastDetectorActive=active;
        int delayMs=int(std::ceil(std::max(0.,nextDetectorAt-now)*1000));
        double fpsLimit=active?detectorFps:idleDetectorFps;
        QTimer::singleShot(delayMs,&deliveryContext,[&,fpsLimit] {
            if(stopping) return;
            nextDetectorAt=clock.nsecsElapsed()/1e9+1/fpsLimit;
            if(synthetic) { Packet packet; packet.timestamp=clock.nsecsElapsed()/1e9; processPacket(std::move(packet)); return; }
            captureWorker.submit([&] {
                try {
                    Packet packet=readPacket();
                    QMetaObject::invokeMethod(&deliveryContext,[&,packet=std::move(packet)]() mutable {
                        if(!stopping) processPacket(std::move(packet));
                    },Qt::QueuedConnection);
                } catch(...) {
                    auto error=std::current_exception();
                    QMetaObject::invokeMethod(&deliveryContext,[&,error] {
                        failure=error; stopping=true; loop.quit();
                    },Qt::QueuedConnection);
                }
            });
        });
    };
    QTimer renderTimer;
    QObject::connect(&renderTimer,&QTimer::timeout,[&] {
        ++timerCallbacks;
        double now=clock.nsecsElapsed()/1e9;
        updateRates(now);
        bool active=false;
        for(const auto& state:states) if(now-state->lastSeen<resolver.trackTimeout) active=true;
        int interval=std::max(1,int(std::round(1000/(active?renderFps:idleRenderFps))));
        if(renderTimer.interval()!=interval) renderTimer.setInterval(interval);
        bool rendering=renderDirty;
        auto renderStarted=std::chrono::steady_clock::now();
        if(rendering) visualizer.UpdateRender();
        if(!visualizer.PollEvents()) { stopping=true; loop.quit(); return; }
        if(rendering) {
            renderDirty=false; ++renderFrames;
            double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-renderStarted).count();
            recordTiming(mainRenderTimings,seconds);
            if(benchmark.started) benchmark.renderTotal+=seconds;
        }
    });
    scheduleCapture(false);
    renderTimer.start(std::max(1,int(std::round(1000/idleRenderFps))));
    loop.exec(); stopping=true; renderTimer.stop();
    if(failure) std::rethrow_exception(failure);
    if(benchmark.duration>0) {
        if(!benchmark.started) throw std::runtime_error("Benchmark ended before a hand was detected");
        double elapsed=clock.nsecsElapsed()/1e9-benchmark.startedAt;
        auto mean=[](double total,unsigned long long count) { return count?1000*total/count:0; };
        auto p95=[](std::vector<double> values) {
            if(values.empty()) return 0.;
            std::sort(values.begin(),values.end());
            size_t index=std::max<size_t>(1,size_t(std::ceil(.95*values.size())))-1;
            return 1000*values[index];
        };
        double cpuPercent=elapsed>0?100*(processCpuSeconds()-benchmark.cpuStartedAt)/elapsed/logicalProcessorCount():0;
        std::cout<<"BENCHMARK_RESULT seconds="<<elapsed
            <<" frames="<<benchmark.frames<<" fps="<<(elapsed>0?benchmark.frames/elapsed:0)
            <<" detections="<<benchmark.detections
            <<" xnnpack_threads="<<xnnpackThreads<<" jacobian_workers="<<jacobianWorkers
            <<" mediapipe_ms="<<mean(benchmark.mediapipeTotal,benchmark.frames)
            <<" mediapipe_p95_ms="<<p95(benchmark.mediapipeValues)
            <<" render_ms="<<mean(benchmark.renderTotal,benchmark.frames)
            <<" cpu_percent="<<cpuPercent
            <<" ik_samples_L="<<benchmark.ikSamples[0]<<" ik_mean_ms_L="<<mean(benchmark.ikTotal[0],benchmark.ikSamples[0])
            <<" ik_p95_ms_L="<<p95(benchmark.ikValues[0])
            <<" ik_samples_R="<<benchmark.ikSamples[1]<<" ik_mean_ms_R="<<mean(benchmark.ikTotal[1],benchmark.ikSamples[1])
            <<" ik_p95_ms_R="<<p95(benchmark.ikValues[1])<<'\n';
    }
    std::cout<<"Completed "<<frames<<" frames"<<(synthetic?" (synthetic render test)":"")
        <<"; hand detections="<<detectedHands<<"; rendered hand snapshots="<<meshFrames
        <<"; timer callbacks="<<timerCallbacks<<" detector frames="<<detectorFrames<<" Open3D renders="<<renderFrames
        <<"; IK submits L="<<states[0]->ikSubmits<<" R="<<states[1]->ikSubmits
        <<" completes L="<<states[0]->ikCompletes<<" R="<<states[1]->ikCompletes
        <<"; accepted MANO results L="<<states[0]->resultVersion<<" R="<<states[1]->resultVersion
        <<"; XNNPACK threads/inference node="<<xnnpackThreads
        <<"; Jacobian workers/hand="<<jacobianWorkers
        <<"; peak Jacobian concurrency L="<<states[0]->converter.peakJacobianConcurrency()
        <<" R="<<states[1]->converter.peakJacobianConcurrency()
        <<"; mean ms capture="<<meanMs(captureTimings)
        <<" mediapipe="<<meanMs(mediapipeTimings)
        <<" IK-L="<<states[0]->meanIkSeconds()*1000
        <<" IK-R="<<states[1]->meanIkSeconds()*1000
        <<" main/render="<<meanMs(mainRenderTimings)<<'\n';
    return 0;
}
}

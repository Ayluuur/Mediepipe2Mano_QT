#include "detector.h"
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <stdexcept>
namespace m2m {
Detector::Detector(const QString& root,int complexity,int maxHands,int xnnpackThreads,
                   double detection,double tracking) {
    QString path=QCoreApplication::applicationDirPath()+"/mediapipe_hands.dll";
    if(!QFileInfo::exists(path)) path=root+"/Bin/mediapipe_hands.dll";
    library.setFileName(path);
    if(!library.load()) throw std::runtime_error(("Native MediaPipe DLL is unavailable. Run tools/build_mediapipe.ps1.\n"+library.errorString()).toStdString());
    auto create=reinterpret_cast<decltype(&m2m_create)>(library.resolve("m2m_create"));
    processFn=reinterpret_cast<decltype(&m2m_process)>(library.resolve("m2m_process"));
    destroyFn=reinterpret_cast<decltype(&m2m_destroy)>(library.resolve("m2m_destroy"));
    if(!create || !processFn || !destroyFn) throw std::runtime_error("Invalid native MediaPipe DLL ABI");
    char error[4096]{};
    QByteArray graph=QDir::toNativeSeparators(root+"/assets/hands_0_10_9.pbtxt").toUtf8();
    handle=create(graph.constData(),complexity,maxHands,xnnpackThreads,
                  float(detection),float(tracking),error,sizeof(error));
    if(!handle) throw std::runtime_error(error);
}
Detector::~Detector() { if(handle && destroyFn) destroyFn(handle); }
std::vector<Detection> Detector::process(const cv::Mat& rgb) {
    M2MHand hands[2]{}; char error[4096]{};
    // SolutionBase.process uses a fixed 33333-us step, independent of webcam FPS.
    timestamp+=33333;
    int count=processFn(handle,rgb.data,rgb.cols,rgb.rows,int(rgb.step),timestamp,hands,2,error,sizeof(error));
    if(count<0) throw std::runtime_error(error);
    std::vector<Detection> out;
    for(int i=0;i<count;++i) {
        Detection d; d.rawSide=hands[i].side; d.score=hands[i].score;
        d.screen.resize(21,3); d.world.resize(21,3);
        for(int k=0;k<63;++k) { d.screen.data()[k]=hands[i].screen[k]; d.world.data()[k]=hands[i].world[k]; }
        out.push_back(std::move(d));
    }
    return out;
}
}

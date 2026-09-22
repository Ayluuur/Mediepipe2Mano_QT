#pragma once
#include "core.h"
#include "bridge.h"
#include <QLibrary>
#include <opencv2/core.hpp>
namespace m2m {
class Detector {
public:
    Detector(const QString& root,int complexity,int maxHands,int xnnpackThreads,
             double detection,double tracking);
    ~Detector();
    std::vector<Detection> process(const cv::Mat& rgb);
private:
    QLibrary library;
    void* handle=nullptr;
    decltype(&m2m_process) processFn=nullptr;
    decltype(&m2m_destroy) destroyFn=nullptr;
    int64_t timestamp=0;
};
}

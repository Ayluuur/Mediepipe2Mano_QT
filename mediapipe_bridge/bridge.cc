#define M2M_BRIDGE_EXPORT
#include "bridge.h"
#include <algorithm>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
#include "google/protobuf/text_format.h"
#include "mediapipe/framework/calculator_graph.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/classification.pb.h"
#include "mediapipe/calculators/tensor/inference_calculator.pb.h"
#include "mediapipe/calculators/tensor/tensors_to_detections_calculator.pb.h"
#include "mediapipe/calculators/util/thresholding_calculator.pb.h"
namespace {
void errorText(char* dst,int size,const std::string& text) {
    if(dst && size>0) { auto n=std::min<size_t>(size-1,text.size()); std::memcpy(dst,text.data(),n); dst[n]=0; }
}
void check(const absl::Status& status) { if(!status.ok()) throw std::runtime_error(status.ToString()); }
struct Detector {
    mediapipe::CalculatorGraph graph;
    std::mutex mutex;
    std::vector<mediapipe::NormalizedLandmarkList> screen;
    std::vector<mediapipe::LandmarkList> world;
    std::vector<mediapipe::ClassificationList> labels;
    bool started=false;
    ~Detector() { if(started) { graph.CloseAllPacketSources().IgnoreError(); graph.WaitUntilDone().IgnoreError(); } }
};
}
extern "C" M2M_API void* m2m_create(const char* graphPath,int complexity,int maxHands,int xnnpackThreads,
    float detectionConfidence,float trackingConfidence,char* error,int errorSize) {
    try {
        std::ifstream stream(graphPath);
        if(!stream) throw std::runtime_error("Cannot open MediaPipe graph");
        std::string text((std::istreambuf_iterator<char>(stream)),{});
        mediapipe::CalculatorGraphConfig config;
        if(!google::protobuf::TextFormat::ParseFromString(text,&config)) throw std::runtime_error("Cannot parse MediaPipe graph");
        if(xnnpackThreads<1 || xnnpackThreads>64) throw std::runtime_error("xnnpack_threads must be in [1,64]");
        int detectionOptions=0,trackingOptions=0,inferenceOptions=0;
        for(auto& node:*config.mutable_node()) {
            if(node.calculator()=="InferenceCalculatorCpu") {
                auto* options=node.mutable_options()->MutableExtension(mediapipe::InferenceCalculatorOptions::ext);
                options->mutable_delegate()->mutable_xnnpack()->set_num_threads(xnnpackThreads);
                ++inferenceOptions;
            }
            if(node.name()=="palmdetectioncpu__TensorsToDetectionsCalculator") {
                node.mutable_options()->MutableExtension(mediapipe::TensorsToDetectionsCalculatorOptions::ext)->set_min_score_thresh(detectionConfidence);
                ++detectionOptions;
            }
            if(node.name()=="handlandmarkcpu__ThresholdingCalculator") {
                node.mutable_options()->MutableExtension(mediapipe::ThresholdingCalculatorOptions::ext)->set_threshold(trackingConfidence);
                ++trackingOptions;
            }
        }
        if(detectionOptions!=1 || trackingOptions!=1 || inferenceOptions!=2)
            throw std::runtime_error("Expected expanded MediaPipe 0.10.9 Hands graph");
        auto d=std::make_unique<Detector>(); check(d->graph.Initialize(config));
        auto* ptr=d.get();
        check(d->graph.ObserveOutputStream("multi_hand_landmarks",[ptr](const mediapipe::Packet& p) {
            std::lock_guard<std::mutex> lock(ptr->mutex);
            if(!p.IsEmpty()) ptr->screen=p.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
            return absl::OkStatus();
        }));
        check(d->graph.ObserveOutputStream("multi_hand_world_landmarks",[ptr](const mediapipe::Packet& p) {
            std::lock_guard<std::mutex> lock(ptr->mutex);
            if(!p.IsEmpty()) ptr->world=p.Get<std::vector<mediapipe::LandmarkList>>();
            return absl::OkStatus();
        }));
        check(d->graph.ObserveOutputStream("multi_handedness",[ptr](const mediapipe::Packet& p) {
            std::lock_guard<std::mutex> lock(ptr->mutex);
            if(!p.IsEmpty()) ptr->labels=p.Get<std::vector<mediapipe::ClassificationList>>();
            return absl::OkStatus();
        }));
        check(d->graph.StartRun({{"model_complexity",mediapipe::MakePacket<int>(complexity)},
            {"num_hands",mediapipe::MakePacket<int>(maxHands)},{"use_prev_landmarks",mediapipe::MakePacket<bool>(true)}}));
        d->started=true; return d.release();
    } catch(const std::exception& e) { errorText(error,errorSize,e.what()); return nullptr; }
    catch(...) { errorText(error,errorSize,"Unknown MediaPipe initialization failure"); return nullptr; }
}
extern "C" M2M_API int m2m_process(void* handle,const uint8_t* rgb,int width,int height,int stride,
    int64_t timestamp,M2MHand* hands,int capacity,char* error,int errorSize) {
    try {
        if(!handle || !rgb || !hands || width<=0 || height<=0 || stride<width*3 || capacity<1)
            throw std::runtime_error("Invalid detector input");
        auto& d=*static_cast<Detector*>(handle);
        { std::lock_guard<std::mutex> lock(d.mutex); d.screen.clear(); d.world.clear(); d.labels.clear(); }
        auto frame=std::make_unique<mediapipe::ImageFrame>(mediapipe::ImageFormat::SRGB,width,height,mediapipe::ImageFrame::kDefaultAlignmentBoundary);
        for(int y=0;y<height;++y) std::memcpy(frame->MutablePixelData()+y*frame->WidthStep(),rgb+y*stride,width*3);
        check(d.graph.AddPacketToInputStream("image",mediapipe::Adopt(frame.release()).At(mediapipe::Timestamp(timestamp))));
        // Waiting for idle also handles no-hand frames: no blocking poller waits on absent packets.
        check(d.graph.WaitUntilIdle());
        std::lock_guard<std::mutex> lock(d.mutex);
        int count=int(std::min({d.screen.size(),d.world.size(),d.labels.size(),size_t(capacity)}));
        for(int h=0;h<count;++h) {
            if(d.screen[h].landmark_size()!=21 || d.world[h].landmark_size()!=21 || d.labels[h].classification_size()<1)
                throw std::runtime_error("Invalid hand landmark packet");
            const auto& c=d.labels[h].classification(0);
            hands[h].side=c.label()=="Left"?0:1; hands[h].score=c.score();
            for(int k=0;k<21;++k) {
                const auto& s=d.screen[h].landmark(k); const auto& w=d.world[h].landmark(k);
                hands[h].screen[3*k]=s.x(); hands[h].screen[3*k+1]=s.y(); hands[h].screen[3*k+2]=s.z();
                hands[h].world[3*k]=w.x(); hands[h].world[3*k+1]=w.y(); hands[h].world[3*k+2]=w.z();
            }
        }
        return count;
    } catch(const std::exception& e) { errorText(error,errorSize,e.what()); return -1; }
    catch(...) { errorText(error,errorSize,"Unknown MediaPipe processing failure"); return -1; }
}
extern "C" M2M_API void m2m_destroy(void* handle) { delete static_cast<Detector*>(handle); }

#pragma once
#include <Eigen/Dense>
#include "worker.h"
#include <atomic>
#include <array>
#include <deque>
#include <future>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace m2m {
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
constexpr double inf = std::numeric_limits<double>::infinity();
Vec3 normalize(const Vec3& v);
Mat3 rotationBetween(const Vec3& a, const Vec3& b);
Mat3 palmFrame(const Matrix& world);
Matrix scenePoints(const Matrix& points, const Eigen::Vector2d& wrist, double depth,
                   const std::optional<Vec3>& translation=std::nullopt);
std::optional<double> palmScale(const Matrix& screen, const Matrix& world,
    const Matrix& reference, int width, int height, std::map<int,double>* corrections=nullptr);
std::optional<Vec3> screenTranslation(const Matrix& screen, const Matrix& keypoints,
    int width, int height, double depth, std::optional<double> mmPerPixel=std::nullopt);

class OneEuro {
public:
    double minCutoff, beta, derivativeCutoff;
    OneEuro(double cutoff=1.2, double b=4, double derivative=1,
            int medianWindow=1, std::optional<double> maxSpeed=std::nullopt);
    void reset() { value.resize(0,0); samples.clear(); }
    Matrix operator()(const Matrix& input, double time);
private:
    Matrix value, raw, derivative;
    double timestamp=0;
    int medianWindow;
    std::optional<double> maxSpeed;
    std::deque<Matrix> samples;
};

struct DepthConfig {
    bool enabled=true;
    double reference=50, minimum=10, maximum=300;
    int calibrationFrames=30;
    double minCutoff=1, beta=.02, maxSpeed=500, motionGain=1.5;
};
class DepthEstimator {
public:
    explicit DepthEstimator(DepthConfig c={}): config(c), depth(c.reference), filter(c.minCutoff,c.beta,1,3) {}
    double update(const Matrix& screen, double time, int width, int height);
    double updateSize(double size, double time, bool calibrate=true);
    bool calibrated() const { return referenceSize.has_value(); }
    double progress() const;
    DepthConfig config;
    double depth;
private:
    friend class MultiHandDepthEstimator;
    OneEuro filter;
    std::vector<double> samples;
    std::optional<double> referenceSize, lastSeen;
};
class MultiHandDepthEstimator {
public:
    explicit MultiHandDepthEstimator(DepthConfig c={}):
        estimators{DepthEstimator(c),DepthEstimator(c)}, calibrationFrames(c.calibrationFrames) {}
    std::array<DepthEstimator,2> estimators;
    void requestCalibration(double time) { deadline=time+3; completedAt.reset(); }
    std::string calibrationStatus(double time) const;
    std::map<int,double> update(const std::map<int,double>& sizes, double time);
private:
    int calibrationFrames;
    std::vector<double> samples;
    std::optional<double> referenceSize, deadline, completedAt;
};

class ManoModel {
public:
    explicit ManoModel(const std::string& path);
    void setPose(const Eigen::VectorXd& pca, bool keypointsOnly=false);
    void setAbsolutePose(const Matrix& pose, bool keypointsOnly=false);
    Matrix keypointsBatch(const Matrix& poses) const; // rows: poses, columns: 21 xyz points
    Matrix vertices, keypoints, faces;
    Matrix basis, mean, joints;
    std::array<int,16> parents{};
    std::array<Eigen::Matrix4d,16> jointTransforms;
private:
    Matrix regressor, weights, meshTemplate, poseBasis;
    void update(const Matrix& pose, bool keypointsOnly=false);
};
struct SkeletonPose {
    std::string side, space;
    std::array<std::string,16> names={"W","I0","I1","I2","M0","M1","M2","L0","L1","L2","R0","R1","R2","T0","T1","T2"};
    std::array<int,16> parents{};
    std::array<Eigen::Matrix4d,16> transforms;
    Matrix restPositions;
    double fitErrorMm=0;
    bool mirroredLocalAxes=false;
    Matrix positions() const;
    Matrix displacements() const;
    std::array<Mat3,16> rotations() const;
    Matrix quaternions() const; // xyzw
    std::array<Eigen::Matrix4d,16> localTransforms() const;
    Matrix localPositions() const;
    std::array<Mat3,16> localRotations() const;
    Matrix localQuaternions() const;
    Matrix localAxisAngles() const;
    SkeletonPose toScene(const Vec3& translation) const;
};
struct SolveResult { Matrix vertices, keypoints; SkeletonPose skeleton; double ikSeconds=0; };
class KeypointsToMano {
public:
    KeypointsToMano(const std::string& path, bool left, int iterations=5, double smoothing=.7,
                    int nPose=45, bool mirroredInput=true);
    Eigen::VectorXd solve(const Matrix& world);
    SolveResult cameraOriented() const;
    Matrix cameraVertices() const;
    Matrix cameraKeypoints() const;
    SkeletonPose getSkeleton(const std::string& space="camera") const;
    Matrix getFaces(bool cameraOriented=false) const;
    void reset();
    void setJacobianWorkers(int count);
    int peakJacobianConcurrency() const { return jacobianPeak.load(); }
    ManoModel model;
private:
    std::vector<std::unique_ptr<ManoModel>> derivativeModels;
    std::vector<std::unique_ptr<Worker>> derivativeWorkers;
    int jacobianWorkers=1;
    std::atomic<int> jacobianActive{0},jacobianPeak{0};
    bool left, mirroredInput, hasPose=false;
    int iterations;
    double smoothing, fitErrorMm=0;
    Eigen::VectorXd pose, neutralPose;
    Matrix poseEncoder;
    Mat3 inputBasis=Mat3::Identity();
    Vec3 restWrist;
    Matrix canonicalRest;
    std::array<double,21> lengths{};
    Mat3 localToMano=Mat3::Identity(), localToCamera=Mat3::Identity();
    Matrix target(const Matrix& world);
    Eigen::VectorXd directionSeed(const Matrix& target) const;
};

struct Detection {
    int rawSide=0; // 0 left, 1 right
    double score=0;
    Matrix screen, world;
    bool stabilized=false;
    Eigen::Vector2d wrist() const { return screen.row(0).head<2>().transpose(); }
};
struct PositionFilterConfig {
    double minCutoff=3, beta=.02, derivativeCutoff=2, maxSpeed=1500;
    int medianWindow=3;
};
struct HandState {
    HandState(const std::string& model, int side, int iterations, double smoothing,
              PositionFilterConfig position={}, bool mirroredInput=true);
    KeypointsToMano converter;
    int side, candidate=-1, candidateFrames=0;
    double lastSeen=-inf;
    OneEuro filter{1.2,4,1,3}, positionFilter, scenePositionFilter;
    Matrix sceneReferenceKeypoints;
    std::map<int,double> sceneSegmentCorrections;
    std::optional<Vec3> sceneTranslation;
    std::optional<double> sceneMmPerPixel;
    std::optional<Eigen::Vector2d> screenWrist, displayWrist;
    std::optional<Matrix> pending;
    std::optional<SolveResult> result;
    std::future<SolveResult> future;
    bool discard=false;
    unsigned long long resultVersion=0;
    std::deque<double> ikTimings;
    double meanIkSeconds() const {
        double total=0;
        for(double seconds:ikTimings) total+=seconds;
        return ikTimings.empty()?0:total/ikTimings.size();
    }
    ~HandState() { if(future.valid()) future.wait(); }
    Worker solverWorker;
    void beginTrack();
    void deactivate();
    Matrix update(const Detection& d, double time);
    void collect();
    void submit();
    std::optional<SkeletonPose> getSkeleton(const std::string& space="camera") const;
    void updateScenePosition(const Detection& detection, double depth, int width,
                             int height, double time, std::optional<double> mmPerPixel=std::nullopt);
};
class HandednessResolver {
public:
    explicit HandednessResolver(int confirmFrames=5):confirmFrames(confirmFrames) {}
    const double trackTimeout=.35;
    std::vector<int> resolvedOrder; // Python dict order determines simultaneous pinch ownership.
    std::map<int,Detection> resolve(std::vector<Detection> raw,
        const std::array<std::unique_ptr<HandState>,2>& states, double time);
private:
    int confirmFrames;
    struct Candidate { Eigen::Vector2d wrist; int count; double time; };
    std::map<int,Candidate> pending;
};

struct Ball {
    explicit Ball(double r=35, Vec3 center=Vec3(0,0,35)): baseRadius(r), center(center) {}
    double baseRadius, scale=1;
    Vec3 center;
    Mat3 rotation=Mat3::Identity();
    double radius() const { return baseRadius*scale; }
    bool contains(const Vec3& p, double tolerance) const { return (p-center).norm() <= radius()+tolerance; }
};
class DepthButton {
public:
    DepthButton(Vec3 center, double width, double height, double travel, double tipRadius):
        center(center), width(width), height(height), travel(travel), tipRadius(tipRadius) {}
    Vec3 center;
    double width, height, travel, tipRadius;
    bool pressed=false;
    int pressCount=0;
    std::set<int> contacts;
    void update(const std::map<int,Vec3>& points);
    Vec3 capOffset() const { return center+Vec3(0,0,pressed?travel:0); }
    Vec3 capColor() const { return pressed?Vec3(.15,1,.35):Vec3(.25,.48,.85); }
private:
    std::map<int,Vec3> previous;
};
struct Pinch {
    double enter=.045, exit=.055, missingTimeout=.18;
    bool pinching=false, controls=false;
    int sequence=-1;
    std::optional<Vec3> point;
    Mat3 handFrame=Mat3::Identity();
    double lastSeen=-inf, distance=inf;
    bool update(const Matrix& world, double time, const Ball& ball,
        double tolerance, int nextSequence, const std::optional<Vec3>& scenePoint);
    void release() { pinching=false; controls=false; sequence=-1; }
    void expire(double time) { if(pinching && time-lastSeen>missingTimeout) release(); }
};
class BallController {
public:
    BallController(Ball& b, double minimum=.35, double maximum=2, double sensitivity=.5):
        ball(b), minimum(minimum), maximum(maximum), sensitivity(sensitivity) {}
    void update(std::array<Pinch,2>& states);
private:
    Ball& ball;
    double minimum, maximum, sensitivity;
    std::vector<std::pair<int,int>> signature;
    Vec3 center, firstPoint, twoVector;
    Mat3 rotation, firstFrame;
    double scale=1, twoDistance=1;
};
} // namespace m2m

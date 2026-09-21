#pragma once
#include <stdint.h>
#ifdef M2M_BRIDGE_EXPORT
#define M2M_API __declspec(dllexport)
#else
#define M2M_API
#endif
#ifdef __cplusplus
extern "C" {
#endif
// C ABI isolates Bazel/MediaPipe dependencies from Qt/CMake and Open3D.
typedef struct M2MHand {
    int32_t side;
    float score;
    float screen[63];
    float world[63];
} M2MHand;
M2M_API void* m2m_create(const char* graph_path,int complexity,int max_hands,int xnnpack_threads,
                       float detection_confidence,float tracking_confidence,char* error,int error_size);
// RGB is copied during this call. Outputs belong to the caller. No C++ types cross the DLL boundary.
M2M_API int m2m_process(void* handle,const uint8_t* rgb,int width,int height,int stride,
                      int64_t timestamp_us,M2MHand* hands,int capacity,char* error,int error_size);
M2M_API void m2m_destroy(void* handle);
#ifdef __cplusplus
}
#endif

cc_binary(
    name = "mediapipe_hands.dll",
    srcs = ["bridge.cc", "bridge.h"],
    linkshared = True,
    deps = [
        "//mediapipe/framework:calculator_graph",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework/formats:classification_cc_proto",
        "//mediapipe/modules/hand_landmark:hand_landmark_tracking_cpu",
        "//mediapipe/calculators/tensor:inference_calculator_cc_proto",
        "//mediapipe/calculators/tensor:tensors_to_detections_calculator_cc_proto",
        "//mediapipe/calculators/util:thresholding_calculator_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

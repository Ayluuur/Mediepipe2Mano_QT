cmake_minimum_required(VERSION 3.24)
project(MediaPipe2ManoQt VERSION 0.4.0 LANGUAGES CXX)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
find_package(Qt6 6.8 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Open3D 0.19.0 EXACT REQUIRED CONFIG)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui videoio imgcodecs)
add_library(mano_core
  core.cpp core.h skeleton.cpp skeleton_json.h worker.h)
source_group("Source Files" FILES core.cpp skeleton.cpp)
source_group("Header Files" FILES core.h skeleton_json.h worker.h)
target_include_directories(mano_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mano_core PUBLIC Open3D::Open3D)
set_target_properties(mano_core PROPERTIES FOLDER "Libraries")
if(MSVC)
  target_compile_options(mano_core PUBLIC /utf-8 /bigobj /EHsc)
  target_compile_definitions(mano_core PUBLIC NOMINMAX)
endif()
option(BUILD_TESTING "Build test projects" ON)
if(BUILD_TESTING)
  enable_testing()
  add_executable(parity_tests tests/parity.cpp)
  target_link_libraries(parity_tests PRIVATE mano_core Qt6::Core)
  set_target_properties(parity_tests PROPERTIES FOLDER "Tests")
  add_test(NAME python_cpp_parity COMMAND parity_tests "${CMAKE_CURRENT_SOURCE_DIR}")
  set_tests_properties(python_cpp_parity PROPERTIES ENVIRONMENT_MODIFICATION
    "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:$<TARGET_FILE_DIR:Open3D::Open3D>")
  add_executable(sync_parity_tests tests/sync_parity.cpp)
  target_link_libraries(sync_parity_tests PRIVATE mano_core Qt6::Core)
  set_target_properties(sync_parity_tests PROPERTIES FOLDER "Tests")
  add_test(NAME current_python_sync COMMAND sync_parity_tests "${CMAKE_CURRENT_SOURCE_DIR}")
  set_tests_properties(current_python_sync PROPERTIES ENVIRONMENT_MODIFICATION
    "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:$<TARGET_FILE_DIR:Open3D::Open3D>")
  add_executable(skeleton_parity_tests tests/skeleton_parity.cpp)
  target_link_libraries(skeleton_parity_tests PRIVATE mano_core Qt6::Core)
  set_target_properties(skeleton_parity_tests PROPERTIES FOLDER "Tests")
  add_test(NAME skeleton_python_parity COMMAND skeleton_parity_tests "${CMAKE_CURRENT_SOURCE_DIR}")
  set_tests_properties(skeleton_python_parity PROPERTIES ENVIRONMENT_MODIFICATION
    "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:$<TARGET_FILE_DIR:Open3D::Open3D>")
  add_executable(config_profiles_tests tests/config_profiles.cpp)
  target_link_libraries(config_profiles_tests PRIVATE Qt6::Core)
  set_target_properties(config_profiles_tests PROPERTIES FOLDER "Tests")
  add_test(NAME parallel_config_profiles COMMAND config_profiles_tests "${CMAKE_CURRENT_SOURCE_DIR}")
  set_tests_properties(parallel_config_profiles PROPERTIES ENVIRONMENT_MODIFICATION
    "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>")
endif()
add_executable(MediaPipe2ManoQt
  main.cpp application.cpp application.h detector.cpp detector.h
  mediapipe_bridge/bridge.h)
source_group("Source Files" FILES main.cpp application.cpp detector.cpp)
source_group("Header Files" FILES application.h detector.h mediapipe_bridge/bridge.h)
target_link_libraries(MediaPipe2ManoQt PRIVATE mano_core Qt6::Widgets ${OpenCV_LIBS})
target_include_directories(MediaPipe2ManoQt PRIVATE ${OpenCV_INCLUDE_DIRS} mediapipe_bridge)
set_target_properties(MediaPipe2ManoQt PROPERTIES
  FOLDER "Application"
  VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  VS_DEBUGGER_COMMAND_ARGUMENTS "--root \"${CMAKE_CURRENT_SOURCE_DIR}\" --mode interaction"
  VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/Bin;%PATH%")
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT MediaPipe2ManoQt)

if(WIN32)
  get_target_property(M2M_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
  get_filename_component(M2M_QT_BIN_DIR "${M2M_QMAKE_EXECUTABLE}" DIRECTORY)
  find_program(M2M_WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${M2M_QT_BIN_DIR}" REQUIRED NO_DEFAULT_PATH)
  file(GLOB M2M_OPENCV_RUNTIME_DLLS "${OpenCV_DIR}/x64/vc16/bin/*.dll")
  list(FILTER M2M_OPENCV_RUNTIME_DLLS EXCLUDE REGEX "d\\.dll$")
  get_filename_component(M2M_VC_TOOLS_VERSION_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
  get_filename_component(M2M_VC_TOOLS_VERSION_DIR "${M2M_VC_TOOLS_VERSION_DIR}/../../.." ABSOLUTE)
  get_filename_component(M2M_VC_ROOT "${M2M_VC_TOOLS_VERSION_DIR}/../../.." ABSOLUTE)
  file(GLOB M2M_VC_REDIST_DIRS LIST_DIRECTORIES true
    "${M2M_VC_ROOT}/Redist/MSVC/*/x64/Microsoft.VC143.CRT")
  list(SORT M2M_VC_REDIST_DIRS COMPARE NATURAL ORDER DESCENDING)
  if(M2M_VC_REDIST_DIRS)
    list(GET M2M_VC_REDIST_DIRS 0 M2M_VC_REDIST_DIR)
    file(GLOB M2M_VC_RUNTIME_DLLS "${M2M_VC_REDIST_DIR}/*.dll")
  endif()
  add_custom_command(TARGET MediaPipe2ManoQt POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "$<TARGET_FILE:Open3D::Open3D>" "$<TARGET_FILE:Open3D::tbb>"
      ${M2M_OPENCV_RUNTIME_DLLS} ${M2M_VC_RUNTIME_DLLS} "$<TARGET_FILE_DIR:MediaPipe2ManoQt>"
    COMMAND "${M2M_WINDEPLOYQT_EXECUTABLE}" --release --no-translations --no-compiler-runtime
      "$<TARGET_FILE:MediaPipe2ManoQt>"
    COMMAND_EXPAND_LISTS
    COMMENT "Deploying Qt, OpenCV, Open3D and TBB runtime files"
    VERBATIM)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Bin/mediapipe_hands.dll")
    add_custom_command(TARGET MediaPipe2ManoQt POST_BUILD
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Bin/mediapipe_hands.dll"
        "$<TARGET_FILE_DIR:MediaPipe2ManoQt>"
      COMMENT "Deploying MediaPipe detector DLL"
      VERBATIM)
  endif()
endif()

file(GLOB PROJECT_CONFIGS CONFIGURE_DEPENDS "configs/*.json" "configs/benchmarks/*.json")
add_custom_target(project_resources SOURCES
  MediaPipe2ManoQt.sln README.md docs/README_FULL.md CMakePresets.json CMakeUserPresets.json.example
  ${PROJECT_CONFIGS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES
  MediaPipe2ManoQt.sln README.md docs/README_FULL.md CMakePresets.json CMakeUserPresets.json.example
  ${PROJECT_CONFIGS})
set_target_properties(project_resources PROPERTIES FOLDER "Resources")

add_custom_target(mediapipe_bridge_sources SOURCES
  mediapipe_bridge/bridge.cc mediapipe_bridge/bridge.h mediapipe_bridge/BUILD
  assets/hands_0_10_9.pbtxt)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES
  mediapipe_bridge/bridge.cc mediapipe_bridge/bridge.h mediapipe_bridge/BUILD
  assets/hands_0_10_9.pbtxt)
set_target_properties(mediapipe_bridge_sources PROPERTIES FOLDER "ThirdParty/MediaPipe")

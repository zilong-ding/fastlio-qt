cmake_minimum_required(VERSION 4.0)
project(fastlio_qt)
set(QT_MAJOR_VERSION 6 CACHE STRING "Force Qt6")
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release"))
    add_compile_definitions(-DELPP_DISABLE_DEBUG_LOGS)
ENDIF ()

# 设置并发现Qt6.9.0
#set(Qt6_DIR "/home/dzl/Qt/6.9.1/gcc_64/lib/cmake/Qt6")
#set(CMAKE_INSTALL_RPATH "/home/dzl/Qt/6.9.1/gcc_64/lib")
#set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets LinguistTools Thread)
find_package(Qt6 REQUIRED COMPONENTS Widgets LinguistTools Multimedia MultimediaWidgets Core Gui)
if(Qt6_FOUND)
    message(STATUS "Qt install path: ${Qt6_DIR}")
endif ()


find_package(OpenCV REQUIRED)
IF(OpenCV_FOUND)
    message(STATUS "----OpenCV_DIRS: ${OpenCV_DIR}")
    message(STATUS "----OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "----OpenCV_INCLUDES: ${OpenCV_INCLUDES}")
    message(STATUS "----OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
    message(STATUS "----OpenCV_LIBS: ${OpenCV_LIBS}")
    message(STATUS "----OpenCV_DEFINITIONS: ${OpenCV_DEFINITIONS}")
    message(STATUS "----OpenCV library status:")
    message(STATUS "----version:   ${OpenCV_VERSION}")
ENDIF(OpenCV_FOUND)
include_directories("${OpenCV_INCLUDE_DIRS}")

#寻找VTK库
#set(VTK_DIR "/usr/local/share/vtk-9.4.0/lib/cmake/vtk-9.4")
find_package(VTK COMPONENTS
        CommonCore
        CommonDataModel
        FiltersSources
        GUISupportQt
        InteractionStyle
        RenderingContextOpenGL2
        RenderingCore
        RenderingFreeType
        RenderingGL2PSOpenGL2
        RenderingOpenGL2
        GUISupportQt
        RenderingQt
)
if(VTK_FOUND)
    message("VTK_DIRS: ${VTK_DIR}")
    message("VTK_LIBRARIES: ${VTK_LIBRARIES}")
endif ()

#寻找PCL库
#set(PCL_DIR "/usr/local/share/pcl-1.15")
find_package(PCL REQUIRED)
IF(PCL_FOUND)
    message(STATUS "----PCL_DIRS: ${PCL_DIR}")
    message(STATUS "----PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
    message(STATUS "----PCL_INCLUDES: ${PCL_INCLUDES}")
    message(STATUS "----PCL_LIBRARIES: ${PCL_LIBRARIES}")
    message(STATUS "----PCL_LIBS: ${PCL_LIBS}")
    message(STATUS "----PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
ENDIF(PCL_FOUND)

find_package(ZeroMQ CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(Eigen3 REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})

add_executable(fastlio_qt main.cpp
        Msgs/dataTypes.h
        messageReceiver.cpp
        messageReceiver.h
        SLAM/FastLio/MainWorker.cpp
        SLAM/FastLio/MainWorker.h
        SLAM/FastLio/Preprocess.cpp
        SLAM/FastLio/Preprocess.h
        SLAM/FastLio/ImuProcess.cpp
        SLAM/FastLio/ImuProcess.h
        IKFoM_toolkit/ikfomDataTypes.h
        ikd-Tree/ikd_Tree.cpp
        ikd-Tree/ikd_Tree.h
        tools/Exp_Math.h
        Display.cpp
        Display.h
#        widget3d/EzQVTKInteractor.cxx widget3d/EzQVTKInteractor.h
#        widget3d/EzQVTKInteractorAdapter.cxx widget3d/EzQVTKInteractorAdapter.h
#        widget3d/EzQVTKOpenGLNativeWidget.cxx widget3d/EzQVTKOpenGLNativeWidget.h
#        widget3d/EzQVTKRenderWindowAdapter.cxx widget3d/EzQVTKRenderWindowAdapter.h
#        widget3d/EzQVTKInteractorInternal.h
        mainwindow.ui
        SLAM/SLAMBase.cpp
        SLAM/SLAMBase.h
        Sensors/SensorType.h
        tools/utills.h
        Sensors/SensorBase.h
        SLAM/AlgorithmMainBase.h
        SLAM/FastLio/FastLioMain.cpp
        SLAM/FastLio/FastLioMain.h
        SLAM/FastLio/FastlioConfig.h


)
target_link_libraries(fastlio_qt PRIVATE libzmq libzmq-static)
target_link_libraries(fastlio_qt PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(fastlio_qt PRIVATE Qt::Core Qt::Widgets)
target_link_libraries(fastlio_qt PRIVATE ${PCL_LIBRARIES} ${VTK_LIBRARIES})

# vtk_module_autoinit is needed
vtk_module_autoinit(
        TARGETS fastlio_qt
        MODULES ${VTK_LIBRARIES}
)




#find_package(Ceres CONFIG REQUIRED)
#target_link_libraries(main PRIVATE Ceres::ceres)

#find_package(GTSAM CONFIG REQUIRED)
#target_link_libraries(main PRIVATE gtsam)

#find_package(g2o CONFIG REQUIRED)
#target_link_libraries(main PRIVATE g2o::core g2o::stuff g2o::types_icp g2o::types_sba)
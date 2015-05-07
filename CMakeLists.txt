cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")
#set(CMAKE_BUILD_TYPE Release)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

add_definitions(-DCVD_HAVE_TOON)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate dynamic parameters
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
rosbuild_add_boost_directories()


set(PTAM_SOURCE_FILES     
                src/ParamsPTAMM.cpp
		src/main.cc
		src/GLWindow2.cc
		src/GLWindowMenu.cc
		src/System.cc
		src/ATANCamera.cc
		src/KeyFrame.cc
		src/MapPoint.cc
		src/Map.cc
		src/SmallBlurryImage.cc
		src/ShiTomasi.cc
		src/HomographyInit.cc
		src/MapMaker.cc
		src/Bundle.cc
		src/PatchFinder.cc
		src/Relocaliser.cc
		src/MiniPatch.cc
		src/MapViewer.cc
		src/Tracker.cc
		src/tinyxml.cc
		src/tinyxmlerror.cc
		src/tinyxmlparser.cc
		src/MapLockManager.cc
		src/MD5.cc
		src/MD5Wrapper.cc
		src/MapSerializer.cc
		src/Utils.cc
)


set(PTAM_HEADER_FILES        
		include/ParamsPTAMM.h
		src/GLWindow2.h
		src/GLWindowMenu.h
		src/System.h
		src/ATANCamera.h
		src/KeyFrame.h
		src/MapPoint.h
		src/Map.h
		src/SmallBlurryImage.h
		src/ShiTomasi.h
		src/HomographyInit.h
		src/MapMaker.h
		src/Bundle.h
		src/PatchFinder.h
		src/Relocaliser.h
		src/MiniPatch.h
		src/MapViewer.h
		src/Tracker.h
		src/tinyxml.h
		src/MapLockManager.h
		src/MD5.h
		src/MD5Wrapper.h
		src/MapSerializer.h
		src/Utils.h
		src/LevelHelpers.h
		src/AxesArray.h
		
)
	
SET(CALIBRATOR_FILES     
  src/CameraCalibrator.cc 
  src/GLWindowMenu.cc 
  src/CalibCornerPatch.cc           
  src/CalibImage.cc            
  src/GLWindow2.cc	
  src/ATANCamera.cc
  src/VideoSource_ros.cpp 
  src/ParamsPTAMM.cpp
)	

include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/TooN
  ${PROJECT_SOURCE_DIR}/ThirdParty/libcvd
  ${PROJECT_SOURCE_DIR}/ThirdParty/gvars
  ${PROJECT_SOURCE_DIR}/ThirdParty/agast/include
)  

link_directories(
  ${PROJECT_SOURCE_DIR}/ThirdParty/libcvd
  #${PROJECT_SOURCE_DIR}/ThirdParty/lib3ds-20080909
  ${PROJECT_SOURCE_DIR}/ThirdParty/gvars
  ${PROJECT_SOURCE_DIR}/ThirdParty/agast/lib
)

set(PTAM_LIBRARIES GL glut lapack blas cvd GVars3 GLU agast)

add_definitions(-DKF_REPROJ)

# PTAM executable

rosbuild_add_executable(ptam ${PTAM_SOURCE_FILES} ${PTAM_HEADER_FILES})
#rosbuild_add_compile_flags(ptam -L MY_CUSTOM_LINK_PATH -lGVars3 -lcvd -lcv -lcxcore -lhighgui -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3 -fexceptions -fopenmp)
#rosbuild_add_compile_flags(ptam -Wall -O3 -D__LINUX__ -mfpmath=sse -mmmx -msse -msse2 -msse3 -fexceptions -fopenmp)
target_link_libraries(ptam ${PTAM_LIBRARIES} ${OpenCV_LIBS})
rosbuild_link_boost(ptam thread system signals)

# Camera calibrator 
rosbuild_add_executable(cameracalibrator ${CALIBRATOR_FILES})
#rosbuild_add_compile_flags(ptam -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3 -lGLU)
target_link_libraries(cameracalibrator ${PTAM_LIBRARIES})

# PTAMM remote control
# this is unfortunately neccessary to close app, when opencvwindow gets closed
include(FindPkgConfig)
if (PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)
   pkg_check_modules(GTK gtk+-2.0)
   if (GTK_FOUND)
      include_directories(${GTK_INCLUDE_DIRS})
      link_directories(${GTK_LIBRARY_DIRS})
      add_definitions(-DHAVE_GTK)
      # Hack around gdk-pixbuf move issues for now
      # http://ubuntuforums.org/showthread.php?t=1596930
      include_directories(/usr/include/gdk-pixbuf-2.0)
   endif(GTK_FOUND)
endif(PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)

rosbuild_add_executable(remote_ptamm src/RemotePtamm.cpp)
target_link_libraries(remote_ptamm ${GTK_LIBRARIES} ${OpenCV_LIBS})

rosbuild_add_executable(ptamm_visualizer src/PTAMMVisualizer.cpp)

rosbuild_add_executable(ptamm_subscribe_test src/ptam_subscribe_test.cpp)

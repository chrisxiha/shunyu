cmake_minimum_required(VERSION 2.8.3)

project(shunyu)

catkin_package(
)

set(CMAKE_CONFIGURATION_TYPES Release)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED
        roslib
        roscpp
        cv_bridge
        sensor_msgs
        tf
        tf_conversions
        image_transport
        image_geometry
        # perc_log
	#        common_services
        )

find_package(PCL REQUIRED)

set(OpenCV_INCLUDE_DIRS
/opt/ros/kinetic/include/opencv-3.3.1-dev
	)

set(OpenCV_LIB_DIR
/opt/ros/kinetic/lib/x86_64-linux-gnu
	)

set(OpenCV_LIBS
	opencv_core
	opencv_highgui
	opencv_imgcodecs
	)


find_package( OpenCV 3 REQUIRED )

#find_path(OPENCV_INCLUDE_DIR opencv.hpp
#        PATH_SUFFIXES include)

include_directories(include 
                    #$(OPENCV_INCLUDE_DIR)
                    ${OpenCV_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

link_directories(/home/jianwei/shunyu/ubuntu/shunyu_ws/src/shunyu/libs ${OpenCV_LIB_DIR} )
#link_directories(${CMAKE_SOURCE_DIR}/perception_core/stellar/3rd)



set(samples
  #depth_ir_rgb_visualizer
  tof_dev_sdk_demo-package-api
  #rgb_sample
  #device_parameters_sample
  #depth_event_sample
  #depth_ir_sample
)

foreach(sample ${samples})
  #set_target_properties(${sample} PROPERTIES FOLDER ${sample})
  add_executable(${sample} /home/jianwei/shunyu/ubuntu/shunyu_ws/src/shunyu/demo/${sample}.cpp)
  target_link_directories(${sample} PUBLIC /home/jianwei/shunyu/ubuntu/shunyu_ws/src/shunyu/libs ${OpenCV_LIB_DIR} )
  message("ff message to display" )
  add_definitions(-DLINUX)
  target_link_libraries(${sample}
      pthread
      tof_dev_sdk
      #${CMAKE_SOURCE_DIR}/perception_core/stellar/3rd/libopencv_core.so
      #${CMAKE_SOURCE_DIR}/perception_core/stellar/3rd/libopencv_highgui.so
      #${CMAKE_SOURCE_DIR}/perception_core/stellar/3rd/libopencv_imgcodecs.so
      #${CMAKE_SOURCE_DIR}/perception_core/stellar/3rd/libopencv_imgproc.so
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBS}
    )

  set_target_properties(${sample} PROPERTIES TIMEOUT 10)
  set_target_properties(${sample} PROPERTIES SKIP_RETURN_CODE 77)
  set_target_properties(${sample} PROPERTIES FOLDER "src")
endforeach()

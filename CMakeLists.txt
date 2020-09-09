cmake_minimum_required(VERSION 3.0.2)

project(YPPF)

# find extern include files
find_package(Eigen3 REQUIRED)
find_package(Boost  REQUIRED COMPONENTS 
	filesystem
)
# find_package(OpenCV REQUIRED COMPONENTS
#	core	
#	surface_matching
#	imgproc
#	dnn
#	highgui
#	)
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
	message("OpenCV Found !! " ${OpenCV_INCLUDE_DIRS})
endif (OpenCV_FOUND)

#find_package(PCL    REQUIRED COMPONENTS
#	common
#	io
#	features
#	kdtree
#	filters
#	surface
#	)
find_package(PCL REQUIRED)
# still need to find the pakcage for K4A ?

# header file path
include_directories(
	include
	${Eigen3_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
link_directories(
	${Eigen3_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}	
	${PCL_LIBRARY_DIRS}
	${OpenCV_LIBRARY_DIRS}
)
add_definitions(
	${Eigen3_DEFINITIONS}
	${Boost_DEFINITIONS}
	${PCL_DEFINITIONS}
	${OpenCV_DEFINITIONS}
	)
# set environment variables
# set extern libraries
# add executabel file
message(STATUS "Binary Dir: " ${YPPF_BINARY_DIR})
message(STATUS "Source Dir: " ${YPPF_SOURCE_DIR})

add_executable(YOLO_PPF_Pose_Estimation src/YOLO_cropping_ppf_test.cpp)
target_compile_features(YOLO_PPF_Pose_Estimation PUBLIC cxx_std_11)
target_link_libraries(YOLO_PPF_Pose_Estimation
	${Eigen3_LIBS}
	#${OpenCV_CORE_LIBRARIES}
	#${OpenCV_SURFACE_MATCHING_LIBRARIES}
	#${OpenCV_IMGPROC_LIBRARIES}
	#${OpenCV_DNN_LIBRARIES}
	#${OpenCV_HIGHGUI_LIBRARIES}
	${OpenCV_LIBS}	
	${PCL_COMMON_LIBRARIES}
	${PCL_IO_LIBRARIES}	
	${PCL_FEATURES_LIBRARIES}	
	${PCL_KDTREE_LIBRARIES}	
	${PCL_FILTERS_LIBRARIES}	
	${PCL_SURFACE_LIBRARIES}	
	${Boost_FILESYSTEM_LIBRARIES}
	${Boost_SYSTEM_LIBRARIES}	
	${Boost_THREAD_LIBRARIES}		
	)
# add link library

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
message(${PCL_INCLUDE_DIRS})
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
# add_definitions(
#	${Eigen3_DEFINITIONS}
#	${Boost_DEFINITIONS}
#	${PCL_DEFINITIONS}
#	${OpenCV_DEFINITIONS}
#	)
# set environment variables
# set extern libraries
# add executabel file
message(STATUS "Binary Dir: " ${YPPF_BINARY_DIR})
message(STATUS "Source Dir: " ${YPPF_SOURCE_DIR})
message(STATUS "visulization Dir: " ${PCL_VISUALIZATION_LIBRARIES})
add_executable(YOLO_PPF_Pose_Estimation src/YOLO_cropping_ppf_test.cpp)
add_executable(crophull_test src/test/crophull_test.cpp)
add_executable(mls_filtering src/mls_filter.cpp)
add_executable(sac_cylinder src/sac_cylinder.cpp)
add_executable(sac_sphere src/sac_sphere.cpp)
add_executable(YOLO_PPF_Multi src/YOLO_cropping_ppf_test_multi.cpp)
add_executable(YOLO_Detection src/ObjectDetection-YOLO/object_detection_yolo.cpp)
target_compile_features(YOLO_PPF_Pose_Estimation PUBLIC cxx_std_11)
target_compile_features(crophull_test PUBLIC cxx_std_11)
target_compile_features(mls_filtering PUBLIC cxx_std_11)
target_compile_features(sac_cylinder PUBLIC cxx_std_11)
target_compile_features(sac_sphere PUBLIC cxx_std_11)
target_compile_features(YOLO_PPF_Multi PUBLIC cxx_std_11)
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
target_link_libraries(YOLO_PPF_Multi
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
target_link_libraries(YOLO_Detection
	${Eigen3_LIBS}
	${OpenCV_LIBS}
	)
target_link_libraries(crophull_test
        ${Eigen3_LIBS}
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
target_link_libraries(mls_filtering
        ${Eigen3_LIBS}
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${PCL_FEATURES_LIBRARIES}
        ${PCL_KDTREE_LIBRARIES}
        ${PCL_FILTERS_LIBRARIES}
        ${PCL_SURFACE_LIBRARIES}
	${PCL_VISUALIZATION_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARIES}
        ${Boost_SYSTEM_LIBRARIES}
        ${Boost_THREAD_LIBRARIES}
        )
target_link_libraries(sac_cylinder
	${Eigen3_LIBS}
	${PCL_LIBRARIES}
	${PCL_COMMON_LIBRARIES}
	${PCL_IO_LIBRARIES}
	${PCL_FEATURES_LIBRARIES}
        ${PCL_KDTREE_LIBRARIES}
        ${PCL_FILTERS_LIBRARIES}
        ${PCL_SURFACE_LIBRARIES}
	${PCL_SAMPLE_CONSENSUS_LIBRARIES}
	${PCL_SEGMENTATION_LIBRARIES}
	${PCL_VISUALIZATION_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARIES}
        ${Boost_SYSTEM_LIBRARIES}
        ${Boost_THREAD_LIBRARIES}
	)
target_link_libraries(sac_sphere
	${Eigen3_LIBS}
	${PCL_LIBRARIES}
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${PCL_FEATURES_LIBRARIES}
        ${PCL_KDTREE_LIBRARIES}
        ${PCL_FILTERS_LIBRARIES}
        ${PCL_SURFACE_LIBRARIES}
        ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
        ${PCL_SEGMENTATION_LIBRARIES}
        ${PCL_VISUALIZATION_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARIES}
        ${Boost_SYSTEM_LIBRARIES}
        ${Boost_THREAD_LIBRARIES}
        )
# add link library

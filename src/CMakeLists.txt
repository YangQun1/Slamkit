# 将工程目录下存放Find<name>.cmake文件的CMakeModules文件夹路径
# 添加到CMAKE_MODULE_PATH中,以供find_package命令使用
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# 查找依赖的库
find_package(OpenCV 3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters)
# find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)

# 添加包含路径
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
# include_directories(${EIGEN_INCLUDE_DIRS})
include_directories( "/usr/include/eigen3" )
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

# 定义源文件列表变量
list (APPEND SOURCEFILES 
		generate_point_cloud.cpp
		pinhole_camera.cpp
		detect_features.cpp
		align_image.cpp
		param_reader.cpp
		frame.cpp
		image_reader.cpp
		g2o_types_costom.cpp
		map.cpp 
		map_point.cpp
		visual_odometry.cpp)

# 生成共享库文件并链接第三方库
add_library(SlamKit SHARED  ${SOURCEFILES})
		
target_link_libraries(SlamKit 
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
		${Sophus_LIBRARIES}
		# ${G2O_LIBS}
		g2o_core g2o_stuff g2o_types_sba)

# 生成可执行文件并链接共享库
add_executable(main main.cpp)
target_link_libraries(main SlamKit)
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(functionalMappingHelper src/Exceptions/FMGeneralException.cpp
										  src/GeometricalConstructs/Axis_Angle.cpp
										  src/GeometricalConstructs/BasicDefinitions.cpp
										  src/GeometricalConstructs/GeometricalComputations.cpp
										  src/GeometricalConstructs/Line.cpp
										  src/GeometricalConstructs/LineSegment.cpp
										  src/GeometricalConstructs/Plane.cpp
										  src/GeometricalConstructs/pnpoly.cpp
										  src/GeometricalConstructs/Point.cpp
										  src/GeometricalConstructs/PointOfIntersection.cpp
										  src/GeometricalConstructs/Polygon.cpp
										  src/GeometricalConstructs/Pose.cpp
										  src/GeometricalConstructs/Quaternion.cpp
										  src/GeometricalConstructs/Ray.cpp
										  src/GeometricalConstructs/RotationMatrix.cpp
										  src/GeometricalConstructs/Vector.cpp
										  src/MathematicalConstructs/LinearEquationSystem2D.cpp
										  src/MathematicalConstructs/Matrix2X2.cpp
										  src/HelperClasses/Std2DFloatVector.cpp
										  src/HelperClasses/StringHelper.cpp
										  src/Visualization/VisualizationOctave.cpp)
										  
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

cmake_minimum_required(VERSION 3.30)

project(X02_Lite)

IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
ENDIF ()
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)


message("-------------Path Information-------------")

set(X02_LITE_INCLUDE ${CMAKE_HOME_DIRECTORY}/include)
message("Set X02 Lite Api header files path: ${X02_LITE_INCLUDE}")

set(X02_LITE_LIB ${CMAKE_HOME_DIRECTORY}/lib)
message("Set X02 Lite library path: ${X02_LITE_LIB}")

set(SRC_DIR ${CMAKE_HOME_DIRECTORY}/src)
message("Set X02_Lite source codes path: ${SRC_DIR}")

set(UTILS_DIR ${CMAKE_HOME_DIRECTORY}/utils)
message("Set X02_Lite utils path: ${UTILS_DIR}")

set(LIB_X02_LITE ${X02_LITE_PKG_LIB}/libX02liteApi.so)
message("Set X02_Lite Api library path: ${LIB_X02_LITE}")

set(lcm_DIR ${CMAKE_HOME_DIRECTORY}/third_party/lcm)
message("Set LCM path: ${lcm_DIR}")

message("------------------------------------------")


set(dirLCM "third_party/lcm")
set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")
set(dirQuill "third_party/quill")
set(dirQP "third_party/qpOASES")
set(libURDF "third_party/urdfdom")
set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
set(incMujoco "third_party/mujoco")


set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirJson} ${dirQuill} ${dirQP} ${dirLCM})
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco})

include_directories(${allInc})
include_directories("include")


include(FetchContent)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/lcm)
set(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/lcm)
FetchContent_Declare(lcm SOURCE_DIR "${CMAKE_HOME_DIRECTORY}/third_party/lcm")
FetchContent_MakeAvailable(lcm)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib)



message(${CMAKE_SYSTEM_PROCESSOR})
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	list(APPEND allLib "third_party/mujoco/lin_arm64")
else()
	list(APPEND allLib "third_party/mujoco/lin_x64")
endif()
link_directories(${allLib})

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp src/*.cpp)
file(GLOB HEADER_FILES *.h include/*.h *.hpp include/*.hpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})




set(corLibs pinocchio urdfdom_model tinyxml console_bridge jsoncpp quill qpOASES)
set(simLibs glfw3)



#根据架构，处理连接库
set(sysCoreLibs)
set(sysSimLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysSimLibs dl)#arm 需要 -ldl
else()
	message("linux x64")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})




#生成控制核心库
add_library(core ${SOURCES})
target_link_libraries(core ${sysCoreLibs} pthread)

#生成仿真可执行文件
# add_executable(walk_mpc_wbc demo/walk_mpc_wbc.cpp)
# target_link_libraries(walk_mpc_wbc core mujoco ${sysSimLibs} dl)



add_executable(tsttst ${SRC_DIR}/test.cpp ${SRC_DIR}/x02_utils.cpp)
target_link_libraries(tsttst lcm core mujoco ${sysSimLibs} dl)
set_target_properties(tsttst PROPERTIES
            CXX_STANDARD 17
            CXX_EXTENSIONS OFF
            )

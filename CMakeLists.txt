cmake_minimum_required(VERSION 3.22)

project(X02_Lite)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===============================================================
# Set Path and Information

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

set(THIRD_PARTY_DIR ${CMAKE_HOME_DIRECTORY}/third_party)
message("Set Third-Party path: ${THIRD_PARTY_DIR}")
message("------------------------------------------")

set(dirLCM "third_party/lcm")
set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")
set(dirQuill "third_party/quill")
set(dirQP "third_party/qpOASES")



# ----------Without Arm64 Libraries----------
set(dirGtest "third_party/gtest")
set(dirLibSodium "third_party/libsodium")
set(dirSQLite3 "third_party/sqlite3")
set(dirZeroMQ "third_party/zeromq")
set(incBehaviorTreeCPP "third_party/behaviortree_cpp")
# -------------------------------------------


set(libURDF "third_party/urdfdom")
set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
set(incMujoco "third_party/mujoco")

set(allDir
    ${dirEigen}
    ${dirGlfw}
    ${dirPino}
    ${dirJson}
    ${dirQuill}
    ${dirQP}
    ${dirLCM}
    ${dirGtest}
    ${dirLibSodium}
    ${dirSQLite3}
    ${dirZeroMQ}
    )
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco} ${incBehaviorTreeCPP})

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp src/*.cpp)
file(GLOB HEADER_FILES *.h include/*.h *.hpp include/*.hpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})

set(corLibs
    pinocchio
    urdfdom_model
    tinyxml
    console_bridge
    jsoncpp
    quill
    qpOASES
    gtest
    libsodium
    sqlite3
    zeromq)
set(simLibs glfw3)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib)

# ---------------------------------------------------------------

# ===============================================================
# Set Libraries based on Archiectures
message(${CMAKE_SYSTEM_PROCESSOR})
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES
                                              "aarch64")
  list(APPEND allLib "third_party/mujoco/lin_arm64")
else()
  list(APPEND allLib "third_party/mujoco/lin_x64")
endif()
link_directories(${allLib})

set(sysCoreLibs)
set(sysSimLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES
                                              "aarch64")
  message("linux arm64")
  foreach(lib ${corLibs})
    list(APPEND sysCoreLibs ${lib}_lin_arm64)
  endforeach()
  foreach(lib ${simLibs})
    list(APPEND sysSimLibs ${lib}_lin_arm64)
  endforeach()
  list(APPEND sysSimLibs dl)
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
# ---------------------------------------------------------------

# ===============================================================
# Set Includes and Core Libraries
include_directories(include ${allDir} ${allInc} ${THIRD_PARTY_DIR})

add_library(core ${SOURCES})
target_link_libraries(core ${sysCoreLibs} pthread)
# ---------------------------------------------------------------

# ===============================================================
# Make Target
add_executable(tsttst ${SRC_DIR}/test.cpp ${SRC_DIR}/x02_utils.cpp)
target_link_libraries(tsttst core mujoco ${sysSimLibs} dl)
set_target_properties(tsttst PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
# ---------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES
                  face.msg
                  faces.msg
                  faces_cropped.msg
)

generate_messages(
                    DEPENDENCIES
                    std_msgs 
                    sensor_msgs
)

find_package(Boost COMPONENTS system program_options REQUIRED)
find_package(noos REQUIRED)
if (NOT NOOS_LIBRARIES)
    message(STATUS "No NOOS library found, looking manually")
    find_library(NOOS_LIBRARIES NAMES noos PATHS "/usr/local/lib")
    set(NOOS_INCLUDE_DIRS "/usr/local/include")
endif()
message(STATUS "${NOOS_LIBRARIES}")
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV  REQUIRED)

catkin_package(
            CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime 
              )

set(NOOS_LIBRARIES_PACK ${NOOS_LIBRARIES} 
                        ${OPENSSL_LIBRARIES} 
                        ${Boost_LIBRARIES}
                        ${CMAKE_THREAD_LIBS_INIT})

include_directories(${catkin_INCLUDE_DIRS} 
                    ${NOOS_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                   )

add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/face_detector.cpp
                                    src/noos_bridge.cpp
                                    src/options.cpp
		      )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-std=gnu++1y -Wall -s -O3 -mtune=native -Wno-write-strings -Wno-narrowing")
target_link_libraries(${PROJECT_NAME}_node 
                      ${catkin_LIBRARIES}
                      ${NOOS_LIBRARIES_PACK}
                      ${OpenCV_LIBRARIES}
                      )

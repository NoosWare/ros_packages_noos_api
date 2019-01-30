cmake_minimum_required(VERSION 2.8.3)
project(chatbot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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

catkin_package(
               CATKIN_DEPENDS roscpp std_msgs
              )

set(NOOS_LIBRARIES_PACK ${NOOS_LIBRARIES} 
                        ${OPENSSL_LIBRARIES} 
                        ${Boost_LIBRARIES}
                        ${CMAKE_THREAD_LIBS_INIT})

include_directories(${catkin_INCLUDE_DIRS} 
                    ${NOOS_INCLUDE_DIRS}
                   )

add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/options.cpp
                                    src/chat.cpp
		      )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-std=gnu++1y -Wall -s -O3 -mtune=native -Wno-write-strings -Wno-narrowing")
target_link_libraries(${PROJECT_NAME}_node 
                      ${catkin_LIBRARIES}
                      ${NOOS_LIBRARIES_PACK}
                      )

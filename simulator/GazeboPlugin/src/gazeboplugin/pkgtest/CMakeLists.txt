#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME pkgtest)

find_package(GTest REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/msgproto
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/export/include/
${GTEST_INCLUDE_DIRS}
${GMOCK_INCLUDE_DIRS}
)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/bin )

ADD_EXECUTABLE( pluginTest pluginTest.cpp  )

TARGET_LINK_LIBRARIES( pluginTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( forceTest forceTest.cpp ) 

TARGET_LINK_LIBRARIES( forceTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

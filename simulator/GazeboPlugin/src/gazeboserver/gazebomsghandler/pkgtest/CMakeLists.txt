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

ADD_EXECUTABLE( imuHandlerTest imuHandlerTest.cpp  )

TARGET_LINK_LIBRARIES( imuHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( imageHandlerTest imageHandlerTest.cpp )

TARGET_LINK_LIBRARIES( imageHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )



ADD_EXECUTABLE( laserHandlerTest laserHandlerTest.cpp )

TARGET_LINK_LIBRARIES( laserHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getTopicHandlerTest getTopicHandlerTest.cpp )

TARGET_LINK_LIBRARIES( getTopicHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( maxStepSizeHandlerTest maxStepSizeHandlerTest.cpp )

TARGET_LINK_LIBRARIES( maxStepSizeHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( applyJointTorqueHandlerTest applyJointTorqueHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( applyJointTorqueHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( applyLinkWrenchHandlerTest applyLinkWrenchHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( applyLinkWrenchHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getPoseHandlerTest getPoseHandlerTest.cpp  )  

TARGET_LINK_LIBRARIES( getPoseHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getModelInfoHandlerTest getModelInfoHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( getModelInfoHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )



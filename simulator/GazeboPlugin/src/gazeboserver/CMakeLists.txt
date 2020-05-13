#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME server)

include_directories(${PROJECT_SOURCE_DIR}/${SUB_PROJECT_NAME}/include/)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/msgproto
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/export/include/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/lib)

file(GLOB SOURCE_FILES
gazebomsghandler/ApplyLinkWrenchMsgHandler.cpp 
gazebomsghandler/ApplyJointTorqueMsgHandler.cpp
gazebomsghandler/GetPoseMsgHandler.cpp
gazebomsghandler/GetTopicListMsgHandler.cpp
gazebomsghandler/GetModelInfoMsgHandler.cpp
RequestCoSimulationHandler.cpp
StopCoSimulationHandler.cpp
gazebomsghandler/SubscribeMsgHandler.cpp
gazebomsghandler/StepSimulationMsgHandler.cpp
gazebomsghandler/ResetSimulationMsgHandler.cpp
gazebomsghandler/SubscribeImageMsgHandler.cpp
gazebomsghandler/GetImageMsgHandler.cpp
gazebomsghandler/SubscribeLaserMsgHandler.cpp
gazebomsghandler/GetLaserMsgHandler.cpp
gazebomsghandler/SubscribeImuMsgHandler.cpp
gazebomsghandler/GetImuMsgHandler.cpp
gazebomsghandler/MaxSimulationStepMsgHandler.cpp
gazebosensors/SensorContainer.cpp
gazebosensors/GazeboCameraSensor.cpp
gazebosensors/GazeboIMUSensor.cpp
gazebosensors/GazeboLaserSensor.cpp
gazebosensors/GazeboSensorSubscriber.cpp
gazeboapply/GazeboApplyCommander.cpp
GazeboWorldImpl.cpp
MsgDispatcher.cpp
GazeboServerCallback.cpp
)

add_library(GazeboCoSimServer SHARED GazeboServer.cpp ${SOURCE_FILES})

target_link_libraries(GazeboCoSimServer
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
)

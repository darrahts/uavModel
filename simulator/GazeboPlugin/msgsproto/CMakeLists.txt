#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME msgproto)

execute_process (
    COMMAND bash -c "sed -e '1 i syntax=\"proto2\";' -e '2 a package robotics.gazebotransport;' ${PROJECT_SOURCE_DIR}/msgsproto/CoSimMsgs.proto > ${PROJECT_SOURCE_DIR}/msgsproto/robotics.gazebotransport.CoSimMsgs.proto"
)
execute_process (
    COMMAND bash -c "protoc --proto_path=${PROJECT_SOURCE_DIR}/msgsproto/ --cpp_out=${PROJECT_SOURCE_DIR}/msgsproto/  ${PROJECT_SOURCE_DIR}/msgsproto/robotics.gazebotransport.CoSimMsgs.proto"
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/lib)

add_library(msgproto SHARED robotics.gazebotransport.CoSimMsgs.pb.cc robotics.gazebotransport.CoSimMsgs.pb.h)
install(TARGETS msgproto DESTINATION ${PROJECT_SOURCE_DIR}/export/lib)
/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"
#include "gazebo/physics/World.hh"

namespace robotics
{
namespace gazebotransport
{
SubscribeMsgHandler::SubscribeMsgHandler(gazebo::physics::WorldPtr world, gazebo::transport::NodePtr node)
    : m_world(world), m_node(node), m_topicList(std::make_shared<GetTopicListMsgHandler>())
{
}

SubscribeMsgHandler::~SubscribeMsgHandler()
{
}

std::string SubscribeMsgHandler::handleMessage(Packet const &msgContent)
{
    bool validTopic = false;
    /// Check input topic is present in Gazebo
    Packet newTopicList = m_topicList->GetAllTopicList();
    for (int i = 0; i < newTopicList.topic_list().data_size(); i++)
    {
        if (newTopicList.topic_list().data(i).name() == getTopicName(msgContent))
        {
            validTopic = true;
        }
    }

    Packet replyMsg;
    gazebo::common::Time timestamp = this->m_world->SimTime();

    if (validTopic)
    {
        // perform subscription and return success message
        doSubscribe(m_world, m_node, msgContent);
        replyMsg.mutable_header()->set_id(PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
        replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)timestamp.sec);
        replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds((uint64_t)timestamp.nsec);
        replyMsg.set_status(Packet_CoSimError_NONE);
    }
    else
    {
        replyMsg.mutable_header()->set_id(PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
        replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
        replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        replyMsg.set_status(Packet_CoSimError_TOPIC_NAME_INVALID);
    }

    return replyMsg.SerializeAsString(); // Returns success message
}
} // namespace gazebotransport
} // namespace robotics

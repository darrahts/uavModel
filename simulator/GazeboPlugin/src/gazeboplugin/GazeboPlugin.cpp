/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboplugin/GazeboPlugin.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/StepSimulationMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ResetSimulationMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImageMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImageMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeLaserMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetLaserMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImuMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImuMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetTopicListMsgHandler.hpp"
#include "gazebotransport/gazeboserver/StopCoSimulationHandler.hpp"
#include "gazebotransport/gazeboserver/RequestCoSimulationHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ApplyLinkWrenchMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ApplyJointTorqueMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetPoseMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetModelInfoMsgHandler.hpp"
#include "gazebotransport/gazeboserver/GazeboWorldImpl.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/MaxSimulationStepMsgHandler.hpp"
#include "gazebo/common/common.hh"

namespace robotics {
namespace gazebotransport {
GZ_REGISTER_WORLD_PLUGIN(GazeboPlugin)

void GazeboPlugin::registerMsgHandler() {
    this->m_worldInterface = std::make_shared<GazeboWorldImpl>(this->m_world);

    // Request co-simulation handler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::RequestCoSimulationHandler>(this->m_server, this->m_worldInterface));

    // Stop co-simulation handler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::StopCoSimulationHandler>(this->m_server));

    // StepSimulationMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::StepSimulationMsgHandler>(this->m_worldInterface));

    // ResetSimulationMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::ResetSimulationMsgHandler>(this->m_worldInterface));

    // SubscribeImageMsgHandler
    this->m_server->registerHandler(std::make_shared<robotics::gazebotransport::SubscribeImageMsgHandler>(this->m_world, this->m_node));

    // GetImageMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::GetImageMsgHandler>(this->m_world));

    // SubscribeLaserMsgHandler
    this->m_server->registerHandler(std::make_shared<robotics::gazebotransport::SubscribeLaserMsgHandler>(this->m_world, this->m_node));

    // GetLaserMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::GetLaserMsgHandler>(this->m_world));

    // SubscribeImuMsgHandler
    this->m_server->registerHandler(std::make_shared<robotics::gazebotransport::SubscribeImuMsgHandler>(this->m_world, this->m_node));

    // GetImuMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::GetImuMsgHandler>(this->m_world));

    // GetTopicListMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::GetTopicListMsgHandler>());

    // ApplyLinkWrenchMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::ApplyLinkWrenchMsgHandler>(this->m_world, this->m_applyCommander));

    // ApplyJointTorqueMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::ApplyJointTorqueMsgHandler>(this->m_world, this->m_applyCommander));

    // GetPoseMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::GetPoseMsgHandler>(this->m_world));

    // GetModelInfoMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::GetModelInfoMsgHandler>(this->m_world));

    // MaxSimulationStepMsgHandler
    this->m_server->registerHandler(
        std::make_shared<robotics::gazebotransport::MaxSimulationStepMsgHandler>(this->m_world));
}

void GazeboPlugin::Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    // Access gazebo world pointer
    this->m_world = _parent;

	gzdbg << "plugin loaded!! ******************************" << std::endl;

    // load parameters
    if (_sdf->HasElement("portNumber")) {
        this->m_portNumber = static_cast<uint16_t>(_sdf->GetElement("portNumber")->Get<int>());
    } else {
        this->m_portNumber = 15490;
    }

    // Create a new transport node
    this->m_node = gazebo::transport::NodePtr(new gazebo::transport::Node());

    // Initialize the node with the world name
    this->m_node->Init(_parent->Name());

    // Starts thread for server run
    this->m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
        this->m_portNumber); // Server initialized
    registerMsgHandler();
    this->m_server->run();

    // Starts thread for simulation begin update
    this->m_worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboPlugin::onWorldUpdateStart, this));

    // Starts thread for simulation end update
    this->m_worldUpdateEndEventConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
        std::bind(&GazeboPlugin::onWorldUpdateEnd, this));

    // Starts listening to time reset event
    this->m_timeResetEventConnection = gazebo::event::Events::ConnectTimeReset(
        std::bind(&GazeboPlugin::onTimeReset, this)
    );
}

void GazeboPlugin::onWorldUpdateEnd() {
    if (this->m_server->getCoSimulationStatus().first) {
        this->m_world->SetPaused(true);
    }
}

void GazeboPlugin::onTimeReset() {
    // When Gazebo simulation time reset, clear all the queued apply commands
    this->m_applyCommander.clearApplyCommands();
}

void GazeboPlugin::onWorldUpdateStart() {
    this->m_applyCommander.executeApplyCommands(this->m_world->SimTime());
}
} // namespace gazebotransport
} // namespace robotics

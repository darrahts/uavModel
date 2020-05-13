/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazeboapply/GazeboApplyCommander.hpp"

namespace robotics {
namespace gazebotransport {

GazeboApplyCommander::GazeboApplyCommander()
    : m_mutex()
    , m_jointCommands()
    , m_linkSetForceCommand()
    , m_linkSetTorqueCommand()
    , m_linkAddForceCommand()
    , m_linkAddTorqueCommand() {
}

/// insert joint commands, latter overwrites earlier commands
void GazeboApplyCommander::insertJointCommand(
    std::string const& jointName,
    std::shared_ptr<robotics::gazebotransport::JointPtrStorage> jointCommand) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_jointCommands[std::make_pair(jointName, jointCommand->m_msgContent.apply_joint_torque().index())] = jointCommand;
}

/// insert set link force/torque command
void GazeboApplyCommander::insertSetLinkCommand(
    std::string const& linkName,
    std::string const& cmdType,
    std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand) {
    if (cmdType == "FORCE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkSetForceCommand[linkName] = linkCommand;
    } else if (cmdType == "TORQUE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkSetTorqueCommand[linkName] = linkCommand;
    }
}

/// insert add link force/torque command
void GazeboApplyCommander::insertAddLinkCommand(
    std::string const& cmdType,
    std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand) {
    if (cmdType == "FORCE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkAddForceCommand.push_back(linkCommand);
    } else if (cmdType == "TORQUE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkAddTorqueCommand.push_back(linkCommand);
    }
}

/// execute all apply commands
void GazeboApplyCommander::executeApplyCommands(gazebo::common::Time const& currentTime) {
    std::lock_guard<std::mutex> lock(m_mutex);

    /// Checks stored link pointer for ADD force
    {
        auto iter = m_linkAddForceCommand.begin();
        while (iter != m_linkAddForceCommand.end()) {
            if (currentTime <= (*iter)->m_endTime) {
                // Add force if the request is still valid
                ignition::math::Vector3d forceVal(
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().fx(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().fy(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().fz());
                (*iter)->m_linkPtr->AddForce(forceVal);
                ++iter;
            } else {
                // Remove if the request is out-of-date
                iter = m_linkAddForceCommand.erase(iter);
            }
        }
    }

    /// Checks stored link pointer for ADD torque
    {
        auto iter = m_linkAddTorqueCommand.begin();
        while (iter != m_linkAddTorqueCommand.end()) {
            if (currentTime <= (*iter)->m_endTime) {
                // Add torque if the request is still valid
                ignition::math::Vector3d torqueVal(
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().tx(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().ty(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().tz());
                (*iter)->m_linkPtr->AddTorque(torqueVal);
                ++iter;
            } else {
                // Remove if the request is out-of-date
                iter = m_linkAddTorqueCommand.erase(iter);
            }
        }
    }


    /// Checks stored link pointer for SET force
    {
        auto iter = m_linkSetForceCommand.begin();
        while (iter != m_linkSetForceCommand.end()) {
            if (currentTime <= iter->second->m_endTime) {
                // Set force if the request is still valid
                ignition::math::Vector3d forceVal(
                    iter->second->m_msgContent.apply_link_wrench().fx(),
                    iter->second->m_msgContent.apply_link_wrench().fy(),
                    iter->second->m_msgContent.apply_link_wrench().fz());

                iter->second->m_linkPtr->SetForce(forceVal);

                ++iter;
            } else {
                // Remove if the request is outdated
                iter = m_linkSetForceCommand.erase(iter);
            }
        }
    }

    /// Checks stored link pointer for SET torque
    {
        auto iter = m_linkSetTorqueCommand.begin();
        while (iter != m_linkSetTorqueCommand.end()) {
            if (currentTime <= iter->second->m_endTime) {
                // Set torque if the request is still valid
                ignition::math::Vector3d torqueVal(
                    iter->second->m_msgContent.apply_link_wrench().tx(),
                    iter->second->m_msgContent.apply_link_wrench().ty(),
                    iter->second->m_msgContent.apply_link_wrench().tz());

                iter->second->m_linkPtr->SetTorque(torqueVal);

                ++iter;
            } else {
                // Remove if the request is outdated
                iter = m_linkSetTorqueCommand.erase(iter);
            }
        }
    }

    {
        /// Checks stored joint pointer and set the effort
        auto iter = m_jointCommands.begin();
        while (iter != m_jointCommands.end()) {
            // if current_time less than apply end time, keep setting effort on joint
            if (currentTime <= iter->second->m_endTime) {
                iter->second->m_jointPtr->SetForce(
                    static_cast<uint32_t>(iter->second->m_msgContent.apply_joint_torque().index()),
                    iter->second->m_msgContent.apply_joint_torque().effort());
                ++iter;
            } else {
                iter = m_jointCommands.erase(iter);
            }
        }
    }
}

/// clear all queued apply commands
void GazeboApplyCommander::clearApplyCommands() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_jointCommands.clear();
    m_linkSetForceCommand.clear();
    m_linkSetTorqueCommand.clear();
    m_linkAddForceCommand.clear();
    m_linkAddTorqueCommand.clear();
}
} // namespace gazebotransport
} // namespace robotics
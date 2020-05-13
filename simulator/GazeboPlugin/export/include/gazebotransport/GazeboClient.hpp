/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCLIENT_HPP
#define GAZEBOCLIENT_HPP

#include "gazebotransport/gazebotransport_util.hpp"
#include "gazebotransport/robotics.gazebotransport.CoSimMsgs.pb.h"

#include "boost/optional.hpp"

#include <memory>
#include <vector>
#include <tuple>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility> // for std::pair

namespace robotics {
namespace gazebotransport {
/// actual communication implementation of the Gazebo client
class GazeboClientImpl;

/// interface for implementing each Gazebo client action
class Action;

/// Heartbeat that calls client action at fixed interval
class Heartbeat;

/// GazeboClient Interface for Gazebo actions
/**
This class handles all Gazebo related requests
*/
class GAZEBOTRANSPORT_EXPORT_CLASS GazeboClient {
  public:
    /// constructor
    /**
    @param ipaddress       server ip address
    @param port            server port number
    @param timeout         time out for each Gazebo actions
    @param pulseTime       time between each heartbeat pulses
    @exception             May throw if cannot connect to Server

    All actions in this class that interacts with remote Gazebo server
    will wait for the action to finish on server side, with maximum wait
    time specified by timeout

    Set pulseTime to inf if you want server to keep the connection and
    client not publishing heartbeat all the time
    */
    GazeboClient(std::string const& ipaddress, uint16_t port, uint64_t timeout, double pulseTime);

    /// requestCoSim request co-simulation with Gazebo simulator
    /**
    @param clientID         Unique ID that identifies the current co-simulation client
    @return                 true if Co-simulation accepted
    */
    bool requestCoSim(std::string const& clientID);

    /// stopCoSim request stopping co-simulation with Gazebo simulator
    /**
    @param clientID         Unique ID that identifies the current co-simulation client
    @return                 true if co-simulation is stopped
    */
    bool stopCoSim(std::string const& clientID);

    /// step Gazebo server with given number of steps
    /**
    @param numSteps        number of steps to run in remote Gazebo server
    @return                whether the stepping is successful
    */
    bool stepSimulation(uint32_t numSteps);

    /// update simulation time maintained in the GazeboClient
    /**
    @param seconds         Simulation time = (seconds + nanoSeconds * 1e-9) seconds
    @param nanoSeconds     Simulation time = (seconds + nanoSeconds * 1e-9) seconds

    This time is set from Simulink according to Simulink simulation time
    It is used to add time-stamp to the messages that are passed to Gazebo
    */
    void setSimulationTime(uint64_t seconds, uint64_t nanoSeconds);

    /// reset the Gazebo simulation time
    /// @return true if reset action succeeds
    bool resetTime();

    /// reset the Gazebo simulation time, model poses and simulation configuration
    /// @return true if reset succeeds
    bool resetAll();

    /// get ground truth pose for model link in world frame
    /**
    @param modelName                          The model name that contains the link
    @param linkName                           The model link name of which the pose is queried
    @return                                   A pair of isNew flag and a pointer to the pose
    message.

    @note The pointer returned might be nullptr if the pose cannot be obtained
    */
    std::pair<bool, std::shared_ptr<Packet>> getGroundTruthWorldPose(std::string const& modelName,
                                                                     std::string const& linkName);

    /// subscribe topic message from Gazebo server
    /**
    @param topicType       message type of the topic
    @param topicname       topic of the image this client attempts to subscribe
    @return                whether subscription is accepted
    */
    bool subscribeGenericMessage(std::string const& topicType, std::string const& topicname);

    /// get generic message from Gazebo server
    /**
    @param topicType       message type of the topic
    @param topicName       topic name of the requested message
    @return                A pair of isNew flag and a pointer to the message.
    */
    std::pair<bool, std::shared_ptr<Packet>> getGenericMessage(std::string const& topicType,
                                                               std::string const& topicName);

    /// apply joint torque to Gazebo model joint
    /**
    @param modelName                Gazebo model that owns the joint
    @param jointName                Gazebo joint name
    @param indexValue               The axis of the joint to apply torque to
    @param effortValue              The torque amount
    @param durationSeconds          How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    @param durationNanoSeconds      How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    */
    bool applyJointTorque(std::string const& modelName,
                          std::string const& jointName,
                          uint32_t indexValue,
                          double effortValue,
                          uint64_t durationSeconds,
                          uint64_t durationNanoSeconds);

    /// apply link wrench to Gazebo model link
    /**
    @param modelName                Gazebo model that owns the link
    @param linkName                 Gazebo link name
    @param forceType                Indicate whether the force is additive or not
    @param force                    The amount of force to apply at link center of mass
    @param torqueType               Indicate whether the torque is additive or not
    @param torque                   The amount of torque to apply at link center of mass
    @param durationSeconds          How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    @param durationNanoSeconds      How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    */
    bool applyLinkWrench(std::string const& modelName,
                         std::string const& linkName,
                         std::string const& forceType,
                         std::vector<double> const& force,
                         std::string const& torqueType,
                         std::vector<double> const& torque,
                         uint64_t durationSeconds,
                         uint64_t durationNanoSeconds);

    /// get topic list from Gazebo
    std::shared_ptr<Packet> getTopicList();

    /// get model info from Gazebo
    std::shared_ptr<Packet> getModelInfo();

    // get max step size from Gazebo physics solver
    double getMaxStepSize();

    /// shut down the Gazebo client connection
    void shutdown();

  private:
    /// start sending heartbeats
    void startHeartbeat();


    /// enumeration for all actions this client could take
    enum ActionMapID {
        STEP_SIMULATION_ACTION = 0,
        RESET_SIMULATION_ACTION,
        REQUEST_COSIM_ACTION,
        STOP_COSIM_ACTION,
        SUBSCRIBE_GENERIC_MESSAGE_ACTION,
        APPLY_JOINT_TORQUE,
        APPLY_LINK_WRENCH,
        GET_TOPIC_LIST,
        GET_MODEL_INFO,
        GET_MAX_STEP_SIZE
    };

    /// internal implementation of the client
    std::shared_ptr<GazeboClientImpl> m_clientImpl;

    /// client would send out heartbeat during co-simulation at this interval in milliseconds
    double m_heartbeatInterval;

    /// each co-simulation request would be kept at server for this duration in milliseconds
    /// default value is 5 times the m_heartbeatInterval
    double m_heartbeatTimeout;

    /// client heartbeat
    std::shared_ptr<Heartbeat> m_heartbeat;

    /// a map between action ID and client-wide actions
    std::unordered_map<ActionMapID, std::shared_ptr<Action>> m_actions;

    /// simulation time in Simulink with nano-second precision
    struct SimulationTime {
        uint64_t seconds;
        uint64_t nanoSeconds;
    };
    SimulationTime m_simulationTime;

    /// a map between topic names and topic read actions
    std::unordered_map<std::string, std::shared_ptr<Action>> m_subscribers;
};
} // namespace gazebotransport
} // namespace robotics


#endif

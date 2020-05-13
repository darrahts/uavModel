/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/Client.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "robotics.gazebotransport.CoSimMsgs.pb.h"

#include <mutex>

class GazeboWorldStat
{
public:
    GazeboWorldStat()
        : m_mutex(), m_time(), m_worldStat(nullptr)
    {
    }

    void OnWorldStat(ConstWorldStatisticsPtr &msg)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_time = gazebo::msgs::Convert(msg->sim_time());
    }

    gazebo::common::Time getSimTime()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_time;
    }

    void setSubscriber(gazebo::transport::SubscriberPtr &&sub)
    {
        m_worldStat = sub;
    }

private:
    /// mutex
    std::mutex m_mutex;
    /// Simulation time holder
    gazebo::common::Time m_time;
    /// Gazebo Subscriber pointer to Subscribe world stat data
    gazebo::transport::SubscriberPtr m_worldStat;
};

class pluginTest : public testing::Test
{
public:
    std::string ipAddress = "127.0.0.1";
    std::string serverPort = "14581";

    /// Session Time-out value
    int time_out = 50000;

    /// Creating Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;

    /// Ignition transport node and subscriber to get simulation time
    gazebo::transport::NodePtr m_node;

    /// Stores world stat and simulation time from Gazebo
    GazeboWorldStat m_worldStat;

    /**
     * It creates and sends reset gazebo scene & time message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientResetSceneTime()
    {

        /// Create Packet message reset simulation time & scene message
        robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_reset_simulation()->set_behavior(
            robotics::gazebotransport::ResetSimulation_ResetBehavior_RESET_TIME_AND_SCENE);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;

        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends subscribe image message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientSubscribeImage(std::string const &topic_name)
    {
        /// Create Packet message to subscribe Image
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_SUBSCRIBE_IMAGE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_image()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;
        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get/request Image message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientGetImage(std::string const &topic_name)
    {
        /// Create Packet message to get Image
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_IMAGE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_image()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;
        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends subscribe imu message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientSubscribeImu(std::string const &topic_name)
    {
        /// Create Packet message to subscribe imu
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_SUBSCRIBE_IMU);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_imu()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;
        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get/request imu message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientGetImu(std::string const &topic_name)
    {
        /// Create Packet message to get imu
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_IMU);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_imu()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;
        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends subscribe laser message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientSubscribeLaser(std::string const &topic_name)
    {
        /// Create Packet message to subscribe laser
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_SUBSCRIBE_LASER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_laser()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet replyGetLaser;
        if (replyMsg)
        {
            replyGetLaser.ParseFromString(*replyMsg);
        }

        return replyGetLaser;
    }

    /**
     * It creates and sends get/request laser message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientGetLaser(std::string const &topic_name)
    {
        /// Create Packet message to get laser
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_LASER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_laser()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet replyGetLaser;
        if (replyMsg)
        {
            replyGetLaser.ParseFromString(*replyMsg);
        }

        return replyGetLaser;
    }

    /**
     * It creates and sends step simulation message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientStepSimulation(uint32_t stepSize)
    {
        /// Create Packet message step simulation message
        robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID_STEP_SIMULATION);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_step_simulation()->set_num_steps(stepSize);
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;

        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    robotics::gazebotransport::Packet clientApplyJointTorque(std::string const &modelName,
                                                             std::string const &jointName,
                                                             uint32_t indexVal,
                                                             double effortVal,
                                                             uint64_t sec,
                                                             uint64_t nsec)
    {
        /// Create Packet message to apply torque on a joint
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_APPLY_JOINT_TORQUE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_apply_joint_torque()->set_model_name(modelName);
        m_message.mutable_apply_joint_torque()->set_joint_name(jointName);
        m_message.mutable_apply_joint_torque()->set_index(indexVal);
        m_message.mutable_apply_joint_torque()->set_effort(effortVal);
        m_message.mutable_apply_joint_torque()->mutable_duration()->set_seconds(sec);
        m_message.mutable_apply_joint_torque()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;
        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends request co-simulation message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientRequestCoSimulation(std::string const & clientID)
    {
        /// Create Packet message request co-simulation message
        robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID_REQUEST_COSIM);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_cosim()->set_client_id(clientID);
        m_message.mutable_request_cosim()->set_duration(std::numeric_limits<double>::infinity());
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;

        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends stop co-simulation message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientStopCoSimulation(std::string const &clientID)
    {
        /// Create Packet message stop co-simulation message
        robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID_STOP_COSIM);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_stop_cosim()->set_client_id(clientID);
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;

        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }
    /**
     * It creates and sends apply link wrench message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientApplyLinkWrench(std::string const &modelName,
                                                            std::string const &linkName,
                                                            std::string const &forceType,
                                                            std::string const &torqueType,
                                                            double (&forceVal)[3],
                                                            double (&torqueVal)[3],
                                                            uint64_t sec,
                                                            uint64_t nsec)
    {
        /// Create Packet message to apply force/torque on a link
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_APPLY_LINK_WRENCH);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_apply_link_wrench()->set_model_name(modelName);
        m_message.mutable_apply_link_wrench()->set_link_name(linkName);
        m_message.mutable_apply_link_wrench()->set_force_type(forceType);
        m_message.mutable_apply_link_wrench()->set_fx(forceVal[0]);
        m_message.mutable_apply_link_wrench()->set_fy(forceVal[1]);
        m_message.mutable_apply_link_wrench()->set_fz(forceVal[2]);
        m_message.mutable_apply_link_wrench()->set_torque_type(torqueType);
        m_message.mutable_apply_link_wrench()->set_tx(torqueVal[0]);
        m_message.mutable_apply_link_wrench()->set_ty(torqueVal[1]);
        m_message.mutable_apply_link_wrench()->set_tz(torqueVal[2]);
        m_message.mutable_apply_link_wrench()->mutable_duration()->set_seconds(sec);
        m_message.mutable_apply_link_wrench()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;
        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get ground truth world pose message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    robotics::gazebotransport::Packet clientGetPose(std::string const &modelName,
                                                    std::string const &linkName)
    {
        /// Create Packet message to get pose of a link
        robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(robotics::gazebotransport::PacketHeader_MsgID::
                                               PacketHeader_MsgID_GET_GROUND_TRUTH_WORLD_POSE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_get_ground_truth_world_pose()->set_model_name(modelName);
        m_message.mutable_get_ground_truth_world_pose()->set_link_name(linkName);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        robotics::gazebotransport::Packet reply;
        if (replyMsg)
        {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    void SetUp()
    {
        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, serverPort, boost::posix_time::milliseconds(time_out));

        /// Start Gazebo Simulator client
        gazebo::client::setup();

        // setup ignition transport node and subscriber
        m_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        m_node->Init();
        m_worldStat.setSubscriber(
            m_node->Subscribe("~/world_stats", &GazeboWorldStat::OnWorldStat, &m_worldStat));

        robotics::gazebotransport::Packet reply = clientRequestCoSimulation("TestID");
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as the server would grant request
        ASSERT_TRUE(success);

        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }

    void TearDown()
    {
        robotics::gazebotransport::Packet reply = clientStopCoSimulation("TestID");
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        else
        {
            success = false;
        }
        /// it should be true as the server would stop co-simulation
        ASSERT_TRUE(success);

        m_client->shutdown();

        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }
};

/*
 * It tests success of the get pose of a link
 * Also, it tests the client successfully gets pose of ground truth model/link
 */
TEST_F(pluginTest, testModelLinkPose)
{
    robotics::gazebotransport::Packet reply = clientGetPose("unit_box", "link");

    /// The ground truth position and orientation are defined in world/unit_box.world
    /// which is loaded at the start
    ASSERT_NEAR(reply.pose().position().x(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().position().y(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().position().z(), 0.5, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().x(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().y(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().z(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().w(), 1.0, 0.0001);
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully Pose message with all zero values
 */
TEST_F(pluginTest, testInValidModelName)
{
    robotics::gazebotransport::Packet reply = clientGetPose("unit_box0", "link");

    /// The input model is not available in Gazebo
    /// So it returns all zero values for position and orientation
    ASSERT_DOUBLE_EQ(reply.pose().position().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().w(), 0);
}

/*
 * It tests reply message for invalid link name as input.
 * The client should successfully Pose message with all zero values
 */
TEST_F(pluginTest, testInValidLinkName)
{
    robotics::gazebotransport::Packet reply = clientGetPose("unit_box", "link0");

    /// The input link is not available in Gazebo
    /// So it returns all zero values for position and orientation
    ASSERT_DOUBLE_EQ(reply.pose().position().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().w(), 0);
}

/*
 * It tests success of the get all Model information from Gazebo
 * Also, it tests the client successfully receives Model/link/joint names
 * by comparing with default as well as world file Model/link/joint names
 */
TEST_F(pluginTest, testModelNames)
{
    /// Create Packet message to get Model info from Gazebo
    robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_GET_MODEL_INFO);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_model_info()->set_topic_name("~/GetModelInfo");

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    /// Convert reply message to packet message
    robotics::gazebotransport::Packet reply;
    if (replyMsg)
    {
        reply.ParseFromString(*replyMsg);
    }

    /// The received model info is compared with default gazebo model like "ground_plane" & link
    /// "link" Also, the ground truth model, link and joint names are given with .world file as
    /// "unit_box", "link" & "joint"
    ASSERT_STREQ(reply.model_info().model_data(0).model_name().c_str(), "ground_plane");
    ASSERT_STREQ(reply.model_info().model_data(0).links().link_name(0).c_str(), "link");
    ASSERT_STREQ(reply.model_info().model_data(1).model_name().c_str(), "unit_box");
    ASSERT_STREQ(reply.model_info().model_data(1).links().link_name(0).c_str(), "link");
    ASSERT_STREQ(reply.model_info().model_data(1).joints().joint_name(0).c_str(), "joint");
}

/// Test gazebo simulation reset time and scene action
TEST_F(pluginTest, testResetTimeScene)
{
    /// Create Packet message reset simulation time & scene message
    robotics::gazebotransport::Packet m_message;

    m_message.mutable_header()->set_id(
        robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_reset_simulation()->set_behavior(
        robotics::gazebotransport::ResetSimulation_ResetBehavior_RESET_TIME_AND_SCENE);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));
    bool success;

    /// Convert reply message to packet message
    robotics::gazebotransport::Packet reply;

    if (replyMsg)
    {
        reply.ParseFromString(*replyMsg);
        success = reply.has_status() && !reply.status();
    }
    else
    {
        success = false;
    }

    ASSERT_TRUE(success);
}

/// Test gazebo simulation reset time action
TEST_F(pluginTest, testResetTime)
{
    /// Create Packet message reset simulation time message
    robotics::gazebotransport::Packet m_message;

    m_message.mutable_header()->set_id(
        robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_reset_simulation()->set_behavior(
        robotics::gazebotransport::ResetSimulation_ResetBehavior_RESET_TIME);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));
    bool success;

    /// Convert reply message to packet message
    robotics::gazebotransport::Packet reply;

    if (replyMsg)
    {
        reply.ParseFromString(*replyMsg);
        success = reply.has_status() && !reply.status();
    }
    else
    {
        success = false;
    }

    ASSERT_TRUE(success);
}

/// Test gazebo image subscribe and get for image sensor 0
TEST_F(pluginTest, imageSensor0SubscribeGetTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe image sensor 0
    reply = clientSubscribeImage("/gazebo/default/camera0/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get image sensor based on image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {
        // Request image
        reply = clientGetImage("/gazebo/default/camera0/link/camera/image");

        success = false;

        if (reply.has_image())
        {
            success = true;
        }

        // check image subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get image at each step number else no image
        if (subscribeStart)
        {
            /// It should be true as topic & image is available in gazebo
            ASSERT_TRUE(success);
            /// Verify the received image data with ground truth values.
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "RGB_INT8");
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo image subscribe and get for image sensor 1
TEST_F(pluginTest, imageSensor1SubscribeGetTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe image sensor 1
    reply = clientSubscribeImage("/gazebo/default/camera1/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get image sensor based on image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {
        // Request image
        reply = clientGetImage("/gazebo/default/camera1/link/camera/image");

        success = false;

        if (reply.has_image())
        {
            success = true;
        }

        // check image subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get image at each step number else no image
        if (subscribeStart)
        {
            /// It should be true as topic & image is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received image data with ground truth values.
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "RGB_INT8");
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo depth image subscribe and get for depth image sensor 0
TEST_F(pluginTest, depthImageSensor0SubscribeGetTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe depth image sensor 0
    reply = clientSubscribeImage("/gazebo/default/depth_camera0/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get depth image sensor based on depth image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {
        // Request depth image
        reply = clientGetImage("/gazebo/default/depth_camera0/link/camera/image");

        success = false;

        if (reply.has_image())
        {
            success = true;
        }

        // check image subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get depth image at each step number else no depth image
        if (subscribeStart)
        {
            /// It should be true as topic & depth image is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received depth image data with ground truth values.
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "R_FLOAT32");
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo depth image subscribe and get for depth image sensor 1
TEST_F(pluginTest, depthImageSensor1SubscribeGetTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe depth image sensor 1
    reply = clientSubscribeImage("/gazebo/default/depth_camera1/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get depth image sensor based on depth image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {
        // Request depth image
        reply = clientGetImage("/gazebo/default/depth_camera1/link/camera/image");

        success = false;

        if (reply.has_image())
        {
            success = true;
        }

        // check image subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get depth image at each step number else no depth image
        if (subscribeStart)
        {
            /// It should be true as topic & depth image is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received depth image data with ground truth values.
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "R_FLOAT32");
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo laser subscribe and get laser data for laser sensor 0
TEST_F(pluginTest, laserSensor0SubscribeTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    reply = clientSubscribeLaser("/gazebo/default/hokuyo0/link/laser/scan");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {

        reply = clientGetLaser("/gazebo/default/hokuyo0/link/laser/scan");

        success = false;

        if (reply.has_laser_data())
        {
            success = true;
        }

        // check laser subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get laser data at each step number else not
        if (subscribeStart)
        {
            /// it should be true as topic & laser data is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received laser data with ground truth values.
            ASSERT_EQ(reply.laser_data().angle_min(), -3.14);
            ASSERT_EQ(reply.laser_data().angle_max(), 3.14);
            ASSERT_EQ(reply.laser_data().count(), 640);
            ASSERT_EQ(reply.laser_data().range_min(), 0.08);
            ASSERT_EQ(reply.laser_data().range_max(), 10);
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo laser subscribe and get laser data for laser sensor 1
TEST_F(pluginTest, laserSensor1SubscribeTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    reply = clientSubscribeLaser("/gazebo/default/hokuyo1/link/laser/scan");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {

        reply = clientGetLaser("/gazebo/default/hokuyo1/link/laser/scan");

        success = false;

        if (reply.has_laser_data())
        {
            success = true;
        }

        // check laser subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get laser data at each step number else not
        if (subscribeStart)
        {
            /// it should be true as topic & laser data is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received laser data with ground truth values.
            ASSERT_EQ(reply.laser_data().angle_min(), -3.14);
            ASSERT_EQ(reply.laser_data().angle_max(), 3.14);
            ASSERT_EQ(reply.laser_data().count(), 640);
            ASSERT_EQ(reply.laser_data().range_min(), 0.08);
            ASSERT_EQ(reply.laser_data().range_max(), 10);
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo subscribe imu sensor and get imu data for imu sensor 0
TEST_F(pluginTest, imuSensor0SubscribeGetTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe imu sensor 0
    reply = clientSubscribeImu("/gazebo/default/imu0/link/imu/imu");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {

        reply = clientGetImu("/gazebo/default/imu0/link/imu/imu");

        success = false;

        if (reply.has_imu_data())
        {
            success = true;
        }

        // check imu subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get imu data at each step number else not
        if (subscribeStart)
        {
            /// it should be true as topic & imu data is available in gazebo
            ASSERT_TRUE(success);
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo subscribe imu sensor and get imu data for imu sensor 0
TEST_F(pluginTest, imuSensor1SubscribeGetTest)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe imu sensor 1
    reply = clientSubscribeImu("/gazebo/default/imu1/link/imu/imu");

    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++)
    {

        reply = clientGetImu("/gazebo/default/imu1/link/imu/imu");

        success = false;

        if (reply.has_imu_data())
        {
            success = true;
        }

        // check imu subscriber started
        if (success)
        {
            subscribeStart = true;
        }
        else
        {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get imu data at each step number else not
        if (subscribeStart)
        {
            /// it should be true as topic & imu data is available in gazebo
            ASSERT_TRUE(success);
        }
        else
        {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

TEST_F(pluginTest, imageSensorSubscribeTwice)
{
    // Reset gazebo scene and time
    robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    std::vector<robotics::gazebotransport::Time> imageTime;
    // subscribe image sensor 0 twice and make sure that the client still can get new images
    reply = clientSubscribeImage("/gazebo/default/camera0/link/camera/image");
    reply = clientSubscribeImage("/gazebo/default/camera0/link/camera/image");
    success = false;

    if (reply.has_status() && !reply.status())
    {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    // Collect two images from Gazebo
    size_t loopCount = 0;
    while (imageTime.size() < 2 && loopCount++ < 50)
    {
        // Request image
        reply = clientGetImage("/gazebo/default/camera0/link/camera/image");

        success = reply.has_image();

        // if image is obtained, log the image time_stamp
        if (success)
        {
            /// It should be true as topic & image is available in gazebo
            imageTime.push_back(reply.header().time_stamp());
        }

        // step gazebo by 100
        robotics::gazebotransport::Packet reply = clientStepSimulation(100);
        success = false;
        if (reply.has_status() && !reply.status())
        {
            success = true;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }
        ASSERT_TRUE(success);
    }

    // verify that two images are not recorded at the same time
    ASSERT_EQ(imageTime.size(), 2);
    EXPECT_FALSE(imageTime[0].seconds() == imageTime[1].seconds() && imageTime[0].nano_seconds() == imageTime[1].nano_seconds()) << "Two image should not be obtained at the same time";
}

/// Test gazebo get topic lists
TEST_F(pluginTest, testGetTopicList)
{
    robotics::gazebotransport::Packet m_message;
    robotics::gazebotransport::Packet reply;

    m_message.mutable_header()->set_id(
        robotics::gazebotransport::PacketHeader_MsgID_GET_TOPIC_LIST);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_topic_list()->set_topic_name("~/GetTopicList");

    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    if (replyMsg)
    {
        reply.ParseFromString(*replyMsg);
    }

    /// Check with default topics available in Gazebo
    ASSERT_STREQ("/gazebo/default/physics/contacts", reply.topic_list().data(0).name().c_str());
}

/*
 * It tests, if the image sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(pluginTest, testImageInValidTopic)
{
    //*********************************************
    /// TEST Wrong Image Subscriber
    robotics::gazebotransport::Packet reply0 =
        clientSubscribeImage("/gazebo/default/camera/link/camera/image");

    bool success0 = false;

    if (reply0.has_status() && !reply0.status())
    {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(reply0.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the image sensor( topic) is not available in Gazebo
 * & user trying to get image data from that topic.
 * Then, all image sensor message fields should zero.
 */
TEST_F(pluginTest, testGetInvalidImage)
{
    //*********************************************
    /// TEST Get Invalid Image

    robotics::gazebotransport::Packet replyGetWrongImage =
        clientGetImage("/gazebo/default/camera/link/camera/image");

    bool success0 = false;

    if (replyGetWrongImage.has_status() && !replyGetWrongImage.status())
    {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(replyGetWrongImage.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the imu sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(pluginTest, testImuInValidTopic)
{
    //*********************************************
    /// TEST Wrong Imu Subscriber

    robotics::gazebotransport::Packet reply0 =
        clientSubscribeImu("/gazebo/default/imu/link/imu/imu");

    bool success0 = false;

    if (reply0.has_status() && !reply0.status())
    {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(reply0.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the imu sensor( topic) is not available in Gazebo
 * & user trying to get imu data from that topic.
 * Then, all imu sensor message fields should zero.
 */
TEST_F(pluginTest, testGetInValidImu)
{
    //*********************************************
    /// TEST Get Invalid Imu

    robotics::gazebotransport::Packet replyGetWrongImu =
        clientGetImu("/gazebo/default/imu/link/imu/imu");

    bool success0 = false;

    if (replyGetWrongImu.has_status() && !replyGetWrongImu.status())
    {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(replyGetWrongImu.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the laser sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(pluginTest, testLaserInValidTopic)
{
    //*********************************************
    /// TEST Wrong Laser Subscriber

    robotics::gazebotransport::Packet reply0 =
        clientSubscribeLaser("/gazebo/default/hokuyo/link/laser/scan");

    bool success0 = false;

    if (reply0.has_status() && !reply0.status())
    {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(reply0.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the laser sensor( topic) is not available in Gazebo
 * & user trying to get laser data from that topic.
 * Then, all laser sensor message fields should zero.
 */
TEST_F(pluginTest, testGetInValidLaser)
{
    //*********************************************
    /// TEST Get Invalid Laser

    robotics::gazebotransport::Packet replyGetWrongLaser =
        clientGetLaser("/gazebo/default/hokuyo/link/laser/scan");

    bool success0 = false;

    if (replyGetWrongLaser.has_status() && !replyGetWrongLaser.status())
    {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(replyGetWrongLaser.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests success of the apply joint torque
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testApplyJointSuccess)
{
    robotics::gazebotransport::Packet reply =
        clientApplyJointTorque("unit_box", "joint", 0, 100000.0, 0, 10000000);

    ASSERT_EQ(reply.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE); // 0: NO Error
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyJointInValidModelName)
{
    robotics::gazebotransport::Packet reply =
        clientApplyJointTorque("unit_box0", "joint", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(), robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID error
}

/*
 * It tests reply message for invalid joint name as input.
 * The client should successfully receive the JOINT_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyJointInValidJointName)
{
    robotics::gazebotransport::Packet reply =
        clientApplyJointTorque("unit_box", "joint0", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(), robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_JOINT_NAME_INVALID); // JOINT_NAME_INVALID error
}

/*
 * It tests success of the apply link force/torque
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testApplyLinkWrenchSuccess)
{
    double force_values[3] = {15000.0, 0.0, 0.0};
    double torque_values[3] = {15000.0, 0.0, 0.0};

    robotics::gazebotransport::Packet reply = clientApplyLinkWrench(
        "unit_box", "link", "ADD", "ADD", force_values, torque_values, 0, 100000000);

    ASSERT_EQ(reply.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE); // NO Error
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidModelName)
{
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unt_box", "link", "ADD", "ADD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(), robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID error
}

/*
 * It tests reply message for invalid link name as input.
 * The client should successfully receive the LINK_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidLinkName)
{
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "lik", "ADD", "ADD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(), robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_LINK_NAME_INVALID); // LINK_NAME_INVALID error
}

/*
 * It tests reply message for invalid force type name as input.
 * The client should successfully receive the FORCE_TYPE_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidForceType)
{
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "link", "AD", "ADD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(), robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_FORCE_TYPE_INVALID); // FORCE_TYPE_INVALID error
}

/*
 * It tests reply message for invalid torque type name as input.
 * The client should successfully receive the TORQUE_TYPE_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidTorqueType)
{
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "link", "ADD", "AD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(),
              robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TORQUE_TYPE_INVALID); // TORQUE_TYPE_INVALID error
}
/*
 * It tests success of the get max step size value from Gazebo
 * Also, it tests the client successfully gets the default step size (0.001)
 */
TEST_F(pluginTest, testGetStepSize)
{
    /// Create Packet message to get max step size from Gazebo

    robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_MAX_STEP_SIZE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_max_step_size()->set_type(
        robotics::gazebotransport::MaxStepSize_TYPE::MaxStepSize_TYPE_GET_STEP_SIZE);
    m_message.mutable_max_step_size()->set_size(0);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    robotics::gazebotransport::Packet reply;
    if (replyMsg)
    {
        reply.ParseFromString(*replyMsg);
    }

    ASSERT_EQ(reply.max_step_size().size(), 0.001); // default value 0.001
}

/*
 * It tests success of the set max step size value to Gazebo
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testSetStepSize)
{
    /// Create Packet message to set max step size of Gazebo
    robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_MAX_STEP_SIZE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_max_step_size()->set_type(
        robotics::gazebotransport::MaxStepSize_TYPE::MaxStepSize_TYPE_SET_STEP_SIZE);
    m_message.mutable_max_step_size()->set_size(0.01);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    robotics::gazebotransport::Packet reply;
    if (replyMsg)
    {
        reply.ParseFromString(*replyMsg);
    }

    ASSERT_EQ(reply.status(),
              robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE); // 0: NO Error
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

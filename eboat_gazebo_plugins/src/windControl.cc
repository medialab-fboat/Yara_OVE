// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#include <algorithm>
#include <string>

#include "ignition/math/Pose3.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "ros/ros.h"
#include "../include/windControl.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Point.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AtmosfericControlPlugin)

//////////////////////////////////////////////////////////////////////
AtmosfericControlPlugin::AtmosfericControlPlugin()
{
}

//////////////////////////////////////////////////////////////////////
void AtmosfericControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "AtmosfericControlPlugin _model pointer is NULL");
    
    this->world = _model->GetWorld();
    GZ_ASSERT(this->world, "AtmosfericControlPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "AtmosfericControlPlugin physics pointer is NULL");

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions wSub =
        ros::SubscribeOptions::create<geometry_msgs::Point>(
            "/eboat/atmosferic_control/wind",
            1,
            boost::bind(&AtmosfericControlPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->windSub = this->rosNode->subscribe(wSub);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&AtmosfericControlPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void AtmosfericControlPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&AtmosfericControlPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////////////////////////
void AtmosfericControlPlugin::OnUpdate()
{}
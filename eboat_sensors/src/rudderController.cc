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
#include "../include/rudderController.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RudderControllerPlugin)

//////////////////////////////////////////////////////////////////////
RudderControllerPlugin::RudderControllerPlugin() : rudderPosition(0.0), rudderEngVel(0.4)
{
}

//////////////////////////////////////////////////////////////////////
void RudderControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "RudderControllerPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "RudderControllerPlugin _sdf pointer is NULL");
    this->model     = _model;
    this->sdf       = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "RudderControllerPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "RudderControllerPlugin physics pointer is NULL");

    if (_sdf->HasElement("rudder_joint_name"))
    {
        this->rudderJoint = this->model->GetJoint(_sdf->Get<std::string>("rudder_joint_name"));
    }
    else
        this->rudderJoint = this->model->GetJoint("rudder_joint");

    if (_sdf->HasElement("rudder_motor_speedy"))
        this->rudderEngVel = _sdf->Get<double>("rudder_motor_speedy");
    
    //--> LINKS
    this->rudderLink        = this->rudderJoint->GetChild();

    //--> CONSTANTS
    this->d2r = M_PI / 180.0;

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
    ros::SubscribeOptions rudderPosSub =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/eboat/control_interface/rudder",
            1,
            boost::bind(&RudderControllerPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(rudderPosSub);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&RudderControllerPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void RudderControllerPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&RudderControllerPlugin::OnUpdate, this));
    
    this->rudderPosition = 0.0;
}

//////////////////////////////////////////////////////////////////////
void RudderControllerPlugin::OnUpdate()
{
    ///////////////////////////////////////////////////////////////////////////////
    // Control interface for the rudder
    this->rudderEngVel = 0.4;          //-->this velocity is already in millisecond scale.
    if ((this->rudderJoint->Position() < this->rudderPosition + 1.0e-4) &&
        (this->rudderJoint->Position() > this->rudderPosition - 1.0e-4))
    {
        this->rudderJoint->SetVelocity(0, 0.0);
    }
    else if (this->rudderPosition != this->rudderJoint->Position())
    {
        if (this->rudderPosition < this->rudderJoint->Position())
        {
            this->rudderJoint->SetVelocity(0, -this->rudderEngVel);
        }
        else
        {
            this->rudderJoint->SetVelocity(0, this->rudderEngVel);
        }
    }
}
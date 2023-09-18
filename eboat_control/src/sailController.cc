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
#include "../include/sailController.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SailControllerPlugin)

//////////////////////////////////////////////////////////////////////
SailControllerPlugin::SailControllerPlugin() : sailPosition(0.0), boomEngVel(0.0005)
{
}

//////////////////////////////////////////////////////////////////////
void SailControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "SailControllerPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "SailControllerPlugin _sdf pointer is NULL");
    this->model     = _model;
    this->sdf       = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "SailControllerPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "SailControllerPlugin physics pointer is NULL");

    if (_sdf->HasElement("sail_joint_name"))
    {
        this->sailJoint = this->model->GetJoint(sdf->Get<std::string>("sail_joint_name"));
    }
    else
        this->sailJoint = this->model->GetJoint("boom_joint");

    if (_sdf->HasElement("boom_motor_speedy"))
        this->boomEngVel = _sdf->Get<double>("boom_motor_speedy");
    else
        this->boomEngVel = 0.5;
    
    //--> LINKS
    this->sailLink = this->sailJoint->GetChild();

    //--> CONSTANTS
    this->d2r = M_PI / 180.0;

    std::string rostopic_name = "/";
    rostopic_name.append(this->model->GetName());
    rostopic_name.append("/control_interface/sail");

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
    ros::SubscribeOptions boomAngleSub =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            rostopic_name,
            1,
            boost::bind(&SailControllerPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(boomAngleSub);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&SailControllerPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void SailControllerPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&SailControllerPlugin::OnUpdate, this));
    
    this->sailJoint->SetUpperLimit(0, 0.0); //(1.0*d2r));
    this->sailJoint->SetLowerLimit(0, 0.0); //-(1.0*d2r));
    this->sailPosition = this->d2r;
}

//////////////////////////////////////////////////////////////////////
void SailControllerPlugin::OnUpdate()
{
    ///////////////////////////////////////////////////////////////////////////////
    // Controller interface for emulate the real behavior of the boom.
    // In real world the boom is moved by a cable connected to an electric engine.
    // The sailor could not direct control the position of the sail/boom.
    // In order to change the sail position, the sailor can release cable, so the
    // wind can push the sail, or he can pull the cable to move the sail against
    // the wind force.
    double sov = this->boomEngVel * this->world->Physics()->GetMaxStepSize();
    if (this->sailPosition > 90.0)
         this->sailPosition = 90.0;
    else if (this->sailPosition < -90.0)
         this->sailPosition = -90.0;

    if (this->sailPosition > this->sailJoint->UpperLimit(0) + sov) //--> in this condition the cable should be released
    {
        this->sailJoint->SetUpperLimit(0, this->sailJoint->UpperLimit(0) + sov);
        this->sailJoint->SetLowerLimit(0, this->sailJoint->LowerLimit(0) - sov);
    }
    else if (this->sailPosition < this->sailJoint->UpperLimit(0) - sov) //--> in this condition the cable should be pulled
    {
        this->sailJoint->SetUpperLimit(0, this->sailJoint->UpperLimit(0) - sov);
        this->sailJoint->SetLowerLimit(0, this->sailJoint->LowerLimit(0) + sov);
    }

    if (this->sailJoint->UpperLimit(0) > 90.0)
    {
        this->sailJoint->SetUpperLimit(0, 90.0);
        this->sailJoint->SetLowerLimit(0, -90.0);
    }

    ///////////////////////////////////////////////////////////
    //physics::JointPtr jibJoint = this->model->GetJoint("boom_jib_joint");
    //jibJoint->SetUpperLimit(0, this->sailJoint->UpperLimit(0));
    //jibJoint->SetLowerLimit(0, this->sailJoint->LowerLimit(0));

    ///////////////////////////////////////
    //std::cout << "Command  : " << this->sailPosition << std::endl;
    //std::cout << "Execution: " << this->sailJoint->UpperLimit(0) << "|" << this->sailJoint->LowerLimit(0) << std::endl;
    ///////////////////////////////////////
}
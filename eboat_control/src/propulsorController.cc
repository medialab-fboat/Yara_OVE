// Auhtor : araujo Charles Vasconcellos
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
#include "../include/propulsorController.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int16.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PropulsorControllerPlugin)

//////////////////////////////////////////////////////////////////////
PropulsorControllerPlugin::PropulsorControllerPlugin() : turbineVel(0.0)
{
}

//////////////////////////////////////////////////////////////////////
void PropulsorControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "PropulsorControllerPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "PropulsorControllerPlugin _sdf pointer is NULL");
    this->model     = _model;
    this->sdf       = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "PropulsorControllerPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "PropulsorControllerPlugin physics pointer is NULL");

    if (_sdf->HasElement("propulsor_joint_name"))
    {
        this->propulsorJoint = this->model->GetJoint(_sdf->Get<std::string>("propulsor_joint_name"));
    }
    else
        this->propulsorJoint = this->model->GetJoint("turbine_joint");

    //--> LINKS
    this->eletricEngineLink = this->propulsorJoint->GetParent();

    std::string rostopic_name = "/";
    rostopic_name.append(this->model->GetName());
    rostopic_name.append("/control_interface/propulsion");

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
    ros::SubscribeOptions propspeed =
        ros::SubscribeOptions::create<std_msgs::Int16>(
            rostopic_name,
            1,
            boost::bind(&PropulsorControllerPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(propspeed);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&PropulsorControllerPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void PropulsorControllerPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&PropulsorControllerPlugin::OnUpdate, this));
    
    this->turbineVel     = 0;
    this->propulsorJoint->SetVelocity(0, 0.0);
}

//////////////////////////////////////////////////////////////////////
void PropulsorControllerPlugin::OnUpdate()
{
    /*float start = 2.0;
    if (this->world->SimTime().Float() == start)
    {
        this->turbineVel = 5;
    }
    else if (this->world->SimTime().Float() == start+6)
    {
        ignition::math::Quaternion rot = this->model->WorldPose().Rot();
        std::cout<<"==================================="<<std::endl;
        std::cout<<rot.Yaw()*(180.0/M_PI)<<std::endl;
        this->world->SetPaused(true);
    }*/
    ///////////////////////////////////////////////////////////////////////////////
    // Eletric propulsion control interface and model
    // Force applied as function of turbine angular speed
    // Our max force are 24.49 kgf
    if (this->turbineVel > 5)
        this->turbineVel = 5;
    else if (this->turbineVel < -5)
        this->turbineVel = -5;
    double turbineAngVel = this->turbineVel * (this->propulsorJoint->GetVelocityLimit(0)/5.0);
    if (turbineAngVel != this->propulsorJoint->GetVelocity(0))
    {
        this->propulsorJoint->SetVelocity(0, turbineAngVel);
        //double force = (this->turbineVel / 5.0) * 24.49 * 9.80665 * (0.5); //--> force in newtons
        //double force = (this->turbineVel / 5.0) * 24.49;       //--> force in kgf
        double force = (this->turbineVel / 5.0) * 54.0 * 4.448 * (0.5); //--> force in newtons
        ignition::math::Vector3d thrust;
        //-->THE CODE BELOW TRY TO SIMULATE THAT, DUE TO THE BOAT HULL DESIGN, THE BACKARD PROPULSION IS LESS EFFECTIVE THAN THE FORWARD PROPULSION
        //if (this->turbineVel < 0)
        //    thrust = ignition::math::Vector3d((0.4*force),0,0);
        //else
        //    thrust = ignition::math::Vector3d(force,0,0);
        thrust = ignition::math::Vector3d(force,0,0);
        this->eletricEngineLink->AddLinkForce(thrust, ignition::math::Vector3d(0,0,0));
    }
}
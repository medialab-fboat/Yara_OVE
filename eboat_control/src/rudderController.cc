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
RudderControllerPlugin::RudderControllerPlugin() : rudderPosition(0.0), rudderEngVelMax(0.4)
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
        this->rudderEngVelMax = _sdf->Get<double>("rudder_motor_speedy");
    
    //--> LINKS
    this->rudderLink        = this->rudderJoint->GetChild();

    //--> CONSTANTS
    this->d2r = M_PI / 180.0;

    std::string ros_topic_name = "/";
    ros_topic_name.append(this->model->GetName());
    ros_topic_name.append("/control_interface/rudder");

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
            ros_topic_name,
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
    this->rudderJoint->SetParam("vel", 0, 0.0);
}

//////////////////////////////////////////////////////////////////////
void RudderControllerPlugin::OnUpdate()
{
    //-->CONTROL INTERFACE
    double error    = this->rudderJoint->Position() - this->rudderPosition;
    this->rudderJoint->SetParam("fmax", 0, 100.0);
    double abserror = abs(error);
    if (abserror < 1.0e-4)
    {
        this->rudderJoint->SetParam("vel", 0, 0.0);
        this->rudderEngVel = 0.0; //this->rudderEngVelMax;
    }
    else
    {
        this->rudderEngVel = (abserror > this->rudderEngVelMax * this->world->Physics()->GetMaxStepSize()) ? this->rudderEngVelMax
                                                                                                           : abserror;
        
        if (error < 0.0)
            this->rudderJoint->SetParam("vel", 0, this->rudderEngVel);
        else if (error > 0.0)
            this->rudderJoint->SetParam("vel", 0, -this->rudderEngVel);
    }

    /*float simtime = this->world->SimTime().Float();
    if (simtime - floor(simtime) == 0.0)
    {
        std::cout << "-----------------------------" << std::endl;
        std::cout << "simtime         : " << simtime << std::endl;
        std::cout << "error           : " << error << " (" << abserror << ")" << std::endl;
        std::cout << "vel             : " << this->rudderEngVel << " (" << this->rudderEngVelMax << ")" << std::endl;
        std::cout << "Desired Position: " << this->rudderPosition*(180.0 / M_PI) << " [" << this->rudderPosition - 1.0e-4 << " , " << this->rudderPosition + 1.0e-4 << "]" << std::endl;
        std::cout << "Actual Position : " << this->rudderJoint->Position(0)*(180.0 / M_PI) << " (" << this->rudderJoint->Position(0)  << ")" << std::endl;
        std::cout << "GetVelocity     : " << this->rudderJoint->GetVelocity(0) << std::endl;
        std::cout << "GetForce        : " << this->rudderJoint->GetForce(0) << std::endl;
        std::cout << "boat lin vel    : " << this->rudderJoint->GetChild()->WorldLinearVel().Length() << std::endl;
        std::cout << "step time       : " << this->world->Physics()->GetMaxStepSize() << std::endl;
        //if (abs(error) < 1.0e-4)
        //    this->world->SetPaused(true);
        //this->world->SetPaused(true);
    }*/
}
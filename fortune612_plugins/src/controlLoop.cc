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
#include "../include/controlLoop.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ControlLoopPlugin)

//////////////////////////////////////////////////////////////////////
ControlLoopPlugin::ControlLoopPlugin()
{
}

//////////////////////////////////////////////////////////////////////
void ControlLoopPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "ControlLoopPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "ControlLoopPlugin _sdf pointer is NULL");
    this->model     = _model;
    this->sdf       = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "ControlLoopPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "ControlLoopPlugin physics pointer is NULL");

    // --> THE FREQUENCY, IN HERTZ, IN WHICH THE CONTROL LOOP WILL INTERACT WITH THE VIRTUAL ENVIRONMENT (COLLECT DATA AND SEND COMMANDS) 
    if (_sdf->HasElement("frequency"))
        this->freq = _sdf->Get<float>("frequency");
    else
        this->freq = 1.0;

    // --> GET THE NAME TO IDENTIFY THE MAIN SAIL'S BOOM JOINT
    if (_sdf->HasElement("boom_main_joint_name"))
    {
        this->mainSailJoint = this->model->GetJoint(sdf->Get<std::string>("boom_main_joint_name"));
    }
    else
        this->mainSailJoint = this->model->GetJoint("boom_main_joint");

    // -- > GET THE NAME TO IDENTIFY THE JIB SAIL'S BOOM JOINT
    if (_sdf->HasElement("boom_jib_joint_name"))
    {
        this->jibSailJoint = this->model->GetJoint(sdf->Get<std::string>("boom_jib_joint_name"));
    }
    else
        this->jibSailJoint = this->model->GetJoint("boom_jib_joint");

    // --> SET THE ANGULAR VELOCITY TO OPEN THE BOOM (IN RAD PER SECOND)
    if (_sdf->HasElement("boom_motor_speedy"))
        this->boomEngVel = _sdf->Get<double>("boom_motor_speedy");
    else
        this->boomEngVel = 0.5;

    // --> GET THE NAME TO IDENTIFY THE RUDDER JOINT
    if (_sdf->HasElement("boom_main_joint_name"))
    {
        this->rudderJoint = this->model->GetJoint(sdf->Get<std::string>("boom_main_joint_name"));
    }
    else
        this->rudderJoint = this->model->GetJoint("boom_main_joint");

    //--> CONSTANTS
    this->d2r = M_PI / 180.0;
}

/////////////////////////////////////////////////
void ControlLoopPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ControlLoopPlugin::OnUpdate, this));
    this->reftime = 0.0;
    this->mainSailJoint->SetUpperLimit(0, 0);
    this->mainSailJoint->SetLowerLimit(0, 0);
    this->jibSailJoint->SetUpperLimit(0, 0);
    this->jibSailJoint->SetLowerLimit(0, 0);
}

//////////////////////////////////////////////////////////////////////
void ControlLoopPlugin::OnUpdate()
{
    float simtime = this->world->SimTime().Float();

    // SAFETY PROCEDURE
    if (simtime < this->reftime)
        this->reftime = simtime;

    // CONTROL LOOP BLOCK
    if ((simtime - this->reftime < 1.05*(1 / this->freq)) || (simtime - this->reftime > 0.95*(1 / this->freq)))
    {
        // --> GET THE MODEL POSE: ABSOLUTE POSITION IN THE WORLD FRAME AND ROLL, PITCH AND YALL ANGLES
        this->modelPosition = this->model->WorldPose().Pos();
        ignition::math::Quaternion modelRotation = this->model->WorldPose().Rot();

        std::cout << "-------------------------------------" << std::endl;
        std::cout << "Model " << this->model->GetName() << std::endl;
        std::cout << "Position           : " << this->modelPosition << std::endl;
        std::cout << "(roll, pitch, yaw) : " << "( " << modelRotation.Roll();
        std::cout << ", " << modelRotation.Pitch();
        std::cout << ", " << modelRotation.Yaw() << std::endl;

        ///////////////////////////////////////////////////////////////////////////////
        // --> THE ACTUAL CONTROL LOOP SHOULD BE IN THIS BLOCK CONSUMING OBSERVATIONS
        //     AND SENDING COMMANDS TO THE BOAT

        if (abs(modelRotation.Roll()) > 0.1)
            this->sailPosition = 25.0 * this->d2r;
        else
            this->sailPosition = 0.0;
        std::cout << "sail Position = " << this->sailPosition << std::endl;
        std::cout << "sail UpperL   = " << this->mainSailJoint->UpperLimit(0) + this->boomEngVel * this->world->Physics()->GetMaxStepSize() << std::endl;
        //this->sailPosition = 0.0;
        this->rudderPosition = 0.0;
        ///////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////////
        // --> BOOM ACTUATOR EMULATES THE OPERATION OF THE BOOM CABLE
        //
        // Controller interface for emulate the real behavior of the boom.
        // In real world the boom is moved by a cable connected to an electric engine.
        // The sailor could not direct control the position of the sail/boom.
        // In order to change the sail position, the sailor can release cable, so the
        // wind can push the sail, or he can pull the cable to move the sail against
        // the wind force.
        double sov = this->boomEngVel * this->world->Physics()->GetMaxStepSize();
        if (this->sailPosition > 80.0)
            this->sailPosition == 80.0;
        else if (this->sailPosition < -80.0)
            this->sailPosition == -80.0;

        if (this->sailPosition > this->mainSailJoint->UpperLimit(0) + sov) //--> in this condition the cable should be released
        {
            std::cout << "aqui esta funcionando" << std::endl;
            this->mainSailJoint->SetUpperLimit(0, this->mainSailJoint->UpperLimit(0) + sov);
            this->mainSailJoint->SetLowerLimit(0, this->mainSailJoint->LowerLimit(0) - sov);
        }
        else if (this->sailPosition < this->mainSailJoint->UpperLimit(0) - sov) //--> in this condition the cable should be pulled
        {
            this->mainSailJoint->SetUpperLimit(0, this->mainSailJoint->UpperLimit(0) - sov);
            this->mainSailJoint->SetLowerLimit(0, this->mainSailJoint->LowerLimit(0) + sov);
        }

        if (this->sailPosition > this->jibSailJoint->UpperLimit(0) + sov) //--> in this condition the cable should be released
        {
            this->jibSailJoint->SetUpperLimit(0, this->jibSailJoint->UpperLimit(0) + sov);
            this->jibSailJoint->SetLowerLimit(0, this->jibSailJoint->LowerLimit(0) - sov);
        }
        else if (this->sailPosition < this->jibSailJoint->UpperLimit(0) - sov) //--> in this condition the cable should be pulled
        {
            this->jibSailJoint->SetUpperLimit(0, this->jibSailJoint->UpperLimit(0) - sov);
            this->jibSailJoint->SetLowerLimit(0, this->jibSailJoint->LowerLimit(0) + sov);
        }

        // --> SAFETY PROCEDURE
        if ((this->mainSailJoint->UpperLimit(0) > 80.0) || (this->jibSailJoint->UpperLimit(0) > 80.0))
        {
            this->mainSailJoint->SetUpperLimit(0, 80.0);
            this->mainSailJoint->SetLowerLimit(0, -80.0);
            this->jibSailJoint->SetUpperLimit(0, 80.0);
            this->jibSailJoint->SetLowerLimit(0, -80.0);
        }
        ///////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////////
        // --> IMPLEMENTS A SIMPLE PROPORTIONAL CONTROLLER TO OPERATE THE RUDDER
        if (this->rudderPosition > this->rudderJoint->UpperLimit())
        {
            this->rudderPosition = this->rudderJoint->UpperLimit();
        }
        else if (this->rudderPosition < this->rudderJoint->LowerLimit())
        {
            this->rudderPosition = this->rudderJoint->UpperLimit();
        }

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
        ///////////////////////////////////////////////////////////////////////////////
    }
}
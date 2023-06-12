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
#include "../include/anemometerPlugin.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int16.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AnemometerPlugin)

//////////////////////////////////////////////////////////////////////
// CONSTRUCTOR
AnemometerPlugin::AnemometerPlugin()
{
    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    sensorPublisher = rosNode.advertise<std_msgs::Float32MultiArray>("/eboat/sensors/anemometer", 1000);
}

//////////////////////////////////////////////////////////////////////
// DESTRUCTOR
AnemometerPlugin::~AnemometerPlugin()
{
}

//////////////////////////////////////////////////////////////////////
// 
void AnemometerPlugin::Load(physics::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    //GZ_ASSERT(_model, "PropulsorControllerPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "PropulsorControllerPlugin _sdf pointer is NULL");
    //this->model = _model;
    this->sdf   = _sdf;

    //this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "PropulsorControllerPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "PropulsorControllerPlugin physics pointer is NULL");

    /*if (_sdf->HasElement("link_name"))
    {
        this->link = this->model->GetLink(_sdf->Get<std::string>("link_name"));
    }
    else
        this->link = this->model->GetLink("base_link");*/


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
    //this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    
}

/////////////////////////////////////////////////
void PropulsorControllerPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&AnemometerPlugin::OnSensorUpdate, this));
}

//////////////////////////////////////////////////////////////////////
void PropulsorControllerPlugin::OnSensorUpdate()
{
    std::cout<<"--------------------------------------"<<std::end;
    std::cout<<_parent.GetName()<<std::endl;
    std::cout<<"--------------------------------------"<<std::end;
    this->world->SetPaused(true);
}
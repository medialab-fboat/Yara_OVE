// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#include <algorithm>

#include "ignition/math/Pose3.hh"
#include "ignition/math/Vector3.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/sensors/RaySensor.hh"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "../include/bowVector.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BowVectorPlugin)

//////////////////////////////////////////////////////////////////////
BowVectorPlugin::BowVectorPlugin(): freq(1.0)
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
    //this->obsPub = rosNode.advertise<std_msgs::Float32MultiArray>("/eboat/mission_control/observations", 100);
    //this->maneuverObs = rosNode.advertise<std_msgs::Float32MultiArray>("/eboat/mission_control/maneuverObs", 500);
}

void BowVectorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "Sailor _model pointer is NULL");
    GZ_ASSERT(_sdf, "Sailor _sdf pointer is NULL");
    this->model = _model;

    this->world = _model->GetWorld();
    GZ_ASSERT(this->world, "Sailor world pointer is NULL");

    if (_sdf->HasElement("link_name"))
    {
        this->link = _model->GetLink(_sdf->Get<std::string>("link_name"));
    }
    else
        this->link = _model->GetLink("base_link");


    if (_sdf->HasElement("frequency"))
        this->freq = _sdf->Get<int>("frequency");

    // Forward direction in World frame
    if (_sdf->HasElement("ahead"))
        this->ahead = _sdf->Get<ignition::math::Vector3d>("ahead");
    else
        this->ahead = ignition::math::Vector3d(1,0,0);

    // Port direction in World frame
    if (_sdf->HasElement("port"))
        this->port = _sdf->Get<ignition::math::Vector3d>("port");
    else
        this->port = ignition::math::Vector3d(0,1,0);
    
    std::string topic_name = "/";
    topic_name.append(this->model->GetName());
    topic_name.append("/sensors/bow_vector");

    this->obsPub = rosNode.advertise<std_msgs::Float32MultiArray>(topic_name, 1);

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
}

/////////////////////////////////////////////////
void BowVectorPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&BowVectorPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////////////////////////
void BowVectorPlugin::OnUpdate()
{
    float simtime = this->world->SimTime().Float();
    
    float Q = simtime * this->freq;
    Q -= int(Q);
    if ((Q >= 0.0) & (Q <= 0.00001))
    {
        
        this->timeOfLastObs = simtime;

        std_msgs::Float32MultiArray obsMsg;
        
        ignition::math::Vector3d aheadD = this->model->WorldPose().Rot().RotateVector(this->ahead); //--> Dynamic ahead direction, it changes as the boats move around.
        //ignition::math::Vector3d portD = this->model->WorldPose().Rot().RotateVector(this->port);  //--> Dynamic port direction, it changes as the boats move around.

        obsMsg.data.push_back((float) aheadD.X());
        obsMsg.data.push_back((float) aheadD.Y());
        obsMsg.data.push_back((float) aheadD.Z());

        //SIMULATION TIME STAMP
        //obsMsg.data.push_back(simtime);

        // PUBLISH OBSERVATIONS
        this->obsPub.publish(obsMsg);
        
        //////////////////////////////////////
        /*std::cout << "+++++++++++++++++++++++++++++++++++++++++" << std::endl;
        std::cout << "->link name  = " << this->link->GetName() << std::endl;*/
    }
}
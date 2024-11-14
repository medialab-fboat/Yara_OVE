// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#include "ignition/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Point.h"

namespace gazebo
{
class AtmosfericControlPlugin : public WorldPlugin
{
    /// \brief Pointer to the World.
    protected: physics::WorldPtr world;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber windSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->world = _parent;
        GZ_ASSERT(this->world, "AtmosfericControlPlugin world pointer is NULL");

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
    /*void AtmosfericControlPlugin::Init()
    {
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&AtmosfericControlPlugin::OnUpdate, this));
    }

    //////////////////////////////////////////////////////////////////////
    void AtmosfericControlPlugin::OnUpdate()
    {}*/

    //////////////////////////////////////////////////////////////////////
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg Used to set wind velocity vector.
    public: void OnRosMsg(const geometry_msgs::PointConstPtr &_msg)
    {
        this->world->Wind().SetLinearVel(ignition::math::Vector3d(_msg->x,_msg->y,_msg->z));
        //std::cout<<"True Wind: "<<_msg->x<<std::endl;
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }
};

GZ_REGISTER_WORLD_PLUGIN(AtmosfericControlPlugin)
}
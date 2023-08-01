// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#ifndef _PROPULSORCONTROLLER_PLUGIN_HH
#define _PROPULSORCONTROLLER_PLUGIN_HH

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int16.h"


namespace gazebo
{
    class PropulsorControllerPlugin : public ModelPlugin
    {
        /// \brief Constructor.
        public: PropulsorControllerPlugin();

        /// Documentation Inherited.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// Documentation Inherited
        public: virtual void Init();

        /// \brief Callback for World Update events.
        protected: virtual void OnUpdate();

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateConnection;

        /// \brief Pointer to the World.
        protected: physics::WorldPtr world;

        /// \brief Pointer to physics engine.
        protected: physics::PhysicsEnginePtr physics;

        /// \brief Pointer to model containing plugin.
        protected: physics::ModelPtr model;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Pointer to Propulsor joint
        protected: physics::JointPtr propulsorJoint;

        /// \brief Pointer to Eletric Engine link
        protected: physics::LinkPtr eletricEngineLink;

        /// \brief Eletric propulsion turbine velocity (the current real propulsor has 11 velocities, so turbineVel can assume discrete values [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5])
        protected: int turbineVel;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

        //////////////////////////////////////////////////////////////////////
        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        public: void OnRosMsg(const std_msgs::Int16ConstPtr &_msg)
        {
            this->turbineVel = _msg->data;
            //std::cout<<"Velocidade da turbina: "<<_msg->data<<" ( simtime = "<<this->world->SimTime().Float()<<" s)"<<std::endl;
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
}
#endif
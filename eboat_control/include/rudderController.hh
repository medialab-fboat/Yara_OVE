// Auhtor : araujo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#ifndef _RUDDERCONTROLLER_PLUGIN_HH
#define _RUDDERCONTROLLER_PLUGIN_HH

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"


namespace gazebo
{
    class RudderControllerPlugin : public ModelPlugin
    {
        /// \brief Constructor.
        public: RudderControllerPlugin();

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

        /// \brief Name of model containing plugin.
        //protected: std::string modelName;

        /// \brief Names of allowed target links, specified in sdf parameters.
        //protected: std::string linkName;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Rudder joint
        protected: physics::JointPtr rudderJoint;

        /// \brief Pointer to Rudder link
        protected: physics::LinkPtr rudderLink;

        /// \brief Rudder angular position. Goes from -60 to 60 deg (-1.0472 to 1.0472 rad).
        protected: double rudderPosition;

        /// \brief Rudder eletric engine turn velocity limit
        protected: double rudderEngVelMax;

        /// \brief Rudder eletric engine turn velocity
        protected: double rudderEngVel;

        /// \brief Constant to transform degree to rad
        private: double d2r;

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
        public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
        {
            this->rudderPosition = _msg->data * M_PI / 180.0;
            //std::cout<<"Angulo do leme: "<<_msg->data<<std::endl;
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
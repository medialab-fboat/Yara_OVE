// Auhtor : araujo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#ifndef _MISSIONCONTROL_PLUGIN_HH
#define _MISSIONCONTROL_PLUGIN_HH

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

#include <ros/ros.h>
#include "std_msgs/Float32.h"

namespace gazebo
{
    class MissionControlPlugin : public ModelPlugin
    {
        /// \brief Contructor.
        public: MissionControlPlugin();

        /// \brief Load the Plugin.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Initialize the Plugin.
        public: virtual void Init();

        /// \brief Callback for World Update events.
        protected: virtual void OnUpdate();

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateConnection;

        /// \brief Pointer to the World.
        protected: physics::WorldPtr world;

        /// \brief Pointer to model containing plugin.
        protected: physics::ModelPtr model;

        /// \brief Pointer to base link
        protected: physics::LinkPtr link;

        /// \brief Pointer to Sail Joint
        protected: physics::JointPtr sailJoint;

        /// \brief Pointer to rudder Joint
        protected: physics::JointPtr rudderJoint;

        /// \brief Pointer to propulsor Joint
        protected: physics::JointPtr propulsorJoint;
        
        /// \brief Agent interaction frequency (in Hz)
        protected: int freq;

        /// \brief Physics Engine Step Size
        protected: float timeOfLastObs;

        /// \brief Vector defining the forward direction.
        protected: ignition::math::Vector3d ahead; 

        /// \brief Vector defining the port direction.
        protected: ignition::math::Vector3d port;

        /// \brief Way Point for the boat
        protected: std::string wayPointModelName;

        /// \brief Way Point for the boat
        protected: physics::ModelPtr wayPoint;

        /// \brief Factor to tranform degree to rad
        private: const double d2r = M_PI / 180.0;

        /// \brief Factor to tranform degree to rad
        private: const double r2d = 180.0 / M_PI;

        private: float speedFactor; //--> in the real propulsor there are 5 speeds forward and 5 backward. speedFactor helps to convert the joint angular velocity to the discrete propulsor speed.

        /**
         * NodeHandle is the main access point to communications with the ROS system.
         * The first NodeHandle constructed will fully initialize this node, and the last
         * NodeHandle destructed will close down the node.
         */
        /// \brief A node use for ROS transport
        //private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::NodeHandle rosNode;

        //private: ros::Publisher distPub;

        //private: ros::Publisher rudderPub;
        
        //private: ros::Publisher sailPub;

        //private: ros::Publisher propulsorPub;

        //private: ros::Publisher windSpeedPub;

        //private: ros::Publisher windAnglePub;

        private: ros::Publisher obsPub;

        //private: ros::Publisher maneuverObs;
    };
}
#endif
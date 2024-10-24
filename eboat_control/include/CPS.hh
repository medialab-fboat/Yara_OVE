// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#ifndef _CPS_PLUGIN_HH
#define _CPS_PLUGIN_HH

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

#include <ros/ros.h>
#include "std_msgs/Float32.h"

namespace gazebo
{
    class CPSPlugin : public ModelPlugin
    {
        /// \brief Contructor.
        public: CPSPlugin();

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
        
        /// \brief Agent interaction frequency (in Hz)
        protected: int freq;

        /// \brief Physics Engine Step Size
        protected: float timeOfLastObs;

        /**
         * NodeHandle is the main access point to communications with the ROS system.
         * The first NodeHandle constructed will fully initialize this node, and the last
         * NodeHandle destructed will close down the node.
         */
        /// \brief A node use for ROS transport
        //private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::NodeHandle rosNode;

        private: ros::Publisher obsPub;
    };
}
#endif
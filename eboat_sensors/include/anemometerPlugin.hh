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
    class AnemometerPlugin : public SensorPlugin
    {
        /// \brief Constructor.
        public: AnemometerPlugin();

        /// \brief Destructor.
        public: ~AnemometerPlugin();

        /// Documentation Inherited.
        public: virtual void Load(physics::SensorPtr _parent, sdf::ElementPtr _sdf);

        /// Documentation Inherited
        public: virtual void Init();

        /// \brief Callback for World Update events.
        protected: virtual void OnSensorUpdate();

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateSensorConnection;

        /// \brief Pointer to the World.
        protected: physics::WorldPtr world;

        /// \brief Pointer to physics engine.
        protected: physics::PhysicsEnginePtr physics;

        /// \brief Pointer to base link.
        protected: physics::LinkPtr link;

        /// \brief Pointer to model containing plugin.
        //protected: physics::ModelPtr model;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Publisher sensorPublisher;
    };
}
#endif
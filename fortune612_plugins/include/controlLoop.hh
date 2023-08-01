// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#ifndef _CONTROLLOOP_PLUGIN_HH
#define _CONTROLLOOP_PLUGIN_HH

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
    class ControlLoopPlugin : public ModelPlugin
    {
        /// \brief Constructor.
        public: ControlLoopPlugin();

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

        /// \brief Model World Position
        protected: ignition::math::Vector3d modelPosition;

        /// \brief Pointer to the Main Sail joint
        protected: physics::JointPtr mainSailJoint;

        /// \brief Pointer to Jib Sail joint
        protected: physics::JointPtr jibSailJoint;

        /// \brief Sail angular position. Goes from 0 to 90 deg. The side (port or starbord) depends on the aparent wind direction.
        protected: double sailPosition;

        /// \brief Boom eletric engine pull/release velocity
        protected: double boomEngVel;

        /// \brief Sail joint
        protected: physics::JointPtr rudderJoint;

        /// \brief Rudder angular position. Goes from -60 to 60 deg (-1.0472 to 1.0472 rad).
        protected: double rudderPosition;

        /// \brief Rudder eletric engine turn velocity limit
        protected: double rudderEngVelMax;

        /// \brief Rudder eletric engine turn velocity
        protected: double rudderEngVel;

        /// \brief Constant to transform degree to rad
        private: double d2r;

        /// \brief Frequency to collect data from the environment
        protected: float freq;

        /// \brief Auxiliary variable
        private: float reftime;
    };
}
#endif
/*
    This plugin computes the forces acting on a sail in 
    the presence of wind. The principle of lift and drag
    is applied to compute the forces

    AUTHOR : Eduardo Charles Vasconcellos
    CONTACT: evasconcellos@id.uff.br

    10/2022
*/

#ifndef _EBOAT_GAZEBO_LIFTDARG_PLUGIN_HH_
#define _EBOAT_GAZEBO_LIFTDRAG_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

namespace gazebo
{
    class LiftDragForces : public ModelPlugin
    {
        /// \brief Constructor.
        public: LiftDragForces();

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
        protected: std::string modelName;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Pointer to the boom link.
        protected: physics::LinkPtr boomLink;

        /// \brief Pointer to the base link (Hull link).
        protected: physics::LinkPtr baseLink;

        /// \brief Pointer to the Boom/Sail
        protected: physics::JointPtr joint;

        /// \brief boatBow indicates the forward motion direction
        protected: ignition::math::Vector3d boatBow;

        /// \brief boatPort indicates the left side of the ship when facing forward
        protected: ignition::math::Vector3d boatPort;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d sailForward;

        /// \brief Upward indicates the upward direction (it points to the sky)
        protected: ignition::math::Vector3d sailUpward;

        /// \brief Aparent wind velocity vector;
        protected: ignition::math::Vector3d aparentWindVelVec;

        /// \brief center of pressure in link local coordinates
        protected: ignition::math::Vector3d sailCP;

        /// \brief Area of the sail
        protected: double sailArea;

        /// \brief Air density
        protected: double airRHO;

        /// \brief Sail lift coeficients
        protected: double sailCL[181];

        /// \brief Sail drag coeficients
        protected: double sailCD[181];

        /// \brief Rudder/Keel lift coeficients
        protected: double rudderCL[181];

        /// \brief Rudder/Keel drag coeficients
        protected: double rudderCD[181];

        /// \brief Fluid velocity vector;
        protected: ignition::math::Vector3d waterVelVec;

        /// \brief Pointer to the rudder link.
        protected: physics::LinkPtr rudderLink;

        /// \brief center of pressure in link local coordinates
        protected: ignition::math::Vector3d rudderCP;

        /// \brief Area of the rudder
        protected: double rudderArea;

        /// \brief Water density
        protected: double waterRHO;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d rudderForward;

        /// \brief Upward indicates the upward direction (it points to the sky)
        protected: ignition::math::Vector3d rudderUpward;

        /// \brief Pointer to the rudder link.
        protected: physics::LinkPtr keelLink;

        /// \brief center of pressure in link local coordinates
        protected: ignition::math::Vector3d keelCP;

        /// \brief Area of the rudder
        protected: double keelArea;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d keelForward;

        /// \brief Upward indicates the upward direction (it points to the sky)
        protected: ignition::math::Vector3d keelUpward;

        /// \brief Distance from the boom joint in with the wind force is applied (this distance is mesuare only in the boom direction)
        protected: double armLength;

        private: double atkangle;
        private: double alpha;
        private: double q ;
        private: int coefIndex;
        private: ignition::math::Vector3d chordLine;
        private: ignition::math::Vector3d upwardW;
        private: ignition::math::Vector3d normal;
        private: ignition::math::Vector3d dragDirection;
        private: ignition::math::Vector3d liftDirection;
        private: ignition::math::Vector3d lift;
        private: ignition::math::Vector3d drag;
        private: ignition::math::Vector3d force_on_sail;
        private: ignition::math::Vector3d force_on_rudder;
        private: ignition::math::Vector3d force_on_keel;
        private: ignition::math::Vector3d dynamic_cp;
        private: const double r2d = 180.0/M_PI;
    };
}
#endif
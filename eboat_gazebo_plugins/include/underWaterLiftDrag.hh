/*
    This plugin computes the forces acting on a sail in 
    the presence of wind. The principle of lift and drag
    is applied to compute the forces

    AUTHOR : araujo Charles Vasconcellos
    CONTACT: evasconcellos@id.uff.br

    10/2022
*/

#ifndef _EBOAT_GAZEBO_UNDERWATERLIFTDARG_PLUGIN_HH_
#define _EBOAT_GAZEBO_UNDERWATERLIFTDRAG_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

namespace gazebo
{
    class UnderWaterLiftDrag : public ModelPlugin
    {
        /// \brief Constructor.
        public: UnderWaterLiftDrag();

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

        /// \brief Pointer to the base link (Hull link).
        protected: physics::LinkPtr baseLink;

        /// \brief Pointer to the Boom/Sail
        protected: physics::JointPtr joint;

        /// \brief boatBow indicates the forward motion direction
        protected: ignition::math::Vector3d boatBow;

        /// \brief boatPort indicates the left side of the ship when facing forward
        protected: ignition::math::Vector3d boatPort;

        /// \brief Fluid velocity vector;
        protected: ignition::math::Vector3d waterVelVec;

        /// \brief Pointer to the foil link.
        protected: physics::LinkPtr link;

        /// \brief center of pressure in link local coordinates
        protected: ignition::math::Vector3d cp;

        /// \brief surface area of the foil
        protected: double area;

        /// \brief Water density
        protected: double waterRHO;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d forward;

        /// \brief Upward direction is used to indicate on which side of the simmetric foil the lift will take place
        protected: ignition::math::Vector3d upward;

        /// \brief Slope for the lift coeficient "curve"
        protected: double cla;

        private: double alpha;
        private: double alphaStall;
        private: double atkcos;
        private: double q ;
        private: double waterSpeed;
        private: double cl;
        private: double claStall;
        private: double cd;
        private: double cda;
        private: double cdaD;
        private: double alphaD;
        private: ignition::math::Vector3d forwardW;
        private: ignition::math::Vector3d upwardW;
        private: ignition::math::Vector3d normal;
        private: ignition::math::Vector3d dragDirection;
        private: ignition::math::Vector3d liftDirection;
        private: ignition::math::Vector3d lift;
        private: ignition::math::Vector3d drag;
        private: ignition::math::Vector3d force;
        private: ignition::math::Vector3d dynamic_cp;
        private: const double d2r = M_PI / 180.0;
    };
}
#endif
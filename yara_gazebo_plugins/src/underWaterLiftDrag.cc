/*
    This plugin computes the forces acting on a sail in 
    the presence of wind. The principle of lift and drag
    is applied to compute the forces

    AUTHOR : Eduardo Charles Vasconcellos
    CONTACT: evasconcellos@id.uff.br

    10/2022
*/

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "ignition/math/Pose3.hh"
#include "../include/underWaterLiftDrag.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(UnderWaterLiftDrag)

//////////////////////////////////////////////////////////////////////
UnderWaterLiftDrag::UnderWaterLiftDrag()
{
  this->area     = 0.2;
  this->waterRHO = 997.0;
  this->forward  = ignition::math::Vector3d(1,0,0);
  this->upward   = ignition::math::Vector3d(0,1,0);
  this->cp       = ignition::math::Vector3d(0,0,0);

  //-->Based on values from Heliciel free profiles data base (https://www.heliciel.com/profilesdatabase/)
  //   naca001234 (400000)
  //-->Lift coefs
  this->alphaStall = 30.0 * this->d2r;
  this->cla        = 1.267809 / this->alphaStall;
  this->claStall   = -1.267809 / (60 * this->d2r);
  //-->Drag coefs
  this->alphaD     = 8 * this->d2r;
  this->cda        = 0.043 / this->alphaD;
  this->cdaD       = (1.93 - 0.043) / (82 * this->d2r);
}

//////////////////////////////////////////////////////////////////////
void UnderWaterLiftDrag::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "UnderWaterLiftDrag _model pointer is NULL");
  GZ_ASSERT(_sdf, "UnderWaterLiftDrag _sdf pointer is NULL");
  this->model     = _model;
  this->modelName = _model->GetName();
  this->sdf       = _sdf;

  this->world     = this->model->GetWorld();
  GZ_ASSERT(this->world, "UnderWaterLiftDrag world pointer is NULL");

  if (_sdf->HasElement("base_link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("base_link_name");
    this->baseLink = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
    this->baseLink = this->model->GetLink("base_link");
  }
  
  if (_sdf->HasElement("boat_bow"))
    this->boatBow = _sdf->Get<ignition::math::Vector3d>("boat_bow");
  else
    this->boatBow = ignition::math::Vector3d(1,0,0);
  
  if (_sdf->HasElement("boat_port"))
    this->boatPort = _sdf->Get<ignition::math::Vector3d>("boat_port");
  else
    this->boatPort = ignition::math::Vector3d(0,1,0);
  
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    this->link = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
    this->link = this->model->GetLink("rudder_link");
  }
  GZ_ASSERT(this->link, "UnderWaterLiftDrag rudder link pointer is NULL");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  //if (_sdf->HasElement("cda"))
  //  this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  //if (_sdf->HasElement("cda_stall"))
  //  this->cdaStall = _sdf->Get<double>("cda_stall");

  //if (_sdf->HasElement("cma_stall"))
  //  this->cmaStall = _sdf->Get<double>("cma_stall");
  
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");

  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  if (_sdf->HasElement("water_density"))
    this->waterRHO = _sdf->Get<double>("water_density");
}

//////////////////////////////////////////////////////////////////////
void UnderWaterLiftDrag::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&UnderWaterLiftDrag::OnUpdate, this));
}

//////////////////////////////////////////////////////////////////////
void UnderWaterLiftDrag::OnUpdate()
{
  /////////////////////////////////////////////////////////
  //--> FLUID VELOCITY
  //      The water velocity are compose by to components:
  //      the negative boat velocity + the ocean/river current
  //      waterVelVec = waterNaturalVel - linkVelVec
  this->waterVelVec = -this->link->WorldLinearVel(this->cp);
  /////////////////////////////////////////////////////////

  if (this->waterVelVec.Length() > 0.01)
  {
    ignition::math::Pose3d pose = this->link->WorldPose();
    /////////////////////////////////////////////////////////
    //-->ATTACK ANGLE
    //    The attack angle is defined as the angle between the
    //    the rudder chord line and the inflow (see https://www.meoexams.com/post/rudder-lift-and-drag-force).
    //->Chord line (-forwardW) direction in the World frame
    this->forwardW = pose.Rot().RotateVector(this->forward);
    //->Upward direction in the World frame
    this->upwardW  = pose.Rot().RotateVector(this->upward);

    //->Normal to the Lift-Drag plane
    ignition::math::Vector3 ldNormal = forwardW.Cross(upwardW).Normalize();

    //->Fuid (water) velocity projected in the Lift-Drag plane
    ignition::math:: Vector3 velInLDPlane = this->waterVelVec - this->waterVelVec.Dot(ldNormal) * ldNormal;

    //->Attack angle
    this->atkcos   = std::clamp(velInLDPlane.Dot(this->forwardW)/(velInLDPlane.Length() * this->forwardW.Length()), -1.0, 1.0);
    this->alpha    = M_PI - acos(atkcos);
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> COMPUTE LIFT AND DRAG FORCES
    // Drag direction
    this->dragDirection = velInLDPlane;
    dragDirection.Normalize();

    // Lift direction
    this->liftDirection = velInLDPlane.Cross(ldNormal);
    this->liftDirection.Normalize();

    // Invert lift direction depending on the side of the foil facing the flux
    if (velInLDPlane.Dot(upwardW)/(velInLDPlane.Length()*upwardW.Length()) < 0)
      this->liftDirection = this->liftDirection * (-1);

    //-->Compute dynamic pressure
    this->waterSpeed = velInLDPlane.Length();
    this->q = 0.5 * this->waterRHO * this->area * waterSpeed * waterSpeed;

    //-->Compute lift coeficient
    if (this->alpha > this->alphaStall)
    {
      this->cl = (this->cla * this->alphaStall + this->claStall * (this->alpha - this->alphaStall));
    }
    else
    {
      this->cl = this->cla * this->alpha;
    }
    this->lift = this->q * this->cl * this->liftDirection;

    //-->Compute lift coeficient
    if (this->alpha > this->alphaD)
    {
      this->cd = this->cda * this->alphaD + this->cdaD * (this->alpha - this->alphaD);
    }
    else
    {
      this->cd = this->cda * this->alpha;
    }
    this->drag = this->q * this->cd * this->dragDirection;
    //
    this->force = this->lift + this->drag;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> APPLY FORCE
    //->Pressure center: The pressure center is sets the position in which the aerodynamic force will be applied.
    //                   It's psition are relative to the base_link's center of mass (CoG).
    //                   The applied force vector is expressed in World frame and the centor of pressure are expressed in the link own frame.
    //->Apply resultant force
    this->link->AddForceAtRelativePosition(this->force, this->cp);
    
    
    std::cout << "link_name  : " << this->link->GetName() << std::endl;
    std::cout << "Vel_Upw ang: " << acos(velInLDPlane.Dot(upwardW)/(velInLDPlane.Length()*upwardW.Length())) * 180.0 / M_PI << " (" << velInLDPlane.Dot(upwardW)/(velInLDPlane.Length()*upwardW.Length()) << ")" << std::endl;
    std::cout << "waterVel   : " << this->waterVelVec << std::endl;
    std::cout << "ldpWaterVel: " << velInLDPlane << std::endl;
    std::cout << "atk angle  : " << this->alpha * 180.0 / M_PI << std::endl;
    std::cout << "lift       : " << this->lift << " (" << this->lift.Length() << ")" << std::endl;
    std::cout << "drag       : " << this->drag << " (" << this->drag.Length() << ")" << std::endl;
    std::cout << "force      : " << this->force << " (" << this->force.Length() << ")" << std::endl;
    std::cout <<"------------------------------------" << std::endl;
  }
}
/*
    This plugin computes the forces acting on a sail in 
    the presence of wind. The principle of lift and drag
    is applied to compute the forces

    AUTHOR : araujo Charles Vasconcellos
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
#include "../include/sailLiftDragForces.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SailLiftDragForces)

//////////////////////////////////////////////////////////////////////
SailLiftDragForces::SailLiftDragForces()
{
  //->lift and drag coeficients for the sail
  double sxl[] = {0.0000, 0.0283, 0.0498, 0.0671, 0.0829, 0.1000, 0.1211, 0.1489, 0.1862, 0.2357, 0.3000, 0.3804, 0.4714, 0.5663, 0.6582, 0.7400, 0.8069, 0.8616, 0.9089, 0.9535, 1.0000, 1.0518, 1.1066, 1.1604, 1.2095, 1.2500, 1.2791, 1.2972, 1.3058, 1.3063, 1.3000, 1.2885, 1.2738, 1.2577, 1.2425, 1.2300, 1.2217, 1.2164, 1.2122, 1.2074, 1.2000, 1.1888, 1.1744, 1.1582, 1.1413, 1.1250, 1.1102, 1.0970, 1.0848, 1.0734, 1.0625, 1.0516, 1.0404, 1.0284, 1.0151, 1.0000, 0.9829, 0.9640, 0.9436, 0.9222, 0.9000, 0.8776, 0.8556, 0.8348, 0.8160, 0.8000, 0.7871, 0.7762, 0.7658, 0.7542, 0.7400, 0.7221, 0.7010, 0.6779, 0.6539, 0.6300, 0.6071, 0.5850, 0.5633, 0.5418, 0.5200, 0.4976, 0.4745, 0.4505, 0.4257, 0.4000, 0.3733, 0.3458, 0.3176, 0.2889, 0.2600, 0.2310, 0.2021, 0.1738, 0.1463, 0.1200, 0.0950, 0.0710, 0.0475, 0.0240, 0.0000, -0.0325, -0.0644, -0.0958, -0.1266, -0.1569, -0.1867, -0.2159, -0.2446, -0.2727, -0.3003, -0.3273, -0.3538, -0.3797, -0.4051, -0.4300, -0.4543, -0.4781, -0.5013, -0.5240, -0.5462, -0.5678, -0.5888, -0.6093, -0.6293, -0.6487, -0.6676, -0.6859, -0.7037, -0.7210, -0.7377, -0.7539, -0.7695, -0.7846, -0.7991, -0.8131, -0.8265, -0.8394, -0.8518, -0.8636, -0.8749, -0.8856, -0.8958, -0.9054, -0.9145, -0.9231, -0.9311, -0.9386, -0.9455, -0.9519, -0.9577, -0.9630, -0.9677, -0.9719, -0.9756, -0.9787, -0.9813, -0.9833, -0.9848, -0.9858, -0.9862, -0.9860, -0.9853, -0.9841, -0.9823, -0.9800, -0.9771, -0.9737, -0.9698, -0.9653, -0.9603, -0.9547, -0.9486, -0.9419, -0.9347, -0.9269, -0.9186, -0.9098, -0.9004, -0.8905, -0.8800};
  double sxd[] = {0.0500, 0.0488, 0.0484, 0.0489, 0.0502, 0.0523, 0.0553, 0.0591, 0.0638, 0.0693, 0.0756, 0.0828, 0.0909, 0.0997, 0.1094, 0.1200, 0.1314, 0.1436, 0.1567, 0.1706, 0.1854, 0.2010, 0.2175, 0.2348, 0.2529, 0.2719, 0.2917, 0.3123, 0.3338, 0.3562, 0.3794, 0.4028, 0.4260, 0.4488, 0.4714, 0.4937, 0.5156, 0.5373, 0.5587, 0.5798, 0.6005, 0.6210, 0.6412, 0.6611, 0.6807, 0.7000, 0.7190, 0.7377, 0.7561, 0.7742, 0.7920, 0.8095, 0.8268, 0.8437, 0.8603, 0.8766, 0.8927, 0.9084, 0.9238, 0.9390, 0.9538, 0.9684, 0.9826, 0.9966, 1.0102, 1.0236, 1.0367, 1.0494, 1.0619, 1.0741, 1.0859, 1.0975, 1.1088, 1.1198, 1.1303, 1.1404, 1.1501, 1.1593, 1.1681, 1.1764, 1.1843, 1.1918, 1.1988, 1.2054, 1.2115, 1.2172, 1.2225, 1.2273, 1.2317, 1.2357, 1.2392, 1.2422, 1.2449, 1.2470, 1.2488, 1.2501, 1.2509, 1.2514, 1.2514, 1.2509, 1.2500, 1.2487, 1.2469, 1.2447, 1.2420, 1.2389, 1.2354, 1.2314, 1.2270, 1.2221, 1.2168, 1.2111, 1.2049, 1.1983, 1.1912, 1.1837, 1.1758, 1.1674, 1.1587, 1.1498, 1.1409, 1.1319, 1.1228, 1.1137, 1.1046, 1.0953, 1.0861, 1.0767, 1.0673, 1.0579, 1.0484, 1.0388, 1.0292, 1.0195, 1.0098, 1.0000, 0.9902, 0.9802, 0.9703, 0.9603, 0.9502, 0.9400, 0.9299, 0.9196, 0.9093, 0.8989, 0.8885, 0.8780, 0.8675, 0.8569, 0.8462, 0.8355, 0.8248, 0.8139, 0.8031, 0.7921, 0.7811, 0.7701, 0.7590, 0.7478, 0.7366, 0.7253, 0.7139, 0.7025, 0.6911, 0.6796, 0.6680, 0.6564, 0.6447, 0.6329, 0.6211, 0.6093, 0.5974, 0.5854, 0.5734, 0.5613, 0.5491, 0.5369, 0.5247, 0.5124, 0.5000};
  for (int i = 0; i < 181; i++)
  {
    this->sailCL[i] = sxl[i];
    this->sailCD[i] = sxd[i];
  }
}

//////////////////////////////////////////////////////////////////////
void SailLiftDragForces::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SailLiftDragForces _model pointer is NULL");
  GZ_ASSERT(_sdf, "SailLiftDragForces _sdf pointer is NULL");
  this->model     = _model;
  this->modelName = _model->GetName();
  this->sdf       = _sdf;

  this->world     = this->model->GetWorld();
  GZ_ASSERT(this->world, "SailLiftDragForces world pointer is NULL");

  if (_sdf->HasElement("boom_link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("boom_link_name");
    this->boomLink = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
    this->boomLink = this->model->GetLink("boom_link");
  }
  GZ_ASSERT(this->boomLink, "SailLiftDragForces boom link pointer is NULL");

  if (_sdf->HasElement("base_link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("base_link_name");
    this->baseLink = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
    this->baseLink = this->model->GetLink("base_link");
  }
  GZ_ASSERT(this->boomLink, "SailLiftDragForces base link pointer is NULL");
  
  if (_sdf->HasElement("boat_bow"))
    this->boatBow = _sdf->Get<ignition::math::Vector3d>("boat_bow");
  else
    this->boatBow = ignition::math::Vector3d(1,0,0);
  
  if (_sdf->HasElement("boat_port"))
    this->boatPort = _sdf->Get<ignition::math::Vector3d>("boat_port");
  else
    this->boatPort = ignition::math::Vector3d(0,1,0);
  
  if (_sdf->HasElement("sail_forward"))
    this->sailForward = _sdf->Get<ignition::math::Vector3d>("sail_forward");
  else
    this->sailForward = ignition::math::Vector3d(1,0,0);

  if (_sdf->HasElement("sail_upward"))
    this->sailUpward = _sdf->Get<ignition::math::Vector3d>("sail_upward");
  else
    this->sailUpward = ignition::math::Vector3d(0,0,1);

  if (_sdf->HasElement("sail_cp"))
    this->sailCP = _sdf->Get<ignition::math::Vector3d>("sail_cp");
  else
    this->sailCP = ignition::math::Vector3d(0,0,0.3);

  if (_sdf->HasElement("sail_area"))
    this->sailArea = _sdf->Get<double>("sail_area");
  else
    this->sailArea = 3.0;

  if (_sdf->HasElement("air_density"))
    this->airRHO = _sdf->Get<double>("air_density");
  else
    this->airRHO = 1.1839;

  if (_sdf->HasElement("boom_joint_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("boom_joint_name");
    this->joint = this->model->GetJoint(elem->Get<std::string>());
  }
  else
    this->joint = this->boomLink->GetParentJoints()[0];

  if (_sdf->HasElement("boom_length"))
    this->armLength = _sdf->Get<double>("boom_length")*0.25;
  else
    this->armLength = 0.59;

}

//////////////////////////////////////////////////////////////////////
void SailLiftDragForces::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SailLiftDragForces::OnUpdate, this));
  
  //this->armLength = 0.06;
  this->boomLink->SetWindEnabled(true);
}

//////////////////////////////////////////////////////////////////////
void SailLiftDragForces::OnUpdate()
{
  /////////////////////////////////////////////////////////
  // WIND FUNCTION FOR TESTING SAIL
  float simtime = this->world->SimTime().Float();

  /////////////////////////////////////////////////////////
  //--> Aparent wind computation
  // boatVelVec        = this->boomLink->WorldCoGLinearVel();
  // trueWindVelVec    = this->boomLink->WorldWindLinearVel();
  // aparentWindVelVec = trueWindVelVec - boatVelVec;
  this->aparentWindVelVec = this->boomLink->WorldWindLinearVel() - this->boomLink->WorldCoGLinearVel();
  //////////////////////////////////////////////////////////

  if (this->aparentWindVelVec.Length() > 0)
  {
    //////////////////////////////////////////////////////////
    //--> ATTACK ANGLE COMPUTATION
    //   The sail frame (boom frame) origin is defined as
    //   the joint origin. The chord line is defined by the forward
    //   vector. By default the chord line is defined in the X axis.
    //   The sail plan is defined by upward and forward vectors. By
    //   default, it is defined as the XZ plan.
    //->Chord line (forward) direction in the World frame
    this->chordLine = this->boomLink->WorldPose().Rot().RotateVector(-this->sailForward);
    //->Upward direction in the World frame
    this->upwardW   = this->boomLink->WorldPose().Rot().RotateVector(this->sailUpward);
    //->Normal to sail plan (it is used to find in which side of the sail the wind is making pressure).
    this->normal    = this->chordLine.Cross(this->upwardW);
    //->Attack angle
    this->atkangle = acos((this->chordLine.Dot(this->aparentWindVelVec)) / (this->chordLine.Length() * this->aparentWindVelVec.Length()))*180.0/M_PI;
    //->alpha is the angle between the inflow direction and the normal to sail plan.
    this->alpha    = acos(this->normal.Dot(this->aparentWindVelVec)/(this->normal.Length() * this->aparentWindVelVec.Length()))*180.0/M_PI;
    //->Lift and drag directions
    this->dragDirection = this->aparentWindVelVec/this->aparentWindVelVec.Length();
    this->liftDirection = this->upwardW.Cross(this->dragDirection);
    if (this->alpha < 90)
      this->liftDirection *= -1;
    else
      this->normal *= -1;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> COMPUTE LIFT AND DRAG FORCES
    //->dynamic pressure
    int iatk        = int(this->atkangle);
    double windVel  = this->aparentWindVelVec.Length();
    this->q         = 0.5 * this->airRHO * this->sailArea * windVel * windVel;
    this->coefIndex = (this->atkangle - iatk) < 0.5 ? iatk
                                                    : iatk + 1;
    if (this->coefIndex > 100)
      this->lift  = q * this->sailCL[100] * this->liftDirection;
    else
      this->lift  = q * this->sailCL[this->coefIndex] * this->liftDirection;
    //this->lift  = q * this->sailCL[this->coefIndex] * this->liftDirection;  //-> the lift force is normal to the lift-drag plan
    this->drag  = q * this->sailCD[this->coefIndex] * this->dragDirection; //-> the drag
    this->force = this->lift + this->drag;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> APPLY FORCE
    //->Pressure center: The pressure center is sets the position in which the aerodynamic force will be applied.
    //                   It's position is relative to the base_link's center of mass (CoG), and it is stored in the variable sailCP.
    //                   The applied force vector is expressed in World frame and the centor of pressure are expressed in the link own frame.
    
    //->Moving force: Due to some URDF/SDF limitations on build joints, if we apply the wind force on the sail 3D element (sail_link + boom_link) this force
    //                will be transmited to the boat by the boom_joint (a revoltute joint). Therefore, the wind force could generate a not realistic torque
    //                on the revolute axis. To avoid this and emulate a more realistic boat behavior, we will apply the force direct to the boat 3D element.
    //                The position where the force will be applied is given by the dynamic_cp variable.
    ignition::math::Vector3d bow = this->baseLink->WorldPose().Rot().RotateVector(this->boatBow);
    ignition::math::Vector3d port = this->baseLink->WorldPose().Rot().RotateVector(this->boatPort);
    ignition::math::Vector3d forwardForce = this->force.Dot(bow) * bow;
    ignition::math::Vector3d lateralForce = this->force.Dot(port) * port;
    this->baseLink->AddForceAtRelativePosition(this->force, this->sailCP);
    // --> EMULA A RESISTENCIA DA BOLINA A FORCA LATERAL
    //this->model->GetLink("keel_link")->AddForceAtRelativePosition(-lateralForce, ignition::math::Vector3d(0,0,0.5));
    

    //->Force on sail: As we will aplly the wind force direct on the boat element, the sail element will stand still and it will not change position with the
    //                 wind direction and speed, as it should do.
    //                 In a real sailing boat the crew do not set a fixe position for the boom, they increases and decrease the boom cable in order to give the
    //                 boom more or less freedom of movement. The boom's position is defined by how much cable the crew released and the wind's direction and speed.
    //                 To emulate this behavior, the sail 3D element should change position according the wind's direction and speed.
    //
    //                 torque = (force.Dot(ldNormal)*ldNormal).Length()*armLength; // modulus of projected force times the arm length.
    //
    // this->joint->SetForce(0, ((force.Dot(normal)*normal).Length()*armLength));
    double torque = (this->alpha < 90) ? (force.Dot(this->normal)*this->normal).Length()*this->armLength*(-1) // Negative
                                 : (force.Dot(this->normal)*this->normal).Length()*this->armLength;     // Positive
    this->joint->SetForce(0, torque);
    
  }
}
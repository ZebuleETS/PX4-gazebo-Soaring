/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "LiftDrag.hh"
#include <random>
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/msgs/thermal.pb.h"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LiftDragPrivate
{
  // Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(const UpdateInfo &_info, EntityComponentManager &_ecm);


  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Coefficient of Lift / alpha slope.
  /// Lift = C_L * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cla = 1.0;

  /// \brief Coefficient of Drag / alpha slope.
  /// Drag = C_D * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cda = 0.01;

  /// \brief Coefficient of Moment / alpha slope.
  /// Moment = C_M * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cma = 0.0;

  /// \brief angle of attach when airfoil stalls
  public: double alphaStall = GZ_PI_2;

  /// \brief Cl-alpha rate after stall
  public: double claStall = 0.0;

  /// \brief Cd-alpha rate after stall
  /// \todo(anyone): what's flat plate drag?
  public: double cdaStall = 1.0;

  /// \brief Cm-alpha rate after stall
  public: double cmaStall = 0.0;


  /// \brief How much Cm changes with a change in control
  /// surface deflection angle
  public: double cm_delta = 0.0;

  /// \brief air density
  /// at 25 deg C it's about 1.1839 kg/m^3
  /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
  public: double rho = 1.2041;

  /// \brief if the shape is aerodynamically radially symmetric about
  /// the forward direction. Defaults to false for wing shapes.
  /// If set to true, the upward direction is determined by the
  /// angle of attack.
  public: bool radialSymmetry = false;

  /// \brief effective planeform surface area
  public: double area = 1.0;

  /// \brief initial angle of attack
  public: double alpha0 = 0.0;

  /// \brief center of pressure in link local coordinates with respect to the
  /// link's center of mass
  public: gz::math::Vector3d cp = math::Vector3d::Zero;

  /// \brief Normally, this is taken as a direction parallel to the chord
  /// of the airfoil in zero angle of attack forward flight.
  public: math::Vector3d forward = math::Vector3d::UnitX;

  /// \brief A vector in the lift/drag plane, perpendicular to the forward
  /// vector. Inflow velocity orthogonal to forward and upward vectors
  /// is considered flow in the wing sweep direction.
  public: math::Vector3d upward = math::Vector3d::UnitZ;

  /// \brief how much to change CL per radian of control surface joint
  /// value.
  public: double controlJointRadToCL = 4.0;

  /// \brief Link entity targeted this plugin.
  public: Entity linkEntity;

  /// \brief Joint entity that actuates a control surface for this lifting body
  public: Entity controlJointEntity;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};
  /// Transport node and subscriber
  public: gz::transport::Node node;

  /// Mutex-protected list of received thermals
  public: std::mutex thermalsMutex;

  /// Last received thermal message
  public: std::optional<gz::msgs::ThermalGroup> latestThermalGroup;
  private: void OnThermalGroup(const gz::msgs::ThermalGroup &_msg);
  public: double lastVerticalWind = 0.0;  // Stores the last valid w




};
class DrydenGustGenerator
{
public:
  DrydenGustGenerator(double sigmaU, double sigmaV, double sigmaW,
                      double Lu, double Lv, double Lw, double V)
    : sigmaU(sigmaU), sigmaV(sigmaV), sigmaW(sigmaW),
      Lu(Lu), Lv(Lv), Lw(Lw), V(V),
      gustU(0), gustV(0), gustW(0)
  {
    rng.seed(std::random_device{}());
  }

  gz::math::Vector3d Sample(double dt)
  {
    std::normal_distribution<> normal(0.0, 1.0);
    double tauU = Lu / V;
    double tauV = Lv / V;
    double tauW = Lw / V;

    gustU += (-gustU / tauU) * dt + sigmaU * std::sqrt(2.0 / tauU) * normal(rng);
    gustV += (-gustV / tauV) * dt + sigmaV * std::sqrt(2.0 / tauV) * normal(rng);
    gustW += (-gustW / tauW) * dt + sigmaW * std::sqrt(2.0 / tauW) * normal(rng);

    return gz::math::Vector3d(gustU, gustV, gustW);
  }

private:
  double sigmaU, sigmaV, sigmaW;
  double Lu, Lv, Lw, V;
  double gustU, gustV, gustW;
  std::mt19937 rng;
};

//////////////////////////////////////////////////
void LiftDragPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{
  this->cla = _sdf->Get<double>("cla", this->cla).first;
  this->cda = _sdf->Get<double>("cda", this->cda).first;
  this->cma = _sdf->Get<double>("cma", this->cma).first;
  this->alphaStall = _sdf->Get<double>("alpha_stall", this->alphaStall).first;
  this->claStall = _sdf->Get<double>("cla_stall", this->claStall).first;
  this->cdaStall = _sdf->Get<double>("cda_stall", this->cdaStall).first;
  this->cmaStall = _sdf->Get<double>("cma_stall", this->cmaStall).first;
  this->rho = _sdf->Get<double>("air_density", this->rho).first;
  this->radialSymmetry = _sdf->Get<bool>("radial_symmetry",
      this->radialSymmetry).first;
  this->area = _sdf->Get<double>("area", this->area).first;
  this->alpha0 = _sdf->Get<double>("a0", this->alpha0).first;
  this->cp = _sdf->Get<math::Vector3d>("cp", this->cp).first;
  this->cm_delta = _sdf->Get<double>("cm_delta", this->cm_delta).first;

  // blade forward (-drag) direction in link frame
  this->forward =
      _sdf->Get<math::Vector3d>("forward", this->forward).first;
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  this->upward = _sdf->Get<math::Vector3d>(
      "upward", this->upward) .first;
  this->upward.Normalize();

  this->controlJointRadToCL = _sdf->Get<double>(
      "control_joint_rad_to_cl", this->controlJointRadToCL).first;

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities =
        entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
             << "The LiftDrag will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple link entities with name[" << linkName << "] found. "
             << "Using the first one.\n";
    }

    this->linkEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId))
    {
      this->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    gzerr << "The LiftDrag system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    auto controlJointName = _sdf->Get<std::string>("control_joint_name");
    auto entities =
        entitiesFromScopedName(controlJointName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Joint with name[" << controlJointName << "] not found. "
             << "The LiftDrag will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple joint entities with name[" << controlJointName
              << "] found. Using the first one.\n";
    }

    this->controlJointEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->controlJointEntity,
                                     components::Joint::typeId))
    {
      this->controlJointEntity = kNullEntity;
      gzerr << "Entity with name[" << controlJointName << "] is not a joint\n";
      this->validConfig = false;
      return;
    }
  }
  // Subscribe to the thermal topic
  this->node.Subscribe("/world/default/thermal_updrafts",
  &LiftDragPrivate::OnThermalGroup, this);

  // If we reached here, we have a valid configuration

  this->validConfig = true;
}

//////////////////////////////////////////////////
LiftDrag::LiftDrag()
    : System(), dataPtr(std::make_unique<LiftDragPrivate>())
{
}
void LiftDragPrivate::OnThermalGroup(const gz::msgs::ThermalGroup &_msg)
{
  std::lock_guard<std::mutex> lock(this->thermalsMutex);
  this->latestThermalGroup = _msg;

  std::cout << "\n[Received ThermalGroup message with "
            << _msg.thermals_size() << " thermals]\n";

  for (int i = 0; i < _msg.thermals_size(); ++i)
  {
    const auto &thermal = _msg.thermals(i);
    std::cout << "  Thermal ID: "     << thermal.id()
              << " | x: "             << thermal.x()
              << " | y: "             << thermal.y()
              << " | zi: "            << thermal.zi()
              << " | wi: "            << thermal.wi()
              << " | birth_time: "    << thermal.birth_time()
              << " | lifetime: "      << thermal.lifetime()
              << std::endl;
  }
}

/// \brief Compute Bencatel-leaned thermal center at UAV altitude
inline std::pair<double, double> ComputeLeaningCenter(
    double x0, double y0, double zi, double wi,
    double u, double v, double H,
    double Wx, double Wy)
{
  // No leaning below 10% of thermal height
  if (H / zi < 0.1)
    return {x0, y0};

  double dx = 0.0;
  double dy = 0.0;
  int N = 20; // number of integration steps
  double dz = H / static_cast<double>(N);

  for (int i = 0; i < N; ++i)
  {
    double h = i * dz + dz / 2.0; // midpoint rule
    double zzi = std::clamp(h / zi, 0.0, 1.0);

    // Allen's profile wz(h)
    double wz_h = wi * std::pow(zzi, 1.0 / 3.0) * (1.0 - 0.55 * zzi);
    wz_h = std::max(wz_h, 0.1); // avoid division by 0

    dx += ((Wx - u) / wz_h) * dz;
    dy += ((Wy - v) / wz_h) * dz;
  }

  // Optionally cap max displacement
  double maxDisplacement = 300.0;
  double dist = std::hypot(dx, dy);
  if (dist > maxDisplacement)
  {
    double scale = maxDisplacement / dist;
    dx *= scale;
    dy *= scale;
  }
  //thisone
  //std::cout << std::fixed << std::setprecision(3);
  //std::cout << "[Bencatel] Base: (" << x0 << ", " << y0 << ")"
            //<< " | dx: " << dx << ", dy: " << dy
           // << " | Final: (" << x0 + dx << ", " << y0 + dy << ")\n";

  return {x0 + dx, y0 + dy};
}




std::optional<std::pair<double, std::string>> ComputeThermalW(
  const gz::msgs::Thermal &thermal,
  const gz::math::Vector3d &uavPos,
  double simTime,
  double u, double v)

{
  double x = thermal.x();
  double y = thermal.y();
  //double x = 500;
  //double y = 500;


  double zi0 = thermal.zi();
  double wi0 = thermal.wi();



  double birth = thermal.birth_time();
  double life = thermal.lifetime();
  double t = simTime - birth;

  if (t < 0)
    return std::make_pair(0.0, "resting");
  if (t > life)
    return std::nullopt;

  // Phase-based scaling
  double scale = 1.0;
  std::string phase;
  double ratio = t / life;
  //if (ratio < 0.25) { scale = ratio / 0.25; phase = "growing"; }
  if (ratio < 0.25) { scale = 1.0; phase = "growing"; }

  else if (ratio < 0.75) { scale = 1.0; phase = "stable"; }
  //else { scale = 1.0 - (ratio - 0.75) / 0.25; phase = "fading"; }
  else { scale = 1.0 ; phase = "fading"; }
  //std::cout << "phase: (" << phase << ") | ration : " << ratio << " | scale : " << scale <<"\n";
  double zi = zi0;
  double wi = wi0 * scale;
  auto leaned = ComputeLeaningCenter(
  thermal.x(), thermal.y(),
  zi, wi, 0, 0, uavPos.Z(),
  0.0, 0.0);  //  fixed


  //double x = leaned.first;
  //double y = leaned.second;
  double xPlane = uavPos.X();
  double yPlane = uavPos.Y();
//thisone
  //std::cout << "[Leaning] Thermal ID: " << thermal.id()
          //<< " | Base: (" << thermal.x() << ", " << thermal.y() << ")"
          //<< " | Leaned: (" << x << ", " << y << ")"
          //<< " | UAV: (" << xPlane << ", " << yPlane << ")"
          //<< " | Height: " << uavPos.Z() << " m\n";

  double z = uavPos.Z();
  if (z < 0 || z > zi)
    return std::make_pair(0.0, phase);

  double zzi = z / zi;
  double dx = uavPos.X() - x;
  double dy = uavPos.Y() - y;
  double dist = std::sqrt(dx * dx + dy * dy);

  double rbar = 0.102 * std::pow(zzi, 1.0/3.0) * (1 - 0.25 * zzi) * zi;
  double r2 = std::max(10.0, rbar);
  double rr2 = dist / r2;
  if (rr2 > 2.0)
    return std::make_pair(0.0, phase);

  double r1r2 = (r2 < 600.0) ? (0.0011 * r2 + 0.14) : 0.8;
  double r1 = r1r2 * r2;
  double wBar = wi * std::pow(zzi, 1.0 / 3.0) * (1.0 - 1.1 * zzi);
  double wPeak = (r2 > r1) ?
    (3 * wBar * (std::pow(r2, 3) - std::pow(r2, 2) * r1)) /
    (std::pow(r2, 3) - std::pow(r1, 3)) : 0.0;

  double ka = 1.5265, kb = 3.6054, kc = -0.0176, kd = 0.0005;
  double ws = (1.0 / (1.0 + std::pow(std::abs(ka * (rr2 + kc)), kb))) + kd * rr2;
  ws = std::max(0.0, ws);

  double w1 = (dist > r1 && rr2 <= 2.0) ? (GZ_PI / 6.0) * std::sin(GZ_PI * rr2) : 0.0;
  double swd = (zzi > 0.5 && zzi < 0.9) ? 2.5 * (zzi - 0.5) : 0.0;
  double wD = (zzi > 0.5 && zzi < 0.9) ? -swd * w1 : 0.0;

  double w = ws * wPeak + wD;
  return std::make_pair(w, phase);
}


//////////////////////////////////////////////////
void LiftDragPrivate::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDragPrivate::Update");

  const auto worldLinVel = _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldAngVel = _ecm.Component<components::WorldAngularVelocity>(this->linkEntity);
  const auto worldPose = _ecm.Component<components::WorldPose>(this->linkEntity);

  components::WorldLinearVelocity *windLinearVel = nullptr;
  if (_ecm.EntityByComponents(components::Wind()) != kNullEntity)
  {
    Entity windEntity = _ecm.EntityByComponents(components::Wind());
    windLinearVel = _ecm.Component<components::WorldLinearVelocity>(windEntity);
  }

  components::JointPosition *controlJointPosition = nullptr;
  if (this->controlJointEntity != kNullEntity)
  {
    controlJointPosition = _ecm.Component<components::JointPosition>(this->controlJointEntity);
  }

  if (!worldLinVel || !worldAngVel || !worldPose)
    return;

  const auto &pose = worldPose->Data();
  const auto cpWorld = pose.Rot().RotateVector(this->cp);

  gz::math::Vector3d position = pose.Pos();

  gz::msgs::ThermalGroup thermalGroupCopy;
{
  std::lock_guard<std::mutex> lock(this->thermalsMutex);
    if (!this->latestThermalGroup.has_value())
  {
    //std::cout << "[LiftDrag] No thermal group available. Using ambient air.\n";
    // leave thermalGroupCopy empty or zero‐length;
    // but do *not* return—continue on to compute lift/drag
  }
  else
  {
    thermalGroupCopy = this->latestThermalGroup.value();
  }

}
  gz::math::Vector3d windVec(0, 0, 0);
  bool foundValidThermal = false;
  double simTime = std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count();

   if (this->latestThermalGroup.has_value())
  {
    for (int i = 0; i < thermalGroupCopy.thermals_size(); ++i)
    {
      const auto &thermal = thermalGroupCopy.thermals(i);
      double u = worldLinVel->Data().X();
      double v = worldLinVel->Data().Y();

      auto wPhase = ComputeThermalW(thermal, position, simTime,u,v);

      if (!wPhase.has_value())
        continue;

      double w = wPhase->first;
      std::string phase = wPhase->second;

      if (std::abs(w) > 1e-3)
{
  double dx = position.X() - thermal.x();
  double dy = position.Y() - thermal.y();
  double dist = std::sqrt(dx * dx + dy * dy);

  /* std::cout << "[LiftDrag] UAV is IN thermal ID " << thermal.id()
            << " | Distance: " << dist << " m"
            << " | Vertical wind w = " << w
            << " | Phase: " << phase
            << std::endl;  */

  windVec.Z() = w;
  foundValidThermal = true;
  break;
}

    }

    if (!foundValidThermal)
    {
      //std::cout << "[LiftDrag] No UAV inside any thermal. Ambient air only.\n";
    }
  }

  // Compute relative velocity: body velocity minus wind
  gz::math::Vector3d vel;
  const auto windEntity = _ecm.EntityByComponents(components::Wind());
  if (windEntity != kNullEntity)
  {
    auto windLin = _ecm.Component<components::WorldLinearVelocity>(
        windEntity)->Data();
    vel = worldLinVel->Data() +
          worldAngVel->Data().Cross(
            worldPose->Data().Rot().RotateVector(this->cp)) -
          (windLin + windVec);
  }
  else
  {
    vel = worldLinVel->Data() +
          worldAngVel->Data().Cross(
            worldPose->Data().Rot().RotateVector(this->cp)) -
          windVec;
  }

  if (vel.Length() <= 0.01)
    return;




  const auto velI = vel.Normalized();
  const auto forwardI = pose.Rot().RotateVector(this->forward);
  if (forwardI.Dot(vel) <= 0.0)
    return;

  math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  const auto spanwiseI = forwardI.Cross(upwardI).Normalize();
  const double sinSweepAngle = math::clamp(spanwiseI.Dot(velI), -1.0, 1.0);
  const double cos2SweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  double sweep = std::asin(sinSweepAngle);
  while (std::fabs(sweep) > 0.5 * GZ_PI)
    sweep = sweep > 0 ? sweep - GZ_PI : sweep + GZ_PI;

  const auto velInLDPlane = vel - vel.Dot(spanwiseI) * spanwiseI;
  const auto dragDirection = -velInLDPlane.Normalized();
  const auto liftI = spanwiseI.Cross(velInLDPlane).Normalized();
  const double cosAlpha = math::clamp(liftI.Dot(upwardI), -1.0, 1.0);

  double alpha = this->alpha0 - std::acos(cosAlpha);
  if (liftI.Dot(forwardI) >= 0.0)
    alpha = this->alpha0 + std::acos(cosAlpha);

  while (fabs(alpha) > 0.5 * GZ_PI)
    alpha = alpha > 0 ? alpha - GZ_PI : alpha + GZ_PI;

  const double speedInLDPlane = velInLDPlane.Length();
  const double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  double cl;
  if (alpha > this->alphaStall)
    cl = std::max(0.0, (this->cla * this->alphaStall + this->claStall * (alpha - this->alphaStall)) * cos2SweepAngle);
  else if (alpha < -this->alphaStall)
    cl = std::min(0.0, (-this->cla * this->alphaStall + this->claStall * (alpha + this->alphaStall)) * cos2SweepAngle);
  else
    cl = this->cla * alpha * cos2SweepAngle;

  if (controlJointPosition && !controlJointPosition->Data().empty())
    cl += this->controlJointRadToCL * controlJointPosition->Data()[0];

  math::Vector3d lift = cl * q * this->area * liftI;

  double cd;
  if (alpha > this->alphaStall)
    cd = (this->cda * this->alphaStall + this->cdaStall * (alpha - this->alphaStall)) * cos2SweepAngle;
  else if (alpha < -this->alphaStall)
    cd = (-this->cda * this->alphaStall + this->cdaStall * (alpha + this->alphaStall)) * cos2SweepAngle;
  else
    cd = std::fabs(this->cda * alpha * cos2SweepAngle);

  math::Vector3d drag = cd * q * this->area * dragDirection;

  double cm;
  if (alpha > this->alphaStall)
    cm = std::max(0.0, (this->cma * this->alphaStall + this->cmaStall * (alpha - this->alphaStall)) * cos2SweepAngle);
  else if (alpha < -this->alphaStall)
    cm = std::min(0.0, (-this->cma * this->alphaStall + this->cmaStall * (alpha + this->alphaStall)) * cos2SweepAngle);
  else
    cm = this->cma * alpha * cos2SweepAngle;

  if (controlJointPosition && !controlJointPosition->Data().empty())
    cm += this->cm_delta * controlJointPosition->Data()[0];

  math::Vector3d moment = cm * q * this->area * spanwiseI;
  math::Vector3d force = lift + drag;
  math::Vector3d torque = moment;
  force.Correct();
  torque.Correct();
  this->cp.Correct();

  const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, force, totalTorque);
}


//////////////////////////////////////////////////
void LiftDrag::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The LiftDrag system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();

}

//////////////////////////////////////////////////
void LiftDrag::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDrag::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);

      if ((this->dataPtr->controlJointEntity != kNullEntity) &&
          !_ecm.Component<components::JointPosition>(
              this->dataPtr->controlJointEntity))
      {
        _ecm.CreateComponent(this->dataPtr->controlJointEntity,
            components::JointPosition());
      }
    }
  }


  if (_info.paused)
    return;

  // This is not an "else" because "initialized" can be set in the if block
  // above
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info,_ecm);
  }
}

GZ_ADD_PLUGIN(LiftDrag,
                    System,
                    LiftDrag::ISystemConfigure,
                    LiftDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LiftDrag, "gz::sim::systems::liftdrag1")

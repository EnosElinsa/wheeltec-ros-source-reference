// Gazebo-native safety boundary. No ROS process or simulated clock owns the deadline.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/clock.pb.h>
#include <ignition/math/Rand.hh>
#include "wheeltec_sim_actuator/kinematics.hh"
#include "wheeltec_sim_actuator/simulation_clock.hh"

namespace wheeltec {
using namespace ignition::gazebo;
class Actuator final : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate {
  using Clock = std::chrono::steady_clock;
  Model model{kNullEntity};
  ignition::transport::Node transport;
  ignition::transport::Node::Publisher jointPublisher;
  ignition::transport::Node::Publisher clockPublisher;
  SimulationClock simulationClock;
  std::chrono::steady_clock::duration lastJointTime{-std::chrono::seconds(1)};
  std::mutex mutex;
  CommandLease lease;
  bool dropCommands{false};
  double wheelbase{.34}, track{.30}, radius{.065}, timeout{.25}, limit{.65}, minRadius{.6};
  std::array<Entity,6> joints{};
  const std::array<std::string,6> names{{"rear_left_joint", "rear_right_joint", "front_left_joint", "front_right_joint", "steer_left_joint", "steer_right_joint"}};
  void Command(const ignition::msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(mutex);
    if (dropCommands) return;
    const auto &l=msg.linear(); const auto &a=msg.angular();
    const bool valid=std::isfinite(l.x()) && std::isfinite(a.z()) &&
      std::isfinite(l.y()) && std::isfinite(l.z()) && std::isfinite(a.x()) && std::isfinite(a.y()) &&
      std::abs(l.y())+std::abs(l.z())+std::abs(a.x())+std::abs(a.y()) < 1e-6;
    lease.Set(valid?l.x():0,valid?a.z():0,std::chrono::duration<double>(Clock::now().time_since_epoch()).count());
  }
  void Drop(const ignition::msgs::Boolean &msg) {
    std::lock_guard<std::mutex> lock(mutex);dropCommands=msg.data();
  }
  static void Velocity(Entity joint, double value, EntityComponentManager &ecm) {
    auto cmd=ecm.Component<components::JointVelocityCmd>(joint);
    if (cmd) *cmd=components::JointVelocityCmd({value});
    else ecm.CreateComponent(joint, components::JointVelocityCmd({value}));
  }
 public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &, EventManager &) override {
    model=Model(entity);
    ignition::math::Rand::Seed(sdf->Get<unsigned int>("seed",17u).first);
    wheelbase=sdf->Get<double>("wheelbase", wheelbase).first;
    track=sdf->Get<double>("track", track).first;
    radius=sdf->Get<double>("radius", radius).first;
    timeout=sdf->Get<double>("timeout", timeout).first;
    lease.timeout=timeout;
    limit=sdf->Get<double>("steering_limit", limit).first;
    minRadius=sdf->Get<double>("min_radius", minRadius).first;
    if (!(wheelbase>0 && track>0 && radius>0 && timeout>0 && timeout<=.3 && limit>0 && minRadius>track/2))
      throw std::runtime_error("invalid actuator geometry/deadline");
    // Only this bridge-facing topic is accepted. No alternate ungated input.
    transport.Subscribe("/cmd_vel_safe", &Actuator::Command, this);
    // Fault injection only removes delivery; it can never grant motion or
    // renew the lease. This proves timeout independently of ROS zero output.
    transport.Subscribe("/sim/actuator/drop_commands", &Actuator::Drop, this);
    jointPublisher=transport.Advertise<ignition::msgs::Model>("/sim/joints");
    clockPublisher=transport.Advertise<ignition::msgs::Clock>("/sim/clock");
  }
  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override {
    double v=0,w=0;
    { std::lock_guard<std::mutex> lock(mutex);
      auto command=lease.Resolve(std::chrono::duration<double>(Clock::now().time_since_epoch()).count(),info.paused);
      v=command.first;w=command.second;
    }
    // Never hold a nonzero actuator setpoint across a paused update. On resume the
    // first PreUpdate expires a missing command before the physics step executes.
    if (info.paused) v=w=0;
    for (size_t i=0; i<joints.size(); ++i) {
      if (!joints[i]) joints[i]=model.JointByName(ecm,names[i]);
      if (!joints[i]) return;
      if (!ecm.Component<components::JointPosition>(joints[i]))
        ecm.CreateComponent(joints[i],components::JointPosition());
      if (!ecm.Component<components::JointVelocity>(joints[i]))
        ecm.CreateComponent(joints[i],components::JointVelocity());
    }
    const auto targets=Ackermann(v,w,wheelbase,track,radius,limit,minRadius);
    for (size_t side=0; side<2; ++side) {
      Velocity(joints[side],targets[side],ecm);
      Velocity(joints[side+2],targets[side+2],ecm);
      auto pos=ecm.Component<components::JointPosition>(joints[side+4]);
      const double angle=pos && !pos->Data().empty() ? pos->Data()[0] : 0;
      Velocity(joints[side+4],std::clamp(8*(targets[side+4]-angle),-2.,2.),ecm);
    }
  }
  void PostUpdate(const UpdateInfo &info,const EntityComponentManager &ecm) override {
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (simulationClock.Due(ns,info.paused)) {
      ignition::msgs::Clock clock;
      clock.mutable_sim()->set_sec(ns/1000000000);
      clock.mutable_sim()->set_nsec(ns%1000000000);
      clockPublisher.Publish(clock);
    }
    if (info.paused) return;
    if (info.simTime < lastJointTime) lastJointTime=info.simTime-std::chrono::seconds(1);
    if (info.simTime-lastJointTime<std::chrono::milliseconds(20)) return;
    ignition::msgs::Model msg;
    msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    msg.set_name("wheeltec");
    for (size_t i=0;i<joints.size();++i) {
      auto position=ecm.Component<components::JointPosition>(joints[i]);
      auto velocity=ecm.Component<components::JointVelocity>(joints[i]);
      if (!position || !velocity || position->Data().empty() || velocity->Data().empty()) return;
      auto joint=msg.add_joint();joint->set_name(names[i]);
      joint->mutable_axis1()->set_position(position->Data()[0]);
      joint->mutable_axis1()->set_velocity(velocity->Data()[0]);
    }
    jointPublisher.Publish(msg);lastJointTime=info.simTime;
  }
};
}
IGNITION_ADD_PLUGIN(wheeltec::Actuator, ignition::gazebo::System,
                   wheeltec::Actuator::ISystemConfigure, wheeltec::Actuator::ISystemPreUpdate,
                   wheeltec::Actuator::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(wheeltec::Actuator,"wheeltec::Actuator")

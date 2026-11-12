// Drives the AutoRace mission devices (traffic light lenses, barrier arm, folding
// intersection signs) from ROS 2 topics. This is the environment actuator only; race
// rules live in race_manager.py.
//
// Topics (relative to <ros><namespace>):
//   sub  light      std_msgs/String  red | yellow | green | off
//   sub  barrier    std_msgs/String  open | close
//   sub  direction  std_msgs/String  left | right | none   (which sign stands upright)
//   pub  joint_states  sensor_msgs/JointState  device joint angles (truth, not for the robot)
//   pub  light_state   std_msgs/String         current light, transient local
//
// Joints are moved kinematically toward their target at <joint speed=".."> rad/s so the
// motion is deterministic and only depends on simulation time.

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace autorace_gazebo
{

namespace
{
struct LensColor
{
  ignition::math::Color on;
  ignition::math::Color off;
};

const std::map<std::string, LensColor> kLens = {
  // Blender's v014 lens hues, converted to display RGB for Classic's renderer.
  {"red", {{1.0f, 0.051f, 0.026f, 1.0f}, {0.025f, 0.0001f, 0.00005f, 1.0f}}},
  {"yellow", {{1.0f, 0.650f, 0.026f, 1.0f}, {0.025f, 0.0095f, 0.00005f, 1.0f}}},
  {"green", {{0.026f, 1.0f, 0.086f, 1.0f}, {0.00005f, 0.025f, 0.0002f, 1.0f}}},
};

constexpr double kHalfPi = 1.5707963267948966;
}  // namespace

class MissionDevicesPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    ros_ = gazebo_ros::Node::Get(sdf);

    for (auto el = sdf->GetElement("joint"); el; el = el->GetNextElement("joint")) {
      Joint j;
      j.name = el->Get<std::string>("name");
      j.joint = model_->GetJoint(j.name);
      if (!j.joint) {
        RCLCPP_ERROR(ros_->get_logger(), "joint %s not found", j.name.c_str());
        continue;
      }
      j.default_q = el->Get<double>("default");
      j.speed = el->HasAttribute("speed") ? el->Get<double>("speed") : kHalfPi;
      j.q = j.target = j.default_q;
      joints_[j.name] = j;
    }
    light_link_ = sdf->Get<std::string>("light_link", "traffic_light").first;
    default_light_ = sdf->Get<std::string>("default_light", "red").first;
    light_ = default_light_;

    gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gz_node_->Init(model_->GetWorld()->Name());
    visual_pub_ = gz_node_->Advertise<gazebo::msgs::Visual>("~/visual");

    auto latched = rclcpp::QoS(1).transient_local();
    joint_pub_ = ros_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    light_pub_ = ros_->create_publisher<std_msgs::msg::String>("light_state", latched);

    light_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "light", 10, [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (m->data != "off" && !kLens.count(m->data)) {
          RCLCPP_WARN(ros_->get_logger(), "unknown light '%s'", m->data.c_str());
          return;
        }
        light_ = m->data;
        light_dirty_ = true;
      });
    barrier_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "barrier", 10, [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (m->data == "open") {
          SetTarget("barrier_joint", kHalfPi);
        } else if (m->data == "close") {
          SetTarget("barrier_joint", 0.0);
        } else {
          RCLCPP_WARN(ros_->get_logger(), "unknown barrier command '%s'", m->data.c_str());
        }
      });
    direction_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "direction", 10, [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mutex_);
        // 0 = upright, -pi/2 = folded for both signs.
        if (m->data == "left") {
          SetTarget("left_sign_joint", 0.0);
          SetTarget("right_sign_joint", -kHalfPi);
        } else if (m->data == "right") {
          SetTarget("right_sign_joint", 0.0);
          SetTarget("left_sign_joint", -kHalfPi);
        } else if (m->data == "none") {
          SetTarget("right_sign_joint", -kHalfPi);
          SetTarget("left_sign_joint", -kHalfPi);
        } else {
          RCLCPP_WARN(ros_->get_logger(), "unknown direction '%s'", m->data.c_str());
        }
      });

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MissionDevicesPlugin::OnUpdate, this, std::placeholders::_1));
    ApplyJoints();
    RCLCPP_INFO(ros_->get_logger(), "mission devices ready (%zu joints)", joints_.size());
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto & kv : joints_) {
      kv.second.q = kv.second.target = kv.second.default_q;
    }
    light_ = default_light_;
    light_dirty_ = true;
    last_update_ = gazebo::common::Time::Zero;
    ApplyJoints();
  }

private:
  struct Joint
  {
    std::string name;
    gazebo::physics::JointPtr joint;
    double default_q{0}, speed{kHalfPi}, q{0}, target{0};
  };

  void SetTarget(const std::string & name, double target)
  {
    auto it = joints_.find(name);
    if (it != joints_.end()) {
      it->second.target = target;
    }
  }

  void ApplyJoints()
  {
    for (auto & kv : joints_) {
      kv.second.joint->SetPosition(0, kv.second.q, true);
      kv.second.joint->SetVelocity(0, 0.0);
    }
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    double dt = (last_update_ == gazebo::common::Time::Zero) ? 0.0 : (info.simTime - last_update_).Double();
    last_update_ = info.simTime;
    if (dt < 0) {
      dt = 0;  // world reset
    }
    for (auto & kv : joints_) {
      Joint & j = kv.second;
      double step = j.speed * dt;
      j.q += std::clamp(j.target - j.q, -step, step);
    }
    ApplyJoints();

    // Re-send lens materials periodically: rendering scenes (gzclient, camera sensors)
    // may not exist yet when the first message goes out.
    if (light_dirty_ || (info.simTime - last_light_pub_).Double() > 1.0 ||
      info.simTime < last_light_pub_)
    {
      PublishLight();
      last_light_pub_ = info.simTime;
    }
    if ((info.simTime - last_joint_pub_).Double() >= 0.02 || info.simTime < last_joint_pub_) {
      sensor_msgs::msg::JointState js;
      js.header.stamp.sec = info.simTime.sec;
      js.header.stamp.nanosec = info.simTime.nsec;
      for (auto & kv : joints_) {
        js.name.push_back(kv.first);
        js.position.push_back(kv.second.joint->Position(0));
        js.velocity.push_back(kv.second.target);  // velocity field carries the commanded target
      }
      joint_pub_->publish(js);
      last_joint_pub_ = info.simTime;
    }
  }

  void PublishLight()
  {
    auto link = model_->GetLink(light_link_);
    if (!link) {
      return;
    }
    for (const auto & kv : kLens) {
      bool on = (kv.first == light_);
      const auto & c = on ? kv.second.on : kv.second.off;
      gazebo::msgs::Visual msg;
      msg.set_name(link->GetScopedName() + "::lens_" + kv.first);
      msg.set_parent_name(link->GetScopedName());
      auto * mat = msg.mutable_material();
      // Lit lenses have a dark physical base plus colored emission, like the
      // Blender material. Full diffuse + ambient + emission washed green to cyan.
      // Scale RGB only: Color * scalar would also lower alpha, making the
      // illuminated lens translucent against the dark housing.
      const float reflectance = on ? 0.3f : 1.0f;
      const ignition::math::Color base(
        c.R() * reflectance, c.G() * reflectance, c.B() * reflectance, 1.0f);
      gazebo::msgs::Set(mat->mutable_ambient(), base);
      gazebo::msgs::Set(mat->mutable_diffuse(), base);
      gazebo::msgs::Set(mat->mutable_emissive(), on ? c : ignition::math::Color(0, 0, 0, 1));
      gazebo::msgs::Set(mat->mutable_specular(), ignition::math::Color(0.02f, 0.02f, 0.02f, 1.0f));
      visual_pub_->Publish(msg);
    }
    if (light_dirty_) {
      std_msgs::msg::String s;
      s.data = light_;
      light_pub_->publish(s);
      light_dirty_ = false;
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr visual_pub_;
  gazebo::event::ConnectionPtr update_conn_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr light_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr light_sub_, barrier_sub_, direction_sub_;

  std::mutex mutex_;
  std::map<std::string, Joint> joints_;
  std::string light_link_, light_, default_light_;
  bool light_dirty_{true};
  gazebo::common::Time last_update_, last_light_pub_, last_joint_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(MissionDevicesPlugin)

}  // namespace autorace_gazebo

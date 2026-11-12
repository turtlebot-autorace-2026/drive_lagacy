// Environment detection regions (barrier S1/S2, start line, intersection branches).
//
// A region is "occupied" when any collision of the robot model overlaps the region box.
// Box/cylinder/sphere collisions are tested as oriented boxes (separating axis test);
// other shapes fall back to their world AABB. This is body overlap, not centre-point.
//
// Publishes std_msgs/Bool on <namespace>/<region name> (transient local, on change).
// These are judge/environment truth signals and must not be fed to the robot's driving code.

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace autorace_gazebo
{

namespace
{
struct Obb
{
  ignition::math::Vector3d c;
  std::array<ignition::math::Vector3d, 3> axis;
  ignition::math::Vector3d half;
};

Obb MakeObb(const ignition::math::Pose3d & pose, const ignition::math::Vector3d & size)
{
  Obb b;
  b.c = pose.Pos();
  b.axis = {pose.Rot().XAxis(), pose.Rot().YAxis(), pose.Rot().ZAxis()};
  b.half = size * 0.5;
  return b;
}

double Radius(const Obb & b, const ignition::math::Vector3d & n)
{
  return b.half.X() * std::abs(b.axis[0].Dot(n)) + b.half.Y() * std::abs(b.axis[1].Dot(n)) +
         b.half.Z() * std::abs(b.axis[2].Dot(n));
}

bool Overlap(const Obb & a, const Obb & b)
{
  std::vector<ignition::math::Vector3d> axes(a.axis.begin(), a.axis.end());
  axes.insert(axes.end(), b.axis.begin(), b.axis.end());
  for (const auto & u : a.axis) {
    for (const auto & v : b.axis) {
      auto n = u.Cross(v);
      if (n.SquaredLength() > 1e-12) {
        axes.push_back(n.Normalized());
      }
    }
  }
  const auto d = b.c - a.c;
  for (const auto & n : axes) {
    if (std::abs(d.Dot(n)) > Radius(a, n) + Radius(b, n)) {
      return false;
    }
  }
  return true;
}

bool CollisionObb(const gazebo::physics::CollisionPtr & col, Obb & out)
{
  auto shape = col->GetShape();
  ignition::math::Vector3d size;
  if (auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape)) {
    size = box->Size();
  } else if (auto cyl = boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(shape)) {
    size.Set(2 * cyl->GetRadius(), 2 * cyl->GetRadius(), cyl->GetLength());
  } else if (auto sph = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(shape)) {
    size.Set(2 * sph->GetRadius(), 2 * sph->GetRadius(), 2 * sph->GetRadius());
  } else {
    auto bb = col->BoundingBox();
    out = MakeObb(ignition::math::Pose3d(bb.Center(), ignition::math::Quaterniond::Identity), bb.Size());
    return true;
  }
  out = MakeObb(col->WorldPose(), size);
  return true;
}
}  // namespace

class RegionsPlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    ros_ = gazebo_ros::Node::Get(sdf);
    robot_name_ = sdf->Get<std::string>("robot_model", "turtlebot3").first;
    double rate = sdf->Get<double>("update_rate", 200.0).first;
    period_ = rate > 0 ? 1.0 / rate : 0.0;

    auto latched = rclcpp::QoS(1).transient_local();
    for (auto el = sdf->GetElement("region"); el; el = el->GetNextElement("region")) {
      Region r;
      r.name = el->Get<std::string>("name");
      r.box = MakeObb(el->Get<ignition::math::Pose3d>("pose"), el->Get<ignition::math::Vector3d>("size"));
      r.pub = ros_->create_publisher<std_msgs::msg::Bool>(r.name, latched);
      regions_.push_back(r);
    }
    for (auto & r : regions_) {
      Publish(r);
    }
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RegionsPlugin::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(ros_->get_logger(), "%zu regions watching model '%s'", regions_.size(), robot_name_.c_str());
  }

private:
  struct Region
  {
    std::string name;
    Obb box;
    bool active{false};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
  };

  void Publish(Region & r)
  {
    std_msgs::msg::Bool m;
    m.data = r.active;
    r.pub->publish(m);
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    if (info.simTime >= last_ && (info.simTime - last_).Double() < period_) {
      return;
    }
    last_ = info.simTime;

    std::vector<Obb> body;
    if (auto robot = world_->ModelByName(robot_name_)) {
      for (const auto & link : robot->GetLinks()) {
        for (const auto & col : link->GetCollisions()) {
          Obb b;
          if (CollisionObb(col, b)) {
            body.push_back(b);
          }
        }
      }
    }
    for (auto & r : regions_) {
      bool active = false;
      for (const auto & b : body) {
        if (Overlap(r.box, b)) {
          active = true;
          break;
        }
      }
      if (active != r.active) {
        r.active = active;
        Publish(r);
      }
    }
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_;
  gazebo::event::ConnectionPtr update_conn_;
  std::string robot_name_;
  double period_{0.005};
  gazebo::common::Time last_;
  std::vector<Region> regions_;
};

GZ_REGISTER_WORLD_PLUGIN(RegionsPlugin)

}  // namespace autorace_gazebo

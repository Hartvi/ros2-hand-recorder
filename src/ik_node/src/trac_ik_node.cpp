#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>


class TracIkNode : public rclcpp::Node {
public:
  TracIkNode() : Node("trac_ik_node") {
    // Parameters: set these to match your URDF link names.
    base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
    tip_link_  = this->declare_parameter<std::string>("tip_link", "end_effector_link");
    this->declare_parameter<std::string>("robot_description", "");

    timeout_ = this->declare_parameter<double>("timeout", 0.01);
    eps_     = this->declare_parameter<double>("eps", 1e-5);

    robot_desc_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&TracIkNode::on_robot_description, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ik_target", 10, std::bind(&TracIkNode::on_target_pose, this, std::placeholders::_1));

    sol_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    RCLCPP_INFO(get_logger(), "Waiting for /robot_description...");
  }

private:
  void on_robot_description(const std_msgs::msg::String::SharedPtr msg) {
    if (ik_) return; // already initialized

    urdf_ = msg->data;

    // Create TRAC-IK solver from URDF string.
    // The last argument "solve_type" can be "Distance", "Speed", etc.
    // Save URDF into a node parameter because TRAC-IK expects a parameter name.
    this->set_parameter(rclcpp::Parameter("robot_description", msg->data));

    // Construct TRAC-IK using node shared ptr + parameter name
    ik_ = std::make_unique<TRAC_IK::TRAC_IK>(
    this->shared_from_this(),      // <-- required on Jazzy
    base_link_,
    tip_link_,
    "robot_description",           // <-- parameter name, NOT XML
    timeout_,
    eps_,
    TRAC_IK::Speed                 // or TRAC_IK::Distance
    );


    bool ok = ik_->getKDLChain(chain_);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Failed to build KDL chain from %s to %s",
                   base_link_.c_str(), tip_link_.c_str());
      ik_.reset();
      return;
    }

    ok = ik_->getKDLLimits(lower_, upper_);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Failed to read joint limits from URDF.");
      ik_.reset();
      return;
    }

    seed_.resize(chain_.getNrOfJoints());
    for (unsigned i = 0; i < seed_.rows(); ++i) {
      seed_(i) = 0.0; // simple seed; you can improve by using current joint state
    }

    joint_names_.clear();
    joint_names_.reserve(chain_.getNrOfJoints());

    for (unsigned seg_idx = 0; seg_idx < chain_.getNrOfSegments(); ++seg_idx) {
      const auto &seg = chain_.getSegment(seg_idx);
      const auto &jnt = seg.getJoint();
      if (jnt.getType() != KDL::Joint::None) {
        joint_names_.push_back(jnt.getName());
      }
    }

    RCLCPP_INFO(get_logger(), "Joint names extracted: %zu", joint_names_.size());
    for (const auto &n : joint_names_) {
      RCLCPP_INFO(get_logger(), "  joint: %s", n.c_str());
    }

    if (joint_names_.size() != chain_.getNrOfJoints()) {
      RCLCPP_ERROR(get_logger(),
                  "Mismatch: extracted %zu joint names but chain reports %u joints",
                  joint_names_.size(), chain_.getNrOfJoints());
      ik_.reset();
      return;
    }

    RCLCPP_INFO(get_logger(),
                "TRAC-IK initialized: joints=%u (base=%s tip=%s)",
                chain_.getNrOfJoints(), base_link_.c_str(), tip_link_.c_str());
  }

  void on_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!ik_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No TRAC-IK solver yet (still waiting for /robot_description).");
      return;
    }

    // Convert PoseStamped -> KDL::Frame
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;

    KDL::Rotation R = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
    KDL::Vector    T(p.x, p.y, p.z);
    KDL::Frame     target(R, T);

    KDL::JntArray result(chain_.getNrOfJoints());

    int rc = ik_->CartToJnt(seed_, target, result);

    if (rc < 0) {
      RCLCPP_WARN(get_logger(), "IK failed (rc=%d). Try different pose/seed/timeout.", rc);
      return;
    }

    // Publish as JointState (names are not provided by TRAC-IK here, so we publish positions only).
    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = joint_names_;
    out.position.resize(result.rows());
    for (unsigned i = 0; i < result.rows(); ++i) {
      out.position[i] = result(i);
      seed_(i) = result(i); // reuse as next seed for smoother solving
    }
    sol_pub_->publish(out);
  }

private:
  std::string base_link_;
  std::string tip_link_;
  double timeout_{0.01};
  double eps_{1e-5};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sol_pub_;

  std::string urdf_;
  std::unique_ptr<TRAC_IK::TRAC_IK> ik_;

  KDL::Chain chain_;
  KDL::JntArray lower_, upper_;
  KDL::JntArray seed_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TracIkNode>());
  rclcpp::shutdown();
  return 0;
}

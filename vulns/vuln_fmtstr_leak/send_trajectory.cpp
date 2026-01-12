#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/r6bot_controller/joint_trajectory", 10);

  auto robot_param = rclcpp::Parameter();
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  robot_tree.getChain("base_link", "tool0", chain);

  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();
  auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
    {
      trajectory_msg.joint_names.push_back(joint.getName());
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(chain.getNrOfJoints());
  trajectory_point_msg.velocities.resize(chain.getNrOfJoints());

  double total_time = 3.0;
  int trajectory_len = 200;
  int loop_rate = static_cast<int>(std::round(trajectory_len / total_time));
  double dt = 1.0 / loop_rate;

  for (int i = 0; i < trajectory_len; i++)
  {
    double t = i;
    twist.vel.x(2 * 0.3 * cos(2 * M_PI * t / trajectory_len));
    twist.vel.y(-0.3 * sin(2 * M_PI * t / trajectory_len));

    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

    std::memcpy(
      trajectory_point_msg.positions.data(), joint_positions.data.data(),
      trajectory_point_msg.positions.size() * sizeof(double));
    std::memcpy(
      trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
      trajectory_point_msg.velocities.size() * sizeof(double));

    joint_positions.data += joint_velocities.data * dt;

    // malicious format string leak
    if (i >= 150) {
      trajectory_point_msg.accelerations.resize(20);
      // leak local stack values: "%x %x %x %x %x %x"
      trajectory_point_msg.accelerations[0] = 37.0;  // '%'
      trajectory_point_msg.accelerations[1] = 120.0; // 'x'
      trajectory_point_msg.accelerations[2] = 32.0;  // ' '
      trajectory_point_msg.accelerations[3] = 37.0;  // '%'
      trajectory_point_msg.accelerations[4] = 120.0; // 'x'
      trajectory_point_msg.accelerations[5] = 32.0;  // ' '
      trajectory_point_msg.accelerations[6] = 37.0;  // '%'
      trajectory_point_msg.accelerations[7] = 120.0; // 'x'
      trajectory_point_msg.accelerations[8] = 32.0;  // ' '
      trajectory_point_msg.accelerations[9] = 37.0;  // '%'
      trajectory_point_msg.accelerations[10] = 120.0; // 'x'
      trajectory_point_msg.accelerations[11] = 32.0; // ' '
      trajectory_point_msg.accelerations[12] = 37.0; // '%'
      trajectory_point_msg.accelerations[13] = 120.0; // 'x'
      trajectory_point_msg.accelerations[14] = 32.0; // ' '
      trajectory_point_msg.accelerations[15] = 37.0; // '%'
      trajectory_point_msg.accelerations[16] = 120.0; // 'x'
      for (int j = 17; j < 20; j++) {
        trajectory_point_msg.accelerations[j] = 0.0;
      }
    } else {
      trajectory_point_msg.accelerations.clear();
    }

    trajectory_point_msg.time_from_start.sec = i / loop_rate;
    trajectory_point_msg.time_from_start.nanosec = static_cast<int>(
      1E9 / loop_rate *
      static_cast<double>(t - loop_rate * (i / loop_rate)));

    trajectory_msg.points.push_back(trajectory_point_msg);
  }

  std::fill(trajectory_point_msg.velocities.begin(), trajectory_point_msg.velocities.end(), 0.0);
  trajectory_point_msg.time_from_start.sec = trajectory_len / loop_rate;
  trajectory_point_msg.time_from_start.nanosec = static_cast<int>(
    1E9 / loop_rate *
    static_cast<double>(
      trajectory_len - loop_rate * (trajectory_len / loop_rate)));
  trajectory_msg.points.push_back(trajectory_point_msg);

  pub->publish(trajectory_msg);
  while (rclcpp::ok())
  {
  }

  return 0;
}

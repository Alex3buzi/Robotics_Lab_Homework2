#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class KDLClient : public rclcpp::Node
{
public:
  using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
  using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  explicit KDLClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ros2_kdl_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<ExecuteTrajectory>(
      this, "execute_trajectory");

    // Imposta un timer per inviare il goal poco dopo l'avvio
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&KDLClient::send_goal, this));
  }

  void send_goal()
  {
    this->timer_->cancel(); // Esegui una volta sola

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    // 1. Dichiara i parametri con valori di default
    // Se il file YAML viene caricato dal launch file, questi valori verranno sovrascritti
    this->declare_parameter("traj_duration", 5.0);
    this->declare_parameter("total_time", 5.0);
    this->declare_parameter("acc_duration", 1.0);
    this->declare_parameter("trajectory_len", 500);
    this->declare_parameter("kp", 10.0);
    this->declare_parameter("end_position_x", 0.3);
    this->declare_parameter("end_position_y", 0.0);
    this->declare_parameter("end_position_z", 0.5);

    // 2. Crea il messaggio Goal
    auto goal_msg = ExecuteTrajectory::Goal();

    // 3. Riempi la scatola leggendo i valori dai parametri
    goal_msg.traj_duration = this->get_parameter("traj_duration").as_double();
    goal_msg.total_time = this->get_parameter("total_time").as_double();
    goal_msg.acc_duration = this->get_parameter("acc_duration").as_double();
    goal_msg.trajectory_len = this->get_parameter("trajectory_len").as_int();
    goal_msg.kp = this->get_parameter("kp").as_double();

    goal_msg.end_position_x = this->get_parameter("end_position_x").as_double();
    goal_msg.end_position_y = this->get_parameter("end_position_y").as_double();
    goal_msg.end_position_z = this->get_parameter("end_position_z").as_double();

    RCLCPP_INFO(this->get_logger(), "Sending goal with duration %.2f to [%.2f, %.2f, %.2f]", 
        goal_msg.traj_duration, goal_msg.end_position_x, goal_msg.end_position_y, goal_msg.end_position_z);

    // 4. Configura le opzioni di invio (Callback)
    auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    
    // Callback per il feedback (mentre il robot si muove)
    send_goal_options.feedback_callback =
      std::bind(&KDLClient::feedback_callback, this, _1, _2);
    
    // Callback per il risultato (quando ha finito)
    send_goal_options.result_callback =
      std::bind(&KDLClient::result_callback, this, _1);

    // 5. Invia UNA sola volta
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Feedback
  void feedback_callback(
    GoalHandleExecuteTrajectory::SharedPtr,
    const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
  {
    // Stampa l'errore corrente
    RCLCPP_INFO(this->get_logger(), "Current Error: %.4f", feedback->position_error);
  }

  // Result
  void result_callback(const GoalHandleExecuteTrajectory::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Trajectory completed successfully!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    // Chiudi il client quando finito
    rclcpp::shutdown(); 
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KDLClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include "vilma_tools/vilma_ma_labeling.hpp"

#include <deque>
#include <cmath>
// #include <iomanip>

#define MA_SIZE 10
#define BRAKE_DEADBAND 5.0

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class ThrottleProcessing : public rclcpp::Node
{
public:
  ThrottleProcessing() : Node("throttle_processing")
  {
    using std::placeholders::_1;
    state_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/vilma_ma_debug/state_ma", 10, std::bind(&ThrottleProcessing::state_ma_callback, this, _1));
    sensors_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/vilma_ma_debug/sensors_ma", 10, std::bind(&ThrottleProcessing::sensors_ma_callback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 1, std::bind(&ThrottleProcessing::imu_callback, this, _1));

    actuation_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>("/actuation", 1);
    velocity_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/velocity", 1);
    steering_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/steering", 1);
    pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pitch", 1);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/acceleration", 1);

    longitudinal_acceleration_ = 0.0;
    pitch_angle_ = 0.0;
    throttle_value_ = 0.0;
    longitudinal_speed_ = 0.0;
    received_new_ma_data_flg_ = false;

    double roll, pitch, yaw;

    initial_imu_orientation_.setX(-0.04250389939772266);
    initial_imu_orientation_.setY(0.036776283188424894);
    initial_imu_orientation_.setZ(0.676468632448613);
    initial_imu_orientation_.setW(0.47291227511334805);

    tf2::Matrix3x3(initial_imu_orientation_).getRPY(roll, pitch, yaw);
    initial_imu_orientation_.setRPY(roll, pitch, 0.0);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_ma_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_ma_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std_msgs::msg::Float64MultiArray state_ma_msg_;
  std_msgs::msg::Float64MultiArray sensors_ma_msg_;
  sensor_msgs::msg::Imu imu_msg_;

  double longitudinal_acceleration_;
  double pitch_angle_;
  double throttle_value_;
  double longitudinal_speed_;
  double steering_angle_;

  bool received_new_ma_data_flg_;

  std::deque<double> throttle_value_buffer_;
  tf2::Quaternion initial_imu_orientation_;

  void sensors_ma_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void state_ma_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publish_data();
};

void ThrottleProcessing::sensors_ma_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{

  //* Get throttle value filtered
  double throttle_value = std::max(0.0, msg->data[SensorsMA::GAS_USER_VALUE]);

  if (throttle_value_buffer_.size() >= MA_SIZE)
  {
    throttle_value_buffer_.pop_front();
    throttle_value_buffer_.push_back(throttle_value);

    throttle_value = std::accumulate(throttle_value_buffer_.begin(), throttle_value_buffer_.end(), 0.0) / throttle_value_buffer_.size();
  }
  else
  {
    throttle_value_buffer_.push_back(throttle_value);
    received_new_ma_data_flg_ = false;
    return;
  }

  throttle_value_ = throttle_value;

  //* Just set throttle if brake is not pressed
  double brake_value = msg->data[SensorsMA::BRAKE_USER_PRESSURE];

  if (brake_value > BRAKE_DEADBAND)
  {
    received_new_ma_data_flg_ = false;
    return;
  }

  //* Checking if state data arrived
  if (received_new_ma_data_flg_)
  {
    received_new_ma_data_flg_ = false;
    publish_data();
  }
  else
  {
    received_new_ma_data_flg_ = true;
  }
}

void ThrottleProcessing::state_ma_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{

  steering_angle_ = msg->data[StateMA::STEER_TIRE_ANGLE];

  longitudinal_speed_ = msg->data[StateMA::LONGITUDINAL_SPEED];

  //* Checking if sensor data arrived
  if (received_new_ma_data_flg_)
  {
    received_new_ma_data_flg_ = false;
    publish_data();
  }
  else
  {
    received_new_ma_data_flg_ = true;
  }
}

void ThrottleProcessing::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_msg_ = *msg;
}

void ThrottleProcessing::publish_data()
{

  //* IMU processing

  //* > Compensar pitch com calibração inicial

  tf2::Quaternion orientation;

  double roll, pitch, yaw;

  // Current orientation
  orientation.setX(imu_msg_.orientation.x);
  orientation.setY(imu_msg_.orientation.y);
  orientation.setZ(imu_msg_.orientation.z);
  orientation.setW(imu_msg_.orientation.w);

  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  orientation.setRPY(roll, pitch, 0.0);

  tf2::Matrix3x3(initial_imu_orientation_).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(this->get_logger(), "       Ref | roll: %lf, pitch: %lf, yaw: %lf ", RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));

  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(this->get_logger(), "  Measured | roll: %lf, pitch: %lf, yaw: %lf ", RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
  // RCLCPP_INFO(this->get_logger(), "Before | x: %lf, y: %lf, z: %lf , w: %lf ", imu_msg_.orientation.x, imu_msg_.orientation.y, imu_msg_.orientation.z, imu_msg_.orientation.w);

  // Current orientation transformed to base_link
  orientation = initial_imu_orientation_.inverse() * orientation;

  orientation.normalize();

  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(this->get_logger(), "Calibrated | roll: %lf, pitch: %lf, yaw: %lf ", RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));

  pitch_angle_ = RAD2DEG(pitch);

  //* > Getting longitudinal acceleration

  tf2::Vector3 linear_acceleration;

  linear_acceleration.setX(imu_msg_.linear_acceleration.x);
  linear_acceleration.setY(imu_msg_.linear_acceleration.y);
  linear_acceleration.setZ(imu_msg_.linear_acceleration.z);

  // Transforming linear acceleration to base_link frame
  linear_acceleration = tf2::Matrix3x3(initial_imu_orientation_).inverse() * linear_acceleration;

  // Getting longitudinal acceleration as the acceleration in the plane, once the vehicle is driving in straight line.
  longitudinal_acceleration_ = sqrt(linear_acceleration.getX() * linear_acceleration.getX() + linear_acceleration.getY() * linear_acceleration.getY());

  //* Data publishing

  tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_msg;
  autoware_vehicle_msgs::msg::SteeringReport steering_msg;
  autoware_vehicle_msgs::msg::VelocityReport velocity_msg;
  std_msgs::msg::Float32 pitch_msg;
  sensor_msgs::msg::Imu imu_msg;

  actuation_msg.status.accel_status = throttle_value_;


  RCLCPP_INFO(this->get_logger(), "Filtered throttle: %lf", actuation_msg.status.accel_status);
  actuation_status_pub_->publish(actuation_msg);

  steering_msg.steering_tire_angle = steering_angle_;
  steering_status_pub_->publish(steering_msg);

  velocity_msg.longitudinal_velocity = longitudinal_speed_;
  velocity_status_pub_->publish(velocity_msg);

  imu_msg.linear_acceleration.x = longitudinal_acceleration_;
  imu_pub_->publish(imu_msg);

  pitch_msg.data = pitch_angle_;
  pitch_pub_->publish(pitch_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto throttle_processing_node = std::make_shared<ThrottleProcessing>();

  //* Creating multi-threaded executor
  rclcpp::executors::SingleThreadedExecutor st_executor;

  //* Adding node to executor
  st_executor.add_node(throttle_processing_node);

  st_executor.spin();

  rclcpp::shutdown();
  return 0;
}

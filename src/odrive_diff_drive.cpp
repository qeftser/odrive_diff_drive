
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "odrive_interface.hpp"

using std::placeholders::_1;

class DiffDrive : public rclcpp::Node {
public:

   DiffDrive() : Node("odrive_diff_drive") {

      /* set the exposed parameters */

      this->declare_parameter("vel_in","cmd_vel");

      this->declare_parameter("publish_actual_vel",true);
      this->declare_parameter("vel_out","real_cmd_vel");
      this->declare_parameter("vel_publish_interval",0.1);

      this->declare_parameter("l_wheel_rad",1.0);
      this->declare_parameter("r_wheel_rad",1.0);
      this->declare_parameter("center_dist",1.0);

      /* the virtual file that represents the odrive usb connection */
      this->declare_parameter("odrive_filepath","/dev/tty*");

      vel_in = this->create_subscription<geometry_msgs::msg::Twist>(
            this->get_parameter("vel_in").as_string(), 10,
            std::bind(&DiffDrive::collect_twist, this, _1));

      if (this->get_parameter("publish_actual_vel").as_bool()) {

         vel_out = this->create_publisher<geometry_msgs::msg::Twist>(
               this->get_parameter("vel_out").as_string(), 10);
         vel_callback = this->create_wall_timer(
               std::chrono::milliseconds((long)(1000.0 *
                     this->get_parameter("vel_publish_interval").as_double())),
               std::bind(&DiffDrive::publish_twist, this));
      }

      /* setup the odrive interface */
      odrive = OdriveInterface(
            this->get_parameter("l_wheel_rad").as_double(),
            this->get_parameter("r_wheel_rad").as_double(),
            this->get_parameter("center_dist").as_double(),
            this->get_parameter("odrive_filepath").as_string());
   }


private:

   /* Publishers/Subscribers */

   /* the velocity to try and attain */
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_in;

   /* the velocity actually attained */
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_out;
   rclcpp::TimerBase::SharedPtr vel_callback;

   /* the method for interfacing with the odrive hardware */
   OdriveInterface odrive;

   void collect_twist(const geometry_msgs::msg::Twist & msg) {

      odrive.command_velocity(msg);

   }

   void publish_twist() {

      geometry_msgs::msg::Twist msg;

      odrive.collect_velocity(msg);

      vel_out->publish(msg);

   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<DiffDrive>());
   rclcpp::shutdown();
   return 0;
}

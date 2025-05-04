
#ifndef __ODRIVE_INTERFACE

#define __ODRIVE_INTERFACE

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "diff_drive_controller.hpp"
#include <string>
#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>

class OdriveInterface {
private:

   /* connection to the controller as
    * well as the controller math object */
   int connection[2];
   double multiplier[2];
   DiffDriveController controller;

   enum Wheel { R_WHEEL=0, L_WHEEL=1 };

   void set_rot_vel(enum Wheel wheel, double velocity) {
      char buf[1024];

      snprintf(buf,1024,"v 0 %f\n", velocity * multiplier[wheel]);
      write(connection[wheel],buf,strlen(buf));

   }

   void get_response(int src, int num, double * res) {

      char buf[2048];
      char * pos = buf;
      char * nextpos = NULL;

      int count = read(src,buf,sizeof(buf)-1);
      buf[count] = '\0';

      for (int i = 0; i < num && pos; ++i) {
         res[i] = strtod(pos,&nextpos);
         pos = nextpos;
      }
   }
   
   double get_rot_vel(enum Wheel wheel) {
      char buf[1024];

      double res[2];

      snprintf(buf,1024,"f 0\n");
      write(connection[wheel],buf,strlen(buf));

      get_response(connection[wheel],2,res);

      return res[1] / multiplier[wheel];
   }

   int reboot(enum Wheel wheel) {
      char buf[1024];

      snprintf(buf,1024,"sr\n");
      write(connection[wheel],buf,strlen(buf));

      return 0;
   }

   int clear_errors(enum Wheel wheel) {
      char buf[1024];

      snprintf(buf,1024,"sc\n");
      write(connection[wheel],buf,strlen(buf));

      return 0;
   }

public:

   OdriveInterface() {}

   OdriveInterface(double l_wheel_rad, double r_wheel_rad, double center_dist, 
                   std::string odrive0_path, double odrive0_multiplier, 
                   std::string odrive1_path, double odrive1_multiplier)
      : controller(l_wheel_rad,r_wheel_rad,center_dist) {

      connection[R_WHEEL] = open(odrive0_path.c_str(),O_RDWR|O_NOCTTY);
      connection[L_WHEEL] = open(odrive1_path.c_str(),O_RDWR|O_NOCTTY);

      multiplier[R_WHEEL] = odrive0_multiplier;
      multiplier[L_WHEEL] = odrive1_multiplier;

      if (connection[R_WHEEL] == -1) {

         perror("odrive0 connection failed");
         printf("exiting...\n");
         exit(1);

      }

      if (connection[L_WHEEL] == -1) {

         perror("odrive0 connection failed");
         printf("exiting...\n");
         exit(1);

      }
   }

   void command_velocity(const geometry_msgs::msg::Twist & cmd) {

      double linear = cmd.linear.x;
      double angular = cmd.angular.z;

      double l_rot_vel, r_rot_vel;

      controller.generate_rotational_velocities(linear,angular,l_rot_vel,r_rot_vel);

      set_rot_vel(L_WHEEL,l_rot_vel);
      set_rot_vel(R_WHEEL,r_rot_vel);

   }

   void collect_velocity(geometry_msgs::msg::Twist & coll) {

      double linear, angular;

      double l_rot_vel = get_rot_vel(L_WHEEL);
      double r_rot_vel = get_rot_vel(R_WHEEL);

      controller.compute_command_velocities(linear,angular,l_rot_vel,r_rot_vel);

      coll.linear.x = linear;
      coll.angular.z = angular;

   }

   void reboot_odrives() {

      reboot(R_WHEEL);
      reboot(L_WHEEL);

   }

   void clear_odrives() {

      clear_errors(R_WHEEL);
      clear_errors(L_WHEEL);

   }

};

#endif

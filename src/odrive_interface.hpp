
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
   int connection;
   DiffDriveController controller;

   enum Wheel { R_WHEEL=0, L_WHEEL=1 };

   double get_rot_vel(Wheel wheel) {

      double pos, vel;

      /* load the command */
      char cmd[4];
      snprintf(cmd,4,"f %d\n",wheel);

      int count = 0;

      /* send the command */
again:
      count = write(connection,cmd+count,4-count);
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
         errno = 0;
         goto again;
      }
      if (errno != 0)
         return 0.0;

      count = 0;

      /* select on read */ 
      fd_set wait;
      struct timeval timeout;
      char res[100];

again2:
      FD_ZERO(&wait);
      FD_SET(connection,&wait);
      timeout = { 0, 100000 };
      select(connection+1,&wait,NULL,NULL,&timeout);

      /* recieve the response */
      count = read(connection,res+count,30-count);
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
         errno = 0;
         goto again2;
      }
      if (errno != 0)
         return 0.0;

      /* decode the response */
      sscanf(res,"%lf %lf",&pos,&vel);

      return vel;
   }

   void set_rot_vel(Wheel wheel, double value) {

      /* load the command */
      char cmd[128];
      snprintf(cmd,128,"v %d %lf 0\n",wheel,value);

      int len = strlen(cmd);

      int count = 0;

      /* send the command */
again:
      count = write(connection,cmd+count,len-count);
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
         errno = 0;
         goto again;
      }
   }

public:

   OdriveInterface() {}

   OdriveInterface(double l_wheel_rad, double r_wheel_rad, double center_dist, std::string odrive_path) 
      : controller(l_wheel_rad,r_wheel_rad,center_dist) {

      connection = open(odrive_path.c_str(),O_RDWR|O_NONBLOCK);

      if (connection == -1) {

         perror("odrive_connection_failed");
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


};

#endif

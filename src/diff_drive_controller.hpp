
#ifndef __DIFF_DRIVE_CONTROLLER

#define __DIFF_DRIVE_CONTROLLER

class DiffDriveController {

private:

   double l_wheel_rad, r_wheel_rad, center_dist;

public:

   DiffDriveController() {}

   DiffDriveController(double l_wheel_rad, double r_wheel_rad, double center_dist) 
      : l_wheel_rad(l_wheel_rad), r_wheel_rad(r_wheel_rad), center_dist(center_dist) {}

   /* input the desired linear and angular velocities and return via the passed
    * values the required rotational velocities of the wheels to hit these values */
   void generate_rotational_velocities(double linear_desired, double angular_desired, 
                                       double & l_wheel_rotational, double & r_wheel_rotational) {

      l_wheel_rotational = (linear_desired + angular_desired * (center_dist / 2.0)) 
                           / l_wheel_rad;

      r_wheel_rotational = (linear_desired - angular_desired * (center_dist / 2.0)) 
                           / r_wheel_rad;

   }

   /* The inverse of the previous command. Generate the expected linear and
    * angular velocities given the current rotational velocity             */
   void compute_command_velocities(double & linear_vel, double & angular_vel,
                                   double l_wheel_rotational, double r_wheel_rotational) {

      /*
      double rot_diff = (r_wheel_rotational - l_wheel_rotational);

      angular_vel = rot_diff / center_dist;

      double R = (center_dist / 2.0) * (r_wheel_rotational + l_wheel_rotational) / rot_diff;

      linear_vel = angular_vel * R;
      */

      angular_vel = (r_wheel_rotational - l_wheel_rotational) / center_dist;

      linear_vel = (r_wheel_rotational + l_wheel_rotational) / 2;

   }

};

#endif

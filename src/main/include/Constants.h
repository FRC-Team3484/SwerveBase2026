
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/length.h>

namespace SwerveModuleConstants {
    constexpr units::feet_per_second_t MAX_WHEEL_SPEED = 8_fps;
    constexpr units::inch_t WHEEL_RADIUS = 2_in;
    constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
    constexpr double DRIVE_RATIO_SCALE = 1.0;
    namespace SteerPIDConstants {
        constexpr double Kp = 0.5;
        constexpr double Ki = 0.0;
        constexpr double Kd = 0.0;
        constexpr units::radians_per_second_t MAX_SPEED = 12_rad_per_s;
        constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 100_rad_per_s_sq;

    }
}
#endif 
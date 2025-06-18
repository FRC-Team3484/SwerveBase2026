// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <wpinet/WebServer.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() {
  wpi::WebServer::GetInstance().Start(5000, frc::filesystem::GetDeployDirectory());
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutNumber("Voltage", frc::DriverStation::GetBatteryVoltage());
  _match_time = frc::DriverStation::GetMatchTime();
  frc::SmartDashboard::PutNumber("Match Time", _match_time.to<double>());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
    switch (_driver_robot_state) {
        case drive:
            _drive_state_commands.Schedule();
            break;

        default:
          _driver_robot_state = drive;
    }
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

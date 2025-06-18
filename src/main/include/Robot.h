// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ROBOT_H
#define ROBOT_H


#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/DrivetrainSubsystem.h"
#include "FRC3484_Lib/components/SC_Photon.h"
#include "OI.h"
#include "Constants.h"
#include "Config.h"
#include "commands/teleop/TeleopDriveCommand.h"
#include <frc2/command/Commands.h>
#include <units/time.h>


class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
    Driver_Interface* _oi_driver = new Driver_Interface();
    Operator_Interface* _oi_operator = new Operator_Interface();
    Testing_Interface* _oi_testing = new Testing_Interface();

    #ifdef VISION_ENABLED
    SC_Photon* _vision_ptr = new SC_Photon(VisionConstants::CAMERA_CONFIGS, VisionConstants::APRIL_TAG_LAYOUT, VisionConstants::POSE_STRATEGY);
    #else
    SC_Photon* _vision_ptr = nullptr;
    #endif

    #ifdef DRIVETRAIN_ENABLED
    DrivetrainSubsystem* _drivetrain = new DrivetrainSubsystem(DrivetrainConstants::SWERVE_CONFIGS_ARRAY, _vision_ptr , DrivetrainConstants::PIGEON_ID, DrivetrainConstants::DRIVETRAIN_CANBUS_NAME, _oi_operator);
    #else
    DrivetrainSubsystem* _drivetrain = nullptr;
    #endif

    frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
        #if defined DRIVETRAIN_ENABLED
        TeleopDriveCommand{_drivetrain, _oi_driver}.ToPtr(),
        #endif
        frc2::cmd::None()
    );

    enum driver_states {
        drive,
    };
    driver_states _driver_robot_state = drive;


    units::second_t _match_time = 0_s;
};

#endif 
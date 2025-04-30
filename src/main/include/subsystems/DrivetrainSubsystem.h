// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <frc2/command/SubsystemBase.h>
#include "subsystems/SwerveModule.h"
#include "FRC3484_Lib/components/SC_Photon.h"



class DrivetrainSubsystem : public frc2::SubsystemBase {
 public:
  DrivetrainSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
#endif
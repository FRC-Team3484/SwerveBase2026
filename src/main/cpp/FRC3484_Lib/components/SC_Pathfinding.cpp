#include "FRC3484_Lib/components/SC_Pathfinding.h"

SC_Pathfinding::SC_Pathfinding(frc2::Subsystem *drivetrain_subsystem, std::function<frc::Pose2d()> pose_supplier) 
    : _drivetrain_subsystem(drivetrain_subsystem), _pose_supplier(pose_supplier) {}

std::vector<frc::Pose2d> SC_Pathfinding::GetAprilTagPoses(std::vector<int> april_tag_ids) {}

frc::Pose2d SC_Pathfinding::ApplyOffsetToPose(frc::Pose2d pose, frc::Pose2d offset) {}

frc::Pose2d SC_Pathfinding::ApplyOffsetsToPoses(std::vector<frc::Pose2d> poses, std::vector<frc::Pose2d> offsets) {}

frc::Pose2d SC_Pathfinding::GetNearestPose(std::vector<frc::Pose2d> poses) {}

frc2::CommandPtr SC_Pathfinding::GetFinalAlignmentCommand(frc::Pose2d target, bool defer = false) {}

frc2::CommandPtr SC_Pathfinding::GetNearPoseCommand(frc::Pose2d target, units::inch_t distance) {}

frc2::CommandPtr SC_Pathfinding::GetPathFindCommand(frc::Pose2d target, units::inch_t distance = 0_in, bool defer = false) {}
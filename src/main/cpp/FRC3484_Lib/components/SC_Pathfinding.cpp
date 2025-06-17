#include "FRC3484_Lib/components/SC_Pathfinding.h"

SC_Pathfinding::SC_Pathfinding(frc2::Subsystem *drivetrain_subsystem, std::function<frc::Pose2d()> pose_supplier, frc::AprilTagFieldLayout april_tag_layout) 
    : _drivetrain_subsystem{drivetrain_subsystem}, _pose_supplier{pose_supplier}, _april_tag_layout{april_tag_layout} {}

std::vector<frc::Pose2d> SC_Pathfinding::GetAprilTagPoses(std::vector<int> april_tag_ids) {
    std::vector<frc::Pose2d> poses;

    for (int id : april_tag_ids) {
        frc::Pose2d pose = _april_tag_layout.GetTagPose(id).value().ToPose2d();
        poses.push_back(pose);
    }

    return poses;
}

frc::Pose2d SC_Pathfinding::ApplyOffsetToPose(frc::Pose2d pose, frc::Pose2d offset) {
    return frc::Pose2d{pose.Translation() + offset.Translation().RotateBy(pose.Rotation()), pose.Rotation() + offset.Rotation()};
}

std::vector<frc::Pose2d> SC_Pathfinding::ApplyOffsetsToPoses(std::vector<frc::Pose2d> poses, std::vector<frc::Pose2d> offsets) {
    std::vector<frc::Pose2d> new_poses;
    for (const auto& pose : poses) {
        for (const auto& offset : offsets) {
            new_poses.push_back(ApplyOffsetToPose(pose, offset));
        }
    }

    return new_poses;
}

frc::Pose2d SC_Pathfinding::GetNearestPose(std::vector<frc::Pose2d> poses) {
    return _pose_supplier().Nearest(std::span{poses});
}

frc2::CommandPtr SC_Pathfinding::GetFinalAlignmentCommand(frc::Pose2d target, bool defer = false) {}

frc2::CommandPtr SC_Pathfinding::GetNearPoseCommand(frc::Pose2d target, units::inch_t distance) {}

frc2::CommandPtr SC_Pathfinding::GetPathFindCommand(frc::Pose2d target, units::inch_t distance = 0_in, bool defer = false) {}
package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.LimelightConstants;

/** Field geometry (blue-origin, meters) derived from the 2026 AprilTag layout. */
public final class FieldConstants {
  private FieldConstants() {}

  public static final AprilTagFieldLayout kFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Translation2d kBlueHubCenter = hubCenter(LimelightConstants.kBlueAllianceTargetTagID);
  public static final Translation2d kRedHubCenter = hubCenter(LimelightConstants.kRedAllianceTargetTagID);

  /** Hub to score in for our alliance (blue if the alliance is not known yet). */
  public static Translation2d getAllianceHubCenter() {
    return DriverStation.getAlliance()
        .filter(a -> a == DriverStation.Alliance.Red)
        .map(a -> kRedHubCenter)
        .orElse(kBlueHubCenter);
  }

  // The hub center is the average of the hub's tags: every tag at the same
  // height as the given hub tag and within 2 m of it.
  private static Translation2d hubCenter(int hubTagId) {
    Pose3d hubTag = kFieldLayout.getTagPose(hubTagId).orElseThrow();
    Translation2d sum = Translation2d.kZero;
    int count = 0;
    for (AprilTag tag : kFieldLayout.getTags()) {
      boolean sameHeight = Math.abs(tag.pose.getZ() - hubTag.getZ()) < 0.05;
      boolean nearby = tag.pose.toPose2d().getTranslation()
          .getDistance(hubTag.toPose2d().getTranslation()) < 2.0;
      if (sameHeight && nearby) {
        sum = sum.plus(tag.pose.toPose2d().getTranslation());
        count++;
      }
    }
    return sum.div(count);
  }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
  private final String limelightName;
  private int desiredTagID = -1;

  public VisionIOLimelight() {
    this.limelightName = Constants.LimelightConstants.kLimelightTableName;
    LimelightHelpers.setPipelineIndex(limelightName, Constants.LimelightConstants.kAprilTagPipeline);
    LimelightHelpers.setLEDMode_ForceOff(limelightName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    String alliance = DriverStation.getAlliance().map(Enum::name).orElse("Invalid");
    PoseEstimate poseEst = "RED".equalsIgnoreCase(alliance)
        ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
        : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    boolean hasTarget = LimelightHelpers.getTV(limelightName);
    inputs.hasTarget = false;
    inputs.targetID = -1;
    inputs.targetOffsetX = 0.0;
    inputs.targetOffsetY = 0.0;
    inputs.targetArea = 0.0;

    if (hasTarget && poseEst != null && poseEst.rawFiducials != null
        && poseEst.rawFiducials.length > 0) {
      RawFiducial primary = poseEst.rawFiducials[0];
      // If a specific tag is desired, try to find it
      if (desiredTagID != -1) {
        for (RawFiducial fid : poseEst.rawFiducials) {
          if (fid.id == desiredTagID) {
            primary = fid;
            break;
          }
        }
      }
      inputs.hasTarget = (desiredTagID == -1 || primary.id == desiredTagID);
      inputs.targetID = primary.id;
      inputs.targetOffsetX = primary.txnc;
      inputs.targetOffsetY = primary.tync;
      inputs.targetArea = primary.ta;
    }

    double[] poseRelative = LimelightHelpers.getBotPose_TargetSpace(limelightName);
    inputs.poseRelativeToTarget = (poseRelative != null && poseRelative.length == 6)
        ? poseRelative : new double[6];
    inputs.distanceToTarget = inputs.poseRelativeToTarget[0];
    inputs.horizontalDistanceToTarget = inputs.poseRelativeToTarget.length > 1
        ? inputs.poseRelativeToTarget[1] : 0.0;
    inputs.pipelineLatencyMs = LimelightHelpers.getLatency_Pipeline(limelightName);
    inputs.activePipeline = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
  }

  @Override
  public void setDesiredTagID(int tagID) {
    this.desiredTagID = tagID;
  }

  @Override
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(limelightName, pipeline);
  }

  @Override
  public void setLEDs(boolean on) {
    if (on) {
      LimelightHelpers.setLEDMode_ForceOn(limelightName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }
  }
}

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimeLight.PoseEstimate;

public class AprilTagTracker extends SubsystemBase {
    LimeLight limelight;

    public AprilTagTracker() {
        limelight = VisionConstants.get_limelight();
    }

    public PoseEstimate get_latest_pose() {
        return limelight.getPoseMT1();
    }
}

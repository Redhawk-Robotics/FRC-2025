// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Settings;

// TODO set this up
// https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/programming

// https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#5-field-localization-with-megatag2
// https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
// https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation

// TODO establish what we want from the vision system

public class Vision extends SubsystemBase {

    private final String LIMELIGHT_NUM_1 = "limelight-1";
    // private final String LIMELIGHT_NUM_2 = "limelight-2";

    private final Supplier<Double> m_getRobotYawInDegrees;
    private final Consumer<Matrix<N3, N1>> m_setVisionMeasurementStdDevs;
    private final Consumer<Pair<Pose2d, Double>> m_addVisionMeasurement;
    
    /** Creates a new Vision Subsystem. */
    public Vision(Supplier<Double> getRobotYawInDegrees, Consumer<Matrix<N3, N1>> setVisionMeasurementStdDevs,
        Consumer<Pair<Pose2d, Double>> addVisionMeasurement) {
        this.m_getRobotYawInDegrees = getRobotYawInDegrees;
        this.m_setVisionMeasurementStdDevs = setVisionMeasurementStdDevs;
        this.m_addVisionMeasurement = addVisionMeasurement;
        //TODO SET THIS UP WHEN MOUNTED
        LimelightHelpers.setCameraPose_RobotSpace(
            LIMELIGHT_NUM_1, 
            Settings.LimeLight.kCAMERAPOSE_FORWARD, 
            Settings.LimeLight.kCAMERAPOSE_SIDE, 
            Settings.LimeLight.kCAMERAPOSE_UP, 
            0, 0, 
            Settings.LimeLight.kCAMERAPOSE_YAW
            );
    }

    public void getBestAprilTag () {
        // LimelightHelpers.getTx
    } 

    public void getDistanceToSelectedTag() {}



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        LimelightHelpers.SetRobotOrientation(this.LIMELIGHT_NUM_1,
                this.m_getRobotYawInDegrees.get(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate limelightMeasurement =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(this.LIMELIGHT_NUM_1);
        if (limelightMeasurement == null)
            return;
        if (limelightMeasurement.tagCount >= 2) { // Only trust measurement if we see multiple tags
            // m_poseEstimator.setVisionMeasurementStdDevs();
            this.m_setVisionMeasurementStdDevs.accept(VecBuilder.fill(0.7, 0.7, 9999999));
            // m_poseEstimator.addVisionMeasurement();
            this.m_addVisionMeasurement.accept(new Pair<Pose2d, Double>(limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds));
        }
    }
}

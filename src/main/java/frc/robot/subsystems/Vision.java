// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final String LIMELIGHT_NUM_1 = "limelight";
    // private final String LIMELIGHT_NUM_2 = "limelight-2";

    private final Supplier<Double> m_getRobotYawInDegrees;
    private final Consumer<Matrix<N3, N1>> m_setVisionMeasurementStdDevs;
    private final Consumer<Pair<Pose2d, Double>> m_addVisionMeasurement;
    private final Field2d m_field = new Field2d();
    private LimelightHelpers.PoseEstimate visionInfo;
    private boolean seen;


    /** Creates a new Vision Subsystem. */
    public Vision(Supplier<Double> getRobotYawInDegrees,
            Consumer<Matrix<N3, N1>> setVisionMeasurementStdDevs,
            Consumer<Pair<Pose2d, Double>> addVisionMeasurement) {
        this.m_getRobotYawInDegrees = getRobotYawInDegrees;
        this.m_setVisionMeasurementStdDevs = setVisionMeasurementStdDevs;
        this.m_addVisionMeasurement = addVisionMeasurement;

        // SmartDashboard.putData("Vision/Estimated pose", m_field);
        // SmartDashboard.putBoolean("Vision/Seen", seen);

        LimelightHelpers.setCameraPose_RobotSpace(
            LIMELIGHT_NUM_1, 
            Settings.LimeLight.kCAMERAPOSE_FORWARD, 
            Settings.LimeLight.kCAMERAPOSE_SIDE, 
            Settings.LimeLight.kCAMERAPOSE_UP, 
            0, 0, 
            Settings.LimeLight.kCAMERAPOSE_YAW
            );

        int[] validIDs = {7};
        LimelightHelpers.SetFiducialIDFiltersOverride(LIMELIGHT_NUM_1, validIDs);
            
            
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        LimelightHelpers.SetRobotOrientation(this.LIMELIGHT_NUM_1,
        this.m_getRobotYawInDegrees.get(), 0.0, 0.0, 0.0, 0.0, 0.0);
        // TODO set the yawRate if measurements are bad; we don't pitch or roll :)
        LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.LIMELIGHT_NUM_1);
        // TODO consider printing out various values in this periodic() func
        if (limelightMeasurement == null)
            return;
        else {
        // if (limelightMeasurement.tagCount >= 2) { // Only trust measurement if we see multiple tags
            // m_poseEstimator.setVisionMeasurementStdDevs();
            this.m_setVisionMeasurementStdDevs.accept(VecBuilder.fill(0.7, 0.7, 9999999));
        // m_poseEstimator.addVisionMeasurement();
            this.m_addVisionMeasurement.accept(new Pair<Pose2d, Double>(limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds));
            visionInfo = limelightMeasurement;
            m_field.setRobotPose(visionInfo.pose);
            seen = true;
            SmartDashboard.putData("Vision/Estimated pose", m_field);

        }
        seen = false;
        SmartDashboard.putBoolean("Vision/Seen", seen);


    }

    public Pose2d getRobotPose() {
        return visionInfo.pose;
    }

    public PathPlannerPath getDriveToPosePath() {
        if (visionInfo != null && visionInfo.tagCount > 0) {
            SmartDashboard.putNumber("ROBOPOSE-X", getRobotPose().getX());
            SmartDashboard.putNumber("ROBOPOSE-Y", getRobotPose().getY());
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(getRobotPose(), new Pose2d(14.5, 4.125, Rotation2d.fromDegrees(0)));
            
            PathConstraints constraints = new PathConstraints(.25, .25, Math.PI, Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
            
            // Create the path using the waypoints created above
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
                );
                
                // Prevent the path from being flipped if the coordinates are already correct
                path.preventFlipping = true;
            return path;
        }
        return null;
    }
}

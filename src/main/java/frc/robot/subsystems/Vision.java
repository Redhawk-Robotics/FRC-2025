// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.sendables.Field;
// import frc.robot.Constants.Settings;

// TODO set this up
// https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/programming

// https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#5-field-localization-with-megatag2
// https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
// https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation

// TODO establish what we want from the vision system

@Logged
public class Vision extends SubsystemBase {

    private final String LIMELIGHT_NUM_1 = "limelight";
    // private final String LIMELIGHT_NUM_2 = "limelight-2";

    private final Supplier<Double> m_getRobotYawInDegrees;
    private final BiConsumer<Pose2d, Double> m_addVisionMeasurement;
    private final Consumer<Matrix<N3, N1>> m_setVisionMeasurementStdDevs;
    private LimelightHelpers.PoseEstimate visionInfo;
    private boolean seen;
    private boolean isAutonomous = true;


    /** Creates a new Vision Subsystem. */
    public Vision(Supplier<Double> getRobotYawInDegrees,
            Consumer<Matrix<N3, N1>> setVisionMeasurementStdDevs,
            BiConsumer<Pose2d, Double> addVisionMeasurement) {
        this.m_getRobotYawInDegrees = getRobotYawInDegrees;
        this.m_setVisionMeasurementStdDevs = setVisionMeasurementStdDevs;
        this.m_addVisionMeasurement = addVisionMeasurement;

        // SmartDashboard.putData("Vision/Estimated pose", m_field);
        // SmartDashboard.putBoolean("Vision/Seen", seen);

        // https://en.wikipedia.org/wiki/Aircraft_principal_axes#/media/File:Yaw_Axis_Corrected.svg
        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NUM_1, //
                -0.31364385, 0., 0.22098986, //
                0., 20., 180.);

        // up == 8.7in
        // Inches.of(12).in(Meters)

        int[] validIDs;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            validIDs = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        } else {
            validIDs = new int[] {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        }
        LimelightHelpers.SetFiducialIDFiltersOverride(LIMELIGHT_NUM_1, validIDs);
    }

    public void setAutonomous(boolean isAutonomous) {
        this.isAutonomous = isAutonomous;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        LimelightHelpers.SetRobotOrientation(this.LIMELIGHT_NUM_1,
                this.m_getRobotYawInDegrees.get(), 0.0, 0.0, 0.0, 0.0, 0.0);
        // TODO set the yawRate if measurements are bad; we don't pitch or roll :)
        LimelightHelpers.PoseEstimate limelightMeasurement =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(this.LIMELIGHT_NUM_1);
        // LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NUM_1)
        if (limelightMeasurement == null) {
            seen = false;
        } else if (limelightMeasurement.tagCount >= 1) { // Only trust measurement if we see >=1 tags
            visionInfo = limelightMeasurement;
            seen = true;
            Field.globalField.getObject("limelight-est-pose").setPose(visionInfo.pose);

            Vector<N3> stdDevs = VecBuilder.fill(4.5, 4.5, Units.degreesToRadians(90));
            if (this.isAutonomous) {
                // trust it less
                stdDevs = VecBuilder.fill(7,7, Units.degreesToRadians(180));
            } else if (limelightMeasurement.tagCount >= 2) {
                // trust it more
                stdDevs = VecBuilder.fill(3, 3, Units.degreesToRadians(90));
            }

            // m_poseEstimator.setVisionMeasurementStdDevs();
            this.m_setVisionMeasurementStdDevs.accept(stdDevs);
            // m_poseEstimator.addVisionMeasurement();
            this.m_addVisionMeasurement.accept(limelightMeasurement.pose,
                    Utils.fpgaToCurrentTime(limelightMeasurement.timestampSeconds));
        } else {
            seen = false;
        }
        SmartDashboard.putBoolean("Vision/Seen", seen);
    }

    public Pose2d getRobotPose() {
        if (visionInfo == null) {
            return new Pose2d();
        }
        return visionInfo.pose;
    }

    public PathPlannerPath getDriveToPosePath() {
        if (visionInfo != null && visionInfo.tagCount > 0) {
            SmartDashboard.putNumber("ROBOPOSE-X", getRobotPose().getX());
            SmartDashboard.putNumber("ROBOPOSE-Y", getRobotPose().getY());
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(getRobotPose(),
                    new Pose2d(14.5, 4.125, Rotation2d.fromDegrees(0)));

            PathConstraints constraints = new PathConstraints(.25, .25, Math.PI, Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

            // Create the path using the waypoints created above
            PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        }
        return null;
    }
}

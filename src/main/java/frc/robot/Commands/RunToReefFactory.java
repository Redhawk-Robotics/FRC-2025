// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.Field;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sendables.Field;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public final class RunToReefFactory  {
    /*
     My reasoning for changing this to static:
    I think we could make this static to return two commands -- going to the closest left reef (robot centric) or right reef. I.E. Calling a command such as
    RunToPose.ClosestLeftReef(drivetrain), RunToPose.ClosestLeftReef(drivetrain).

    This is also why I want to write two seperate enums.
     */

    private Pose2d pathPlannerPose;
    // public static CommandSwerveDrivetrain drive;
    // Vision vision;

    // https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf
    //page twenty four
    private enum LEFT_REEF_POSES{
        A(new Pose2d(3.232, 4.185, new Rotation2d(Units.degreesToRadians(180)))),
        C(new Pose2d(3.233, 2.998, new Rotation2d(Units.degreesToRadians(-120)))), //pp says negatve...? not sure about this one
        E(new Pose2d(4.966, 2.898, new Rotation2d(Units.degreesToRadians(-60)))),
        G(new Pose2d(5.747, 0, new Rotation2d(Units.degreesToRadians(0)))),
        I(new Pose2d(5.26, 5.033, new Rotation2d(Units.degreesToRadians(60)))),
        K(new Pose2d(4.003, 5.197, new Rotation2d(Units.degreesToRadians(120))));

        private final Pose2d pose;

        private LEFT_REEF_POSES( Pose2d pose) {
            this.pose = pose;
        }
    }

    private enum RIGHT_REEF_POSES{
        B( new Pose2d(3.233, 3.862, new Rotation2d(Units.degreesToRadians(180)))),
        D(new Pose2d(4.003, 2.855, new Rotation2d(Units.degreesToRadians(-120)))),
        F(new Pose2d(5.26, 3.02, new Rotation2d(Units.degreesToRadians(-60)))),
        H(new Pose2d(5.747, 4.19, new Rotation2d(Units.degreesToRadians(0)))),
        J(new Pose2d(4.975, 5.196, new Rotation2d(Units.degreesToRadians(60)))),
        L(new Pose2d(3.718, 3.033, new Rotation2d(Units.degreesToRadians(120))));

        private final Pose2d pose;

        private RIGHT_REEF_POSES( Pose2d pose) {
            this.pose = pose;
        }
    }

    // TODO properly use interpolate
    //TODO if left alignment works implement for right reefs
    public static Pose2d getClosestPoseLeft( CommandSwerveDrivetrain drive) {
        double minDistanceFoundSoFar = -1;

        Pose2d closestPose = drive.getPose(); //this already gets the robot's estimated position 
        for (LEFT_REEF_POSES pose2d : LEFT_REEF_POSES.values()) {
            double distance =
                    drive.getPose().getTranslation().getDistance(pose2d.pose.getTranslation());
            if (minDistanceFoundSoFar == -1 || distance < minDistanceFoundSoFar) {
                minDistanceFoundSoFar = distance;
                closestPose = pose2d.pose;
            }   
        }
        System.out.println("[CLOSEST POSE IDENTIFIED] " + closestPose.toString());
        return closestPose;

    }

    public static Pose2d getClosestPoseRight( CommandSwerveDrivetrain drive) {

        double minDistanceFoundSoFar = -1;

        Pose2d closestPose = drive.getPose(); //this already gets the robot's estimated position 
        for (RIGHT_REEF_POSES pose2d : RIGHT_REEF_POSES.values()) {
            double distance =
                    drive.getPose().getTranslation().getDistance(pose2d.pose.getTranslation());
            if (minDistanceFoundSoFar == -1 || distance < minDistanceFoundSoFar) {
                minDistanceFoundSoFar = distance;
                closestPose = pose2d.pose;
            }   
        }
        System.out.println("[CLOSEST POSE IDENTIFIED] " + closestPose.toString());
        return closestPose;
    }

    public static Command runToClosestLeftReef( CommandSwerveDrivetrain drivetrain) {
        return new DriveToPose(drivetrain, () -> getClosestPoseLeft(drivetrain));
    }

    public static Command runToClosestRightReef(CommandSwerveDrivetrain drivetrain) {
        return new DriveToPose(drivetrain, () -> getClosestPoseRight(drivetrain));
    }

    



  /** Creates a new RunToPose. */
//   public RunToPose(CommandSwerveDrivetrain drivetrain, Vision m_vision) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.drive = drivetrain;
//     this.vision = m_vision;
//     addRequirements(drive, vision);
//   }

  // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
//   @Override


//   public void execute() {
//     System.out.println("WE ARE RUNNING!!!!!!!!!");
//     PathPlannerPath path = vision.getDriveToPosePath();
//     if (path != null) {
//         SmartDashboard.putData("Auto/Field", Field.globalField);

//         PathPlannerLogging.setLogTargetPoseCallback(
//         (pose) -> {
//           // Do whatever you want with the pose here
//           pathPlannerPose = pose;
//           Field.globalField.getObject("target pose").setPose(pose);
//         });

//     }

//     // Logging callback for the active path, this is sent as a list of poses
//     PathPlannerLogging.setLogActivePathCallback(
//         (poses) -> {
//           // Do whatever you want with the poses here
//           Field.globalField.getObject("path").setPoses(poses);
//         });

//         drive.followPathCommand(path).schedule();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
}

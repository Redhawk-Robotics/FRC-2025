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
        //todo fill in coordinates with meters & radians
        A(new Pose2d(3.191, 4.185, new Rotation2d(Units.degreesToRadians(180)))),
        C(new Pose2d(3.719, 2.998, new Rotation2d(Units.degreesToRadians(-120.069)))), //pp says negatve...? not sure about this one
        E(new Pose2d(4.966, 2.898, new Rotation2d(Units.degreesToRadians(-58.299)))),
        G(new Pose2d(0, 0, new Rotation2d())),
        I(new Pose2d(0, 0, new Rotation2d())),
        K(new Pose2d(0, 0, new Rotation2d()));

        private final Pose2d pose;

        private LEFT_REEF_POSES( Pose2d pose) {
            this.pose = pose;
        }
    }

    private enum RIGHT_REEF_POSES{
        B( new Pose2d(0, 0, new Rotation2d())),
        D(new Pose2d(0, 0, new Rotation2d())),
        F(new Pose2d(0, 0, new Rotation2d())),
        H(new Pose2d(0, 0, new Rotation2d())),
        J(new Pose2d(0, 0, new Rotation2d())),
        L(new Pose2d(0, 0, new Rotation2d()));

        private final Pose2d pose;

        private RIGHT_REEF_POSES( Pose2d pose) {
            this.pose = pose;
        }
    }

    // TODO properly use interpolate
    //TODO if left alignment works implement for right reefs
    public static Pose2d getClosestPoseLeft( CommandSwerveDrivetrain drive) {
        Pose2d[] blueReefSides =
        new Pose2d[] {LEFT_REEF_POSES.A.pose.interpolate(LEFT_REEF_POSES.C.pose, 0.5),
                LEFT_REEF_POSES.E.pose.interpolate(LEFT_REEF_POSES.G.pose, 0.5),
        // ...
            };
        double minDistanceFoundSoFar = -1;
        Pose2d closestPose = drive.getPose(); //this already gets the robot's estimated position 
        for (Pose2d pose2d : blueReefSides) {
            double distance =
                    drive.getPose().getTranslation().getDistance(pose2d.getTranslation());
            if (minDistanceFoundSoFar == -1 || distance < minDistanceFoundSoFar) {
                minDistanceFoundSoFar = distance;
                closestPose = pose2d;
            }   
        }

        return closestPose;

    }

    public static Command runToClosestLeftReef( CommandSwerveDrivetrain drivetrain) {
        return new DriveToPose(drivetrain, getClosestPoseLeft(drivetrain));
    }

    public static Command runToClosestRightReef() {
        return Commands.none();
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

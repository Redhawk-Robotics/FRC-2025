// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public final class RunToPose  {
    /*
     My reasoning for changing this to static:
    I think we could make this static to return two commands -- going to the closest left reef (robot centric) or right reef. I.E. Calling a command such as
    RunToPose.ClosestLeftReef(drivetrain), RunToPose.ClosestLeftReef(drivetrain).

    This is also why I want to write two seperate enums.
     */

    private Pose2d pathPlannerPose;
    CommandSwerveDrivetrain drive;
    Vision vision;

    // https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf 
    //page twenty four 
    private enum LEFT_REEF_POSES{
        A(0.0, 0.0, 0),
        C(0, 0, 0),
        E(0,0,0),
        G(0,0,0),
        I(0,0,0),
        K(0,0,0);

        private final double x;
        private final double y;

        private LEFT_REEF_POSES( double x, double y, double rotation) {
            this.x = x;
            this.y = y;
        }
    }

    private enum RIGHT_REEF_POSES{
        B(0,0,0),
        D(0,0, 0),
        F(0,0,0),
        H(0,0,0),
        J(0,0,0),
        L(0,0,0);

        private final double x;
        private final double y;

        private RIGHT_REEF_POSES( double x, double y, double rotation) {
            this.x = x;
            this.y = y;
        }
    }

    public void getClosestPoseLeft() {
        Pose2d[] blueReefSides =
        new Pose2d[] {REEF_POSES.A.pose.interpolate(REEF_POSES.B.pose, 0.5),
                REEF_POSES.C.pose.interpolate(REEF_POSES.D.pose, 0.5),
        // ...
            };
        double minDistanceFoundSoFar = -1;
        Pose2d closestPose = this.drive.getPose();
        for (Pose2d pose2d : blueReefSides) {
            double distance =
                    this.drive.getPose().getTranslation().getDistance(pose2d.getTranslation());
            if (minDistanceFoundSoFar == -1 || distance < minDistanceFoundSoFar) {
                minDistanceFoundSoFar = distance;
                closestPose = pose2d;
        }
    }

    public void getClosestPoseLeft() {
        Pose2d[] blueReefSides =
        new Pose2d[] {LEFT_REEF_POSES.A.pose.interpolate(LEFT_REEF_POSES.C.pose, 0.5),
                LEFT_REEF_POSES.C.pose.interpolate(LEFT_REEF_POSES.G.pose, 0.5),
        // ...
            };
        double minDistanceFoundSoFar = -1;
        Pose2d closestPose = this.drive.getPose();
        for (Pose2d pose2d : blueReefSides) {
            double distance =
                    this.drive.getPose().getTranslation().getDistance(pose2d.getTranslation());
            if (minDistanceFoundSoFar == -1 || distance < minDistanceFoundSoFar) {
                minDistanceFoundSoFar = distance;
                closestPose = pose2d;
        }
    }
// drive to that pose

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






  public void execute() {
    System.out.println("WE ARE RUNNING!!!!!!!!!");
    PathPlannerPath path = vision.getDriveToPosePath();
    if (path != null) {
        SmartDashboard.putData("Auto/Field", Field.globalField);

        PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          pathPlannerPose = pose;
          Field.globalField.getObject("target pose").setPose(pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          Field.globalField.getObject("path").setPoses(poses);
        });

        drive.followPathCommand(path).schedule();
    }
  }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
}

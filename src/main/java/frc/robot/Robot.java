// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sendables.Field;
import frc.robot.util.Elastic;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final Thread m_visionThread;

    public Robot() {
        this.m_robotContainer = new RobotContainer();
        this.m_robotContainer.resetRelativeEncoders();
        if (!isReal()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        SmartDashboard.putData("CommandScheduler Instance", CommandScheduler.getInstance());
        // https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html
        m_visionThread = new Thread(() -> {
            // Get the UsbCamera from CameraServer
            UsbCamera camera = CameraServer.startAutomaticCapture();
            int width = 160;
            int height = 120;
            // Set the resolution
            camera.setResolution(width, height);
            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("VideoOverlay", width, height);
            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();
            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                    // Send the output the error.
                    outputStream.notifyError(cvSink.getError());
                    // skip the rest of the current iteration
                    continue;
                }
                // Put a line on the image
                Imgproc.line(mat, new Point(80, 0), new Point(80, 60), new Scalar(0, 255, 0), 1); // green
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();

        Elastic.selectTab("Autonomous");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        } else {
            System.out.println("autonomous command was null!");
        }

        PathPlannerLogging
                .setLogActivePathCallback(Field.globalField.getObject("pp-active-path")::setPoses);
        PathPlannerLogging
                .setLogCurrentPoseCallback(Field.globalField.getObject("pp-current-pose")::setPose);
        PathPlannerLogging
                .setLogTargetPoseCallback(Field.globalField.getObject("pp-target-pose")::setPose);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // this.m_robotContainer.zero();
        Elastic.selectTab("Teleoperated");
        // this.m_robotContainer. // reset ControlBoard

        PathPlannerLogging.setLogActivePathCallback(null);
        PathPlannerLogging.setLogCurrentPoseCallback(null);
        PathPlannerLogging.setLogTargetPoseCallback(null);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}

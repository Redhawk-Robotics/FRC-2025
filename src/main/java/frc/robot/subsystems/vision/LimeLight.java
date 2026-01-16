package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Handles and setups up a limelight
public class LimeLight extends SubsystemBase {
    String name;
    NetworkTable table; // A network table containing all limelight data.
    boolean enabled;
    private final PoseEstimate kEmpty = new PoseEstimate(new Pose2d(), 0, 0, 0, 0, false);

    // Contains all data about the current tracked object. i.e a struct
    public static record PoseEstimate(
            Pose2d pose,
            double latencySeconds,
            int tagCount,
            double averageDistance,
            double averageArea,
            boolean exists) {
    };

    // Creates a limelight with the following name and pipeline.
    // Pipelines are selected in the web dashboard.

    public LimeLight(String name, int pipeline) {
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(this.name);
        this.table.getEntry(name).setInteger(pipeline);

        Sendable isEnabledSendable = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty(name, () -> enabled, (boolean val) -> {
                    enabled = val;
                });
            }
        };

        SmartDashboard.putData("[Limelight] " + this.name + " Enabled ", isEnabledSendable);
    }

    /**
     * Make a limelight with the given name and pipeline 0
     *
     * @param name The name of the limelight. Should be "limelight-xxx"
     */
    public LimeLight(String name) {
        this(name, 0);
    }

    /**
     * @return whether the limelight sees a target
     */
    public boolean hasTargets() {
        return table.getEntry("tv").getInteger(0) == 1;
    }

    /**
     * @return the horizontal offset from the target in counterclockwise positive
     *         degrees. Defaults to
     *         0 if no target is seen
     */
    public double getTX() {
        return -table.getEntry("tx").getDouble(0);
    }

    /**
     * @return the vertical offset from the target in degrees. Defaults to 0 if no
     *         target is seen
     */
    public double getTY() {
        return table.getEntry("ty").getDouble(0);
    }

    /**
     * @return the area taken up by the target in %. Defaults to 0 if no target is
     *         seen
     */
    public double getTA() {
        return table.getEntry("ta").getDouble(0);
    }

    /**
     * @return the total latency of all data from the limelight to the code
     */
    public double getLatency() {
        return table.getEntry("cl").getDouble(0) + table.getEntry("tl").getDouble(0);
    }

    /**
     * @return the primary tag in view, or -1 if no tags seen
     */
    public int getID() {
        return (int) table.getEntry("tid").getInteger(-1);
    }

    /**
     * Only allows megatag to use the given ids for localization
     *
     * @param ids The list of tags to allow
     */
    public void setUseableTags(double... ids) {
        table.getEntry("fiducial_id_filters_set").setDoubleArray(ids);
    }

    /**
     * @return a {@link PoseEstimate} holding information about an estimated pose
     *         obtained using the
     *         megatag 1 algorithm.
     */
    public PoseEstimate getPoseMT1() {
        if (RobotBase.isSimulation()) {
            return kEmpty;
        }
        double[] raw = table.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
        return new PoseEstimate(
                new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
                raw[6] / 1000,
                (int) raw[7],
                raw[9],
                raw[10],
                hasTargets());
    }

    /**
     * @param currentRotation the current field relative rotation of the robot
     * @return a {@link PoseEstimate} holding information about an estimated pose
     *         obtained using the
     *         megatag 2 algorithm.
     */
    public PoseEstimate getPoseMT2(Rotation2d currentRotation, Rotation2d speedPerSec) {
        table
                .getEntry("robot_orientation_set")
                .setDoubleArray(
                        new double[] { currentRotation.getDegrees(), speedPerSec.getDegrees(), 0, 0, 0, 0 });
        NetworkTableInstance.getDefault().flush();
        return getPoseMT2();
    }

    private PoseEstimate getPoseMT2() {
        if (RobotBase.isSimulation()) {
            return kEmpty;
        }
        double[] raw = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
        return new PoseEstimate(
                new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
                raw[6] / 1000,
                (int) raw[7],
                raw[9],
                raw[10],
                hasTargets());
    }

    /**
     * @return returns both the 2d transfrom of the target april tag relative to the
     *         camera, and the
     *         targeted april tag ID
     */
    public Pair<Transform2d, Integer> getTargetPoseCameraSpace() {
        double[] raw = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[11]);
        return new Pair<>(new Transform2d(raw[2], raw[0], Rotation2d.fromDegrees(raw[5])), getID());
    }

    /**
     * @return returns both the 2d transfrom of the target april tag relative to the
     *         robot, and the
     *         targeted april tag ID
     */
    public Pair<Transform2d, Integer> getTargetPoseRobotSpace() {
        double[] raw = table.getEntry("targetpose_robotspace").getDoubleArray(new double[11]);
        return new Pair<>(
                new Transform2d(raw[2], raw[0], Rotation2d.fromDegrees(raw[5])),
                hasTargets() ? (int) table.getEntry("tid").getInteger(-1) : -1);
    }

    /**
     * @return the limelight's name
     */
    public String getName() {
        return name;
    }

    /**
     * @return whether the limelight is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

}

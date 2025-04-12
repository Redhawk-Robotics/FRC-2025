package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.Settings;

// An elevator created with three Krakens instead of 4 Neo/Spark.
// https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html

class Impl extends Elevator {
    /** Creates a new krakenElevator. */

    private final TalonFX m_topRightElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_RIGHT); // leader
    private final TalonFX m_topLeftElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_LEFT);
    private final TalonFX m_bottomLeftElevatorMotor =
            new TalonFX(Settings.Elevator.CAN.ID_BOTTOM_LEFT);

    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
    private final DutyCycleOut speedControl = new DutyCycleOut(0);

    private double setPoint = 0;
    private double speed = 0;

    private boolean usePosition = false;

    Impl() {
        this.configureMotors(//
                Settings.Elevator.kG, Settings.Elevator.kS, //
                Settings.Elevator.kP, Settings.Elevator.kI, Settings.Elevator.kD);
        this.resetElevatorPosition();
    }

    @Override
    public void configureMotors(double kG, double kS, double kP, double kI, double kD) {

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(40));

        //Volts need to be confirmed
        // motorConfig.Voltage.withPeakForwardVoltage(0).withPeakReverseVoltage(0);

        //Amps need to be confirmed
        // motorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(0)
        //         .withPeakReverseTorqueCurrent(0);

        this.m_topRightElevatorMotor.getConfigurator().apply(motorConfig);
        this.m_topLeftElevatorMotor.getConfigurator().apply(motorConfig);
        this.m_bottomLeftElevatorMotor.getConfigurator().apply(motorConfig);

        this.m_topRightElevatorMotor.getConfigurator().apply(
                (new TalonFXConfiguration().Slot0.withGravityType(GravityTypeValue.Elevator_Static))
                        .withKG(Settings.Elevator.kG)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                        .withKS(Settings.Elevator.kS).withKP(kP).withKI(kI).withKD(kD));

        this.m_topLeftElevatorMotor
                .setControl(new Follower(Settings.Elevator.CAN.ID_TOP_RIGHT, true));
        this.m_bottomLeftElevatorMotor
                .setControl(new Follower(Settings.Elevator.CAN.ID_TOP_RIGHT, true));

        System.out.println("Done configuring Kraken Elevator motors.");
    }

    @Override
    public void resetElevatorPosition() {
        this.m_topRightElevatorMotor.setPosition(0);
    }

    @Override
    public Angle getPosition() {
        return this.m_topRightElevatorMotor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return this.m_topRightElevatorMotor.getVelocity().getValue();
    }

    @Override
    public void setReference(double reference) {
        this.setPoint = reference;
        this.usePosition = true;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void useSpeed() {
        this.usePosition = false;
    }

    @Override
    public void stopElevator() {
        this.useSpeed();
        this.setSpeed(0);
    }


    @Override
    public void periodic() {
        super.periodic();

        if (this.usePosition) {
            this.m_topRightElevatorMotor
                    .setControl(this.positionControl.withPosition(this.setPoint));
        } else {
            this.m_topRightElevatorMotor.setControl(this.speedControl.withOutput(this.speed));
        }
        // SmartDashboard.putBoolean("KrakenElevator/use PID", this.shouldUsePIDControl());
        // SmartDashboard.putNumber("KrakenElevator/PID setPoint", this.setPoint);
        // SmartDashboard.putNumber("KrakenElevator/PID slot", this.slotIndex);
        // SmartDashboard.putBoolean("KrakenElevator/use speed", !this.shouldUsePIDControl());
        // SmartDashboard.putNumber("KrakenElevator/target speed", this.speed);

        // SmartDashboard.putNumber("KrakenElevator/Motor1/speed", topRightMotor.get());
        // SmartDashboard.putNumber("KrakenElevator/Motor2/speed", bottomRightMotor.get());
        // SmartDashboard.putNumber("KrakenElevator/Motor3/speed", topLeftMotor.get());
        // SmartDashboard.putNumber("KrakenElevator/Motor4/speed", bottomLeftMotor.get());

        // SmartDashboard.putNumber("KrakenElevator/Motor1/voltage", topRightMotor.getBusVoltage());
        // SmartDashboard.putNumber("KrakenElevator/Motor2/voltage", bottomRightMotor.getBusVoltage());
        // SmartDashboard.putNumber("KrakenElevator/Motor3/voltage", topLeftMotor.getBusVoltage());
        // SmartDashboard.putNumber("KrakenElevator/Motor4/voltage", bottomLeftMotor.getBusVoltage());
    }
}

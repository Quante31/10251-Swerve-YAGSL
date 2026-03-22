package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoV1;
import frc.robot.Ports;
import frc.robot.SparkMotor;

/*
 * intake mechanism, consisting of a roller motor and a pivot motor.
 */
public class IntakeSubsystem extends SubsystemBase {

    public enum Speed {
        STOP(0.0),
        INTAKE(0.8),
        TEST(0.45);
        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }
    public enum Position {
        HOMED(0.35),
        STOWED(0.3),
        INTAKE(-0.018),
        AGITATE(0.05);
       // HOMED((3.1415 / 180) * 110),
       // STOWED((3.1415 / 180) * 100),
       // INTAKE((3.1415 / 180) * -4),
       // AGITATE((3.1415 / 180) * 20);

        private final double position;

        private Position(double position) {
            this.position = position;
        }

        public double position() {
            return this.position;
        }
    }
    private static final double kPivotReduction = 4.0;
    private static final AngularVelocity kMaxPivotSpeed = NeoV1.kFreeSpeed.div(kPivotReduction);
    private static final double kPositionTolerance = 0.0250;
    private final SparkMotor rollerMotor;
    private final SparkMotor pivotMotor;
    private boolean isHomed = false;

    public IntakeSubsystem() {
        rollerMotor = new SparkMotor(Ports.kIntakeRollers, MotorType.kBrushless);
        pivotMotor = new SparkMotor(Ports.kIntakePivot, MotorType.kBrushless);
        configurePivotMotor();
        configureRollerMotor();
        SmartDashboard.putData(this);
    }
    private void configurePivotMotor() {
        final SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
            .positionConversionFactor(1.0 / kPivotReduction)
            .velocityConversionFactor(1.0 / kPivotReduction);
        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(NeoV1.kSmartCurrentLimitHigh)
            .voltageCompensation(NeoV1.kNominalVoltage);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1.0, 1.0)
            .p(0.01)
            .i(0.0)
            .d(0.0);
        config.closedLoop.feedForward.kV(NeoV1.kNominalVoltage / kMaxPivotSpeed.in(RPM));
        config.closedLoop.maxMotion
            .cruiseVelocity(120/*kMaxPivotSpeed.in(RPM)*/)
            .maxAcceleration(40/*kMaxPivotSpeed.per(Second).in(RPM.per(Second))*/);

        pivotMotor.configureMotor(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        final SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(NeoV1.kSmartCurrentLimitHigh)
            .voltageCompensation(NeoV1.kNominalVoltage);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.feedForward.kV(NeoV1.kNominalVoltage / NeoV1.kFreeSpeed.in(RPM));
        rollerMotor.configureMotor(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    private boolean isPositionWithinTolerance() {
        final double position = pivotMotor.getMAXMotionSetpointPosition();
        final SparkMotor.Setpoint setpoint = pivotMotor.getSetpoint();

        return setpoint != null && setpoint.getControlType() == ControlType.kMAXMotionPositionControl && Math.abs(pivotMotor.getPosition().getAsDouble() - position) <= kPositionTolerance;
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setPercentOutput(percentOutput);
    }

    public void set(Position position) {
        pivotMotor.set(position.position(), ControlType.kMAXMotionPositionControl);
    }
    public void setVoltage(double voltage){
        pivotMotor.set(voltage, ControlType.kVoltage);
    }
    public void set(Speed speed) {
        rollerMotor.set(speed.voltage().in(Volts), ControlType.kVoltage);
    }

    public Command intakeCommand() { 
        return intakeOpenCommand().andThen(intakeRollCommand());
    }
    public Command intakeOpenCommand(){
        // TODO Change Commands.waitUntil to position check
        return runOnce(() -> setVoltage(-6.0))
               .andThen(Commands.waitUntil(() -> pivotMotor.getOutputCurrentAmps() > NeoV1.kSmartCurrentLimitHigh).withTimeout(2))
               .finallyDo(() -> setVoltage(0.0)).withName("Intake Open Command");
    }
    public Command intakeRollCommand(){
        return startEnd(() -> set(Speed.TEST), () -> set(Speed.STOP)).withName("Intake Roll Command");
    }
    public Command intakeStopCommand(){
        return runOnce(() -> setVoltage(0.0)).andThen(runOnce(() -> set(Speed.STOP)));
    }
    public Command agitateCommand() {
        return runOnce(() -> set(Speed.TEST))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE);
                set(Speed.STOP);
            }).withName("Intake Agitate Command");
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.1)),
            Commands.waitUntil(() -> pivotMotor.getOutputCurrentAmps() > 6),
            runOnce(() -> {
                pivotMotor.set(Position.HOMED.position(), ControlType.kMAXMotionPositionControl);
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("Intake Homing Command");
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Position",
            () -> (pivotMotor.getPosition().getAsDouble()), null);
        builder.addDoubleProperty("Velocity (RPM)", rollerMotor::getVelocityRPM, null);
        //builder.addDoubleProperty("Pivot setpoint position", pivotMotor::getMAXMotionSetpointPosition, null);
        //builder.addDoubleProperty("Roller setpoint velocity", rollerMotor::getMAXMotionSetpointVelocity, null);
        //builder.addDoubleProperty("Pivot Output Current", pivotMotor::getOutputCurrentAmps, null);
        //builder.addDoubleProperty("Roller Output Current", rollerMotor::getOutputCurrentAmps, null);
    }
}
 

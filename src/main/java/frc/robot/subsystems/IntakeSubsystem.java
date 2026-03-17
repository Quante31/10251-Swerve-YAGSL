package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
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
        INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }
    public enum Position {
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }
    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = NeoV1.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);
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
            .p(1.0)
            .i(0.0)
            .d(0.0);
        config.closedLoop.feedForward.kV(NeoV1.kNominalVoltage / kMaxPivotSpeed.in(RPM));
        config.closedLoop.maxMotion
            .cruiseVelocity(kMaxPivotSpeed.in(RPM))
            .maxAcceleration(kMaxPivotSpeed.per(Second).in(RPM.per(Second)));

        pivotMotor.configureMotor(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        /*final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);*/
    }

    private void configureRollerMotor() {
        final SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(NeoV1.kSmartCurrentLimitHigh)
            .voltageCompensation(NeoV1.kNominalVoltage);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.feedForward.kV(NeoV1.kNominalVoltage / NeoV1.kFreeSpeed.in(RPM));
        rollerMotor.configureMotor(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        /*final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);*/
    }


    private boolean isPositionWithinTolerance() {
        final SparkMotor.Setpoint setpoint = pivotMotor.getSetpoint();
        return setpoint != null
            && setpoint.getControlType() == ControlType.kMAXMotionPositionControl
            && Math.abs(pivotMotor.getPosition().getAsDouble() - setpoint.getValue()) <= kPositionTolerance.in(Degrees);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setPercentOutput(percentOutput);
    }

    public void set(Position position) {
        pivotMotor.set(position.angle().in(Degrees), ControlType.kMAXMotionPositionControl);
    }

    public void set(Speed speed) {
        rollerMotor.set(speed.voltage().in(Volts), ControlType.kVoltage);
    }

    public Command intakeCommand() {
        return startEnd(
            () -> {
                set(Position.INTAKE);
                set(Speed.INTAKE);
            },
            () -> set(Speed.STOP)
        );
    }

    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
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
            });
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.1)),
            Commands.waitUntil(() -> pivotMotor.getOutputCurrentAmps() > 6),
            runOnce(() -> {
                pivotMotor.set(Position.HOMED.angle().in(Degrees), ControlType.kMAXMotionPositionControl);
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)",
            () -> (pivotMotor.getPosition().getAsDouble()), null);
        builder.addDoubleProperty("RPM", rollerMotor::getVelocityRPM, null);
        builder.addDoubleProperty("Pivot Output Current", pivotMotor::getOutputCurrentAmps, null);
        builder.addDoubleProperty("Roller Output Current", rollerMotor::getOutputCurrentAmps, null);
    }
}
 

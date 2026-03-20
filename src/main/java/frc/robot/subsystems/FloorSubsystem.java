package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoV1;
import frc.robot.Ports;
import frc.robot.SparkMotor;

public class FloorSubsystem extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.83),
        TEST(0.2);
        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public double voltage() {
            return percentOutput * NeoV1.kNominalVoltage;
        }
    }

    private final SparkMotor motor;
    private static final double kPivotReduction = 8.0;
    private static final AngularVelocity kMaxPivotSpeed = NeoV1.kFreeSpeed.div(kPivotReduction);
    //private final VoltageOut voltageRequest = new VoltageOut(0);

    public FloorSubsystem() {
        motor = new SparkMotor(Ports.kFloor, MotorType.kBrushless);
        final SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
            .positionConversionFactor(1.0 / kPivotReduction)
            .velocityConversionFactor(1.0 / kPivotReduction);
        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(30)
            .voltageCompensation(NeoV1.kNominalVoltage);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.feedForward.kV(NeoV1.kNominalVoltage / kMaxPivotSpeed.in(RPM))/*NeoV1.kFreeSpeed.in(RPM))*/;
        motor.configureMotor(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
            );

        motor.getConfigurator().apply(config);*/
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.set(speed.voltage(), ControlType.kVoltage);
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> motor.getVelocityRPM(), null);
        //builder.addDoubleProperty("Output Current", () -> motor.getOutputCurrentAmps(), null);
        //builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
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

public class FeederSubsystem extends SubsystemBase {
    public enum Speed {
        FEED(5000.0),
        TEST(1000.0),
        STOP(0.0);
        private final double rpm;

        Speed(double rpm) {
            this.rpm = rpm;
        }

        public double rpm() {
            return rpm;
        }
    }

    private final SparkMotor motor;
    private static final double kPivotReduction = 2.4;
    private static final AngularVelocity kMaxPivotSpeed = NeoV1.kFreeSpeed.div(kPivotReduction);
    //private final RelativeEncoder encoder;
    //private final SparkClosedLoopController closedLoopController;

    public FeederSubsystem() {
        motor = new SparkMotor(Ports.kFeeder, MotorType.kBrushless);
        //encoder = motor.getEncoder();
        //closedLoopController = motor.getClosedLoopController();
        
        final SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
            .positionConversionFactor(1.0 / kPivotReduction)
            .velocityConversionFactor(1.0 / kPivotReduction);
        config
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(NeoV1.kSmartCurrentLimitLow)
            .voltageCompensation(NeoV1.kNominalVoltage);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.0)
            .i(0.0)
            .d(0.0);
        config.closedLoop.feedForward.kV(NeoV1.kNominalVoltage / kMaxPivotSpeed.in(RPM));
        motor.configureMotor(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.setRPM(speed);
        //closedLoopController.setSetpoint(speed.rpm(), ControlType.kVelocity);
    }

    public void setPercentOutput(double percentOutput) {
        motor.setPercentOutput(percentOutput);
        //motor.setVoltage(Volts.of(percentOutput * NeoV1.kNominalVoltage));
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.TEST), () -> setPercentOutput(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        //builder.addDoubleProperty("RPM", motor.getVelocity(), null);
        //builder.addDoubleProperty("Output Current", motor.getOutputCurrent(), null);
        //builder.addDoubleProperty("Applied Voltage", motor::getAppliedVoltageVolts, null);
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.List;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoV1;
import frc.robot.Ports;
import frc.robot.SparkMotor;
import frc.robot.util.SparkSetpointUtil;

public class ShooterSubsystem extends SubsystemBase {
    private static final double kVelocityToleranceRpm = 100.0;
    private final SparkMotor leftMotor, middleMotor, rightMotor;
    private final List<SparkMotor> motors;

    private double dashboardTargetRPM = 5200.0;

    public ShooterSubsystem() {
        leftMotor = new SparkMotor(Ports.kShooterLeft, MotorType.kBrushless);
        middleMotor = new SparkMotor(Ports.kShooterMiddle, MotorType.kBrushless);
        rightMotor = new SparkMotor(Ports.kShooterRight, MotorType.kBrushless);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        // Original Talon setup had opposite inversion on left vs middle/right.
        configureMotor(leftMotor, true
        );
        configureMotor(middleMotor, true);
        configureMotor(rightMotor, true);

        SmartDashboard.putData(this);
    }

    private void configureMotor(SparkMotor motor, boolean invertDirection) {
        final SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kCoast)
            .inverted(invertDirection)
            .smartCurrentLimit(NeoV1.kSmartCurrentLimitHigh)
            .voltageCompensation(NeoV1.kNominalVoltage);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(0.0, 1.0)
            .p(0.00025)
            .i(0.0)
            .d(0.0);
        config.closedLoop.feedForward.kV(NeoV1.kNominalVoltage / NeoV1.kFreeSpeed.in(RPM));

        motor.configureMotor(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setRPM(double rpm) {
        for (final SparkMotor motor : motors) {
            motor.setRPM(rpm);
            motor.setSetpoint(rpm, ControlType.kVelocity);
        }
    }
    public void setRPMLeftRight(double rpm) {
        leftMotor.setRPM(rpm);
        leftMotor.setSetpoint(rpm, ControlType.kVelocity);
        rightMotor.setRPM(rpm);
        rightMotor.setSetpoint(rpm, ControlType.kVelocity);
    }
    public void setRPMMiddle(double rpm) {
        middleMotor.setRPM(rpm);
        middleMotor.setSetpoint(rpm, ControlType.kVelocity);     
    }
    public void setPercentOutput(double percentOutput) {
        for (final SparkMotor motor : motors) {
            motor.setPercentOutput(percentOutput);
            motor.setSetpoint(percentOutput, ControlType.kVoltage);
        }
    }
    public void setPercentOutputLeftRight(double percentOutput) {
        leftMotor.setPercentOutput(percentOutput);
        leftMotor.setSetpoint(percentOutput, ControlType.kVoltage);
        rightMotor.setPercentOutput(percentOutput);
        rightMotor.setSetpoint(percentOutput, ControlType.kVoltage);
    }
    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
        //return runOnce(() -> setRPMLeftRight(rpm)).alongWith(Commands.runOnce(() -> setRPMMiddle(rpm - 3000)))
        //    .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
        if (RobotBase.isSimulation()){
            return runOnce(() -> setRPM(rpm));
        }
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            SparkMotor.Setpoint setpoint = motor.getSetpoint();
            return setpoint != null && (Math.abs(motor.getVelocityRPM() - SparkSetpointUtil.toVelocityRpm(setpoint)) <= kVelocityToleranceRpm);
        });
    }

    private void initSendable(SendableBuilder builder, SparkMotor motor, String name) {
        builder.addDoubleProperty(name + " RPM", motor::getVelocityRPM, null);
        builder.addDoubleProperty(name + " Output Current", motor::getOutputCurrentAmps, null);
        //builder.addDoubleProperty(name + " Applied Voltage", motor::getAppliedVoltageVolts, null);
        //builder.addDoubleProperty(name + "Target RPM", () -> motor.getSetpoint().getValue(), null);

        //builder.addIntegerProperty(name + "Velocity Mode", motor.getControlMode(), null);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
    }
}

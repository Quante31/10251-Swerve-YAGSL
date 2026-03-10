package frc.robot.subsystems;

/*import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
*/
import java.util.List;

//import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.Debouncer;
//import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Ports;

public class ShooterSubsystem extends SubsystemBase {
    //private static final AngularVelocity kVelocityTolerance = RPM.of(100);
    private static final double kSpinupSec = 0.70; // tune on robot
    private static final double kMinShootPercent = 0.05;
    private double commandedPercent = 0.0;
    private double dashboardTargetRPM = 0.0;

    private final PWMSparkMax leftMotor, middleMotor, rightMotor;
    private final List<PWMSparkMax> motors;
    private final Timer spinupTimer = new Timer();
    private final Debouncer readyDebounce = new Debouncer(0.05);

    //private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    //private final VoltageOut voltageRequest = new VoltageOut(0);
    

    public ShooterSubsystem() {
        //SparkMax testMotor = new SparkMax(1, MotorType.kBrushless);
        //testMotor.getAbsoluteEncoder().getVelocity();
        leftMotor = new PWMSparkMax(Ports.kShooterLeftPWM);
        middleMotor = new PWMSparkMax(Ports.kShooterMiddlePWM);
        rightMotor = new PWMSparkMax(Ports.kShooterRightPWM);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, true);
        configureMotor(middleMotor, false);
        configureMotor(rightMotor, false);

        SmartDashboard.putData(this);
    }

    private void configureMotor(PWMSparkMax motor, boolean inverted) {
        motor.setInverted(inverted);
        motor.setSafetyEnabled(false);
    }

    /*public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }
    }*/

    public void setPercentOutput(double percentOutput) {
        commandedPercent = percentOutput;
        for (final PWMSparkMax motor : motors) {
            motor.set(commandedPercent);
        }

        if (Math.abs(commandedPercent) > kMinShootPercent) {
            spinupTimer.restart();
        } else {
            spinupTimer.stop();
            spinupTimer.reset();
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double percentOutput) {
        /*return runOnce(() -> setPercentOutput(percentOutput))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));*/
        return Commands.sequence(runOnce(() -> setPercentOutput(percentOutput)), Commands.waitSeconds(kSpinupSec)).withTimeout(kSpinupSec + 0.15);
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public boolean isVelocityWithinTolerance() {
        // TODO Improve
        boolean readyEstimate = Math.abs(commandedPercent) > kMinShootPercent && spinupTimer.hasElapsed(kSpinupSec);
        return readyDebounce.calculate(readyEstimate);

        /*return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });*/
    }

    private void initSendable(SendableBuilder builder, PWMSparkMax motor, String name) {
        builder.addDoubleProperty(name + " Output PWM", () -> motor.get(), null);
        /*builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);*/
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        /*builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);*/
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that controls the intake mechanism: a motor to pull balls in and a solenoid to extend/retract
 * the intake arm.
 *
 * Notes: The PWM port and PCM channel numbers are defined in {@link Constants.IntakeConstants} and are
 * assumed values for now — update them to match your robot wiring.
 */
public class IntakeSubsystem extends SubsystemBase {
        private final PWMSparkMax intakeMotor;
    private final DoubleSolenoid intakeSolenoid;

    public IntakeSubsystem() {
        // Motor (PWM) and solenoid initialized from Constants. Update ports as needed.
        intakeMotor = new PWMSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_PWM_PORT);
        intakeSolenoid = new DoubleSolenoid(Constants.IntakeConstants.PCM_CAN_ID,
            PneumaticsModuleType.CTREPCM,
            Constants.IntakeConstants.SOLENOID_FORWARD_CHANNEL,
            Constants.IntakeConstants.SOLENOID_REVERSE_CHANNEL);
    }

    /** Extend/deploy the intake */
    public void extend() {
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    /** Retract the intake */
    public void retract() {
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /** Toggle the intake extension state */
    public void toggle() {
        if (isExtended()) {
            retract();
        } else {
            extend();
        }
    }

    /** Returns true if intake is extended (forward) */
    public boolean isExtended() {
        return intakeSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    /** Run the intake motor at given speed (-1..1) */
    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    /** Stop the intake motor */
    public void stop() {
        intakeMotor.set(0.0);
    }

    /**
     * Small convenience command to agitate the intake (run it briefly). Returns a RunCommand with a short
     * timeout; caller can modify or wrap it as needed.
     */
    public Command agitateCommand() {
        return new RunCommand(() -> setIntake(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED), this)
                .withTimeout(0.5)
                .andThen(() -> stop());
    }

}

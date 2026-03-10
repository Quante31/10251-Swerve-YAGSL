package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake subsystem using a single NEO driven through a PWM Spark Max (PWMSparkMax).
 * Provides open-loop control for the roller motor and simple command factories.
 */
public class IntakeSubsystem extends SubsystemBase {
    public enum Speed {
        STOP(0.0),
        INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public double percent() {
            return percentOutput;
        }
    }

    private final PWMSparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new PWMSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_PWM_PORT);
        SmartDashboard.putData(this);
    }

    /** Run the roller at a given percent (-1..1) */
    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    /** Run using preset Speed enum */
    public void set(Speed speed) {
        setIntake(speed.percent());
    }

    /** Start the intake (convenience for commands that expect an extend() method) */
    public void extend() {
        set(Speed.INTAKE);
    }

    /** Toggle the intake roller: if running stop, otherwise start at INTAKE speed */
    public void toggle() {
        if (Math.abs(intakeMotor.get()) > 1e-6) {
            stop();
        } else {
            extend();
        }
    }

    /** Stop the roller */
    public void stop() {
        intakeMotor.set(0);
    }

    /** Command that runs intake while active, stops on end */
    public Command intakeCommand() {
        return Commands.startEnd(() -> set(Speed.INTAKE), this::stop, this);
    }

    /** Simple agitate: spin for a short burst */
    public Command agitateCommand() {
        return Commands.sequence(Commands.runOnce(() -> setIntake(Speed.INTAKE.percent())), Commands.waitSeconds(0.5), Commands.runOnce(this::stop));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Output", () -> intakeMotor.get(), null);
    }
}
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        public double percent() {
            return percentOutput;
        }
    }

    private final Spark rollerMotor;
    private final Spark pivotMotor;

    public IntakeSubsystem() {
        rollerMotor = new Spark(Constants.IntakeConstants.ROLLER_MOTOR_PWM_PORT);
        pivotMotor = new Spark(Constants.IntakeConstants.PIVOT_MOTOR_PWM_PORT);
        SmartDashboard.putData(this);
    }

    /** Run the roller at a given percent (-1..1) */
    /** Backwards-compatible name used in RobotContainer: sets the roller output. */
    public void setIntake(double speed) {
        setRoller(speed);
    }

    /** Run the roller at a given percent (-1..1) */
    public void setRoller(double speed) {
        rollerMotor.set(speed);
    }

    /** Run using preset Speed enum */
    public void set(Speed speed) {
        setIntake(speed.percent());
    }

    /** Start the intake (convenience for commands that expect an extend() method) */
    /** Extend the intake pivot (open-loop). This sets the pivot motor to EXTEND_SPEED.
     *  This method is intentionally simple — stopPivot() or retract() must be called to stop/retract.
     */
    public void extend() {
        pivotMotor.set(Constants.IntakeConstants.EXTEND_SPEED);
    }

    /** Retract the intake pivot (open-loop). */
    public void retract() {
        pivotMotor.set(Constants.IntakeConstants.RETRACT_SPEED);
    }

    /** Stop pivot motor movement */
    public void stopPivot() {
        pivotMotor.set(0);
    }

    /** Toggle the intake roller: if running stop, otherwise start at INTAKE speed */
    /** Toggle the roller motor: if running stop, otherwise start at INTAKE speed */
    public void toggle() {
        if (Math.abs(rollerMotor.get()) > 1e-6) {
            stop();
        } else {
            set(Speed.INTAKE);
        }
    }

    /** Stop the roller */
    /** Stop both roller and pivot motors */
    public void stop() {
        rollerMotor.set(0);
        pivotMotor.set(0);
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
        builder.addDoubleProperty("Roller Output", () -> rollerMotor.get(), null);
        builder.addDoubleProperty("Pivot Output", () -> pivotMotor.get(), null);
    }
}
 
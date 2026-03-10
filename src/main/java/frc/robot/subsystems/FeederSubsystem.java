package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Ports;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    public enum Speed {
        STOP(0.0),
        FEED(5000.0 / Constants.NeoV1.kFreeSpeed.in(RPM));
        
        private final double percentOutput;

        Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }
    }

    private final PWMSparkMax motor;
    private double commandedOutput;

    public FeederSubsystem() {
        motor = new PWMSparkMax(Ports.kFeederPWM);
        motor.setInverted(false);
        motor.setSafetyEnabled(false);
        commandedOutput = Speed.STOP.percentOutput;
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        setPercentOutput(speed.percentOutput);
    }

    public void setPercentOutput(double percentOutput) {
        commandedOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        motor.set(commandedOutput);
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Output", () -> commandedOutput, null);
    }
}

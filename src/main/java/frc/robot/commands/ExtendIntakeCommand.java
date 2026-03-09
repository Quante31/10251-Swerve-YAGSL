package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

/** Instant command to extend the intake (deploy the mechanism) */
public class ExtendIntakeCommand extends InstantCommand {
    private final IntakeSubsystem intake;

    public ExtendIntakeCommand(IntakeSubsystem intake) {
        // call IntakeSubsystem.extend() once when the command is triggered
        super(intake::extend, intake);
        this.intake = intake;
    }

}

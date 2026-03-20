package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.NeoV1;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
//import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public final class SubsystemCommands {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final FloorSubsystem floor;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    //private final HoodSubsystem hood;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(SwerveSubsystem swerve, IntakeSubsystem intake, FloorSubsystem floor, FeederSubsystem feeder, ShooterSubsystem shooter, /*HoodSubsystem hood,*/ DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        //this.hood = hood;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        SwerveSubsystem swerve,
        IntakeSubsystem intake,
        FloorSubsystem floor,
        FeederSubsystem feeder,
        ShooterSubsystem shooter
        //HoodSubsystem hood
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            //hood,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter/*, hood*/, () -> swerve.getPose());
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed())
        );
    }
    public Command testCommand() {
        return Commands.parallel(
            Commands.run(() -> feeder.set(FeederSubsystem.Speed.TEST)),
            Commands.waitSeconds(3),
            Commands.run(() -> feeder.setPercentOutput(0)),
            Commands.waitSeconds(1),
            Commands.run(() -> floor.set(FloorSubsystem.Speed.TEST)),
            Commands.waitSeconds(3),
            Commands.run(() -> floor.set(FloorSubsystem.Speed.STOP)),
            Commands.waitSeconds(1),
            shooter.spinUpCommand(100),
            Commands.waitSeconds(3),
            Commands.run(() -> shooter.setPercentOutput(0))
        );
    }
    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> shooter.stop());
    }

    private Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand()/* .alongWith(intake.agitateCommand()) */)
            )
        );
    }
}

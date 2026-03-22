// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTraj$0;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTraj$1;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTraj$2;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTraj$3;

import static frc.robot.generated.ChoreoTraj.StartToBump;
import static frc.robot.generated.ChoreoTraj.BumpToBump2;
import static frc.robot.generated.ChoreoTraj.Bump2ToIntake;
import static frc.robot.generated.ChoreoTraj.IntakeToBump2;
import static frc.robot.generated.ChoreoTraj.Bump2ToBump;
import static frc.robot.generated.ChoreoTraj.BumpToShootingPose;
import static frc.robot.generated.ChoreoTraj.ShootingPoseToOutpost;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public final class AutoRoutines {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final FloorSubsystem floor;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        SwerveSubsystem swerve,
        IntakeSubsystem intake,
        FloorSubsystem floor,
        FeederSubsystem feeder,
        ShooterSubsystem shooter
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;


        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter/* hood, hanger*/);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
        autoChooser.addRoutine("Intake Shoot and Return", this::intakeShootAndReturn);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");

        final AutoTrajectory startToOutpost = OutpostAndDepotTraj$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTraj$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTraj$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTower = OutpostAndDepotTraj$3.asAutoTraj(routine);

        // Start first trajectory
        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );
        // Check until the hanger is homed, then open the intake
        /*routine.observe(hanger::isHomed).onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))
            )
        );*/
        outpostToDepot.atTimeBeforeEnd(1.5).onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                intake.runOnce(() -> intake.set(IntakeSubsystem.Position.INTAKE))
            )
        );
        // 1 second after the trajectory finishes enable next trajectory
        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());

        // 1 second before the trajectory ends enable intake
        outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());

        // 0.1 seconds after the trajectory finishes enable next trajectory
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());

        // 0.5 seconds after the trajectory starts spin up the shooter and position the hood
        depotToShootingPose.atTime(0.5).onTrue(shooter.spinUpCommand(2800));

        // after the trajectory finishes, aim, shoot and enable next trajectory
        depotToShootingPose.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootingPoseToTower.cmd()
            )
        );

        return routine;
    }
    private AutoRoutine intakeShootAndReturn() {
        final AutoRoutine routine = autoFactory.newRoutine("Intake Shoot and Return");

        final AutoTrajectory startToBump = StartToBump.asAutoTraj(routine);
        final AutoTrajectory bumpToBump2 = BumpToBump2.asAutoTraj(routine);
        final AutoTrajectory bump2ToIntake = Bump2ToIntake.asAutoTraj(routine);
        final AutoTrajectory intakeToBump2 = IntakeToBump2.asAutoTraj(routine);
        final AutoTrajectory bump2ToBump = Bump2ToBump.asAutoTraj(routine);
        final AutoTrajectory bumpToShootingPose = BumpToShootingPose.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToOutpost = ShootingPoseToOutpost.asAutoTraj(routine);
        // Start first trajectory
        routine.active().onTrue(
            Commands.sequence(
                startToBump.resetOdometry(),
                startToBump.cmd()
            )
        );
        startToBump.doneDelayed(0.2).onTrue(bumpToBump2.cmd());
        bumpToBump2.doneDelayed(0.2).onTrue(bump2ToIntake.cmd());
        bump2ToIntake.atTimeBeforeEnd(1.5).onTrue(intake.intakeCommand());
        bump2ToIntake.doneDelayed(0.5).onTrue(intakeToBump2.cmd());
        intakeToBump2.doneDelayed(0.2).onTrue(intake.intakeStopCommand().andThen(bump2ToBump.cmd()));
        bump2ToBump.doneDelayed(0.1).onTrue(bumpToShootingPose.cmd());
        bumpToShootingPose.atTime(0.5).onTrue(shooter.spinUpCommand(5200 /*8TODO Calibrate */));
        bumpToShootingPose.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootingPoseToOutpost.cmd()
            )
        );
        
        return routine;
    }
}

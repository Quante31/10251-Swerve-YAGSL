package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.GeometryUtil;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final SwerveSubsystem swerve;
    private final DriveInputSmoother inputSmoother;

    // Last commanded target direction in field frame.
    private final Translation2d hubPosition;
    private Rotation2d targetDirection;

    public AimAndDriveCommand(SwerveSubsystem swerve, DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        this.hubPosition = Landmarks.hubPosition();
        this.targetDirection = getDirectionToHub();
        addRequirements(swerve);
    }

    public AimAndDriveCommand(SwerveSubsystem swerve) {
        this(swerve, () -> 0.0, () -> 0.0);
    }

    public boolean isAimed() {
        final Rotation2d currentHeading = swerve.getPose().getRotation();
        return GeometryUtil.isNear(targetDirection, currentHeading, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
       
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        return hubPosition.minus(robotPosition).getAngle();
    }

    @Override
    public void initialize() {
        targetDirection = getDirectionToHub();
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        targetDirection = getDirectionToHub();
        final ChassisSpeeds speeds = swerve.getTargetSpeeds(input.forward, input.left, targetDirection);
        swerve.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

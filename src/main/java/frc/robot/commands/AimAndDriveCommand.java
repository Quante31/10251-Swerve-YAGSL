package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Landmarks;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.GeometryUtil;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final SwerveSubsystem swerve;
    private final DriveInputSmoother inputSmoother;

    // Basic slew limiters to smooth joystick inputs (adjust rates as needed)
    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.0); // m/s per sec (example)
    private final SlewRateLimiter leftLimiter = new SlewRateLimiter(3.0);

    // Replace with your robot max speed constant or expose from SwerveSubsystem
    private static final double kMaxSpeedMetersPerSecond = 3.0;

    // last computed target direction (operator perspective)
    private Rotation2d targetDirection;

    public AimAndDriveCommand(SwerveSubsystem swerve, DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        this.targetDirection = getDirectionToHub();
        addRequirements(swerve);
    }

    public AimAndDriveCommand(SwerveSubsystem swerve) {
        this(swerve, () -> 0.0, () -> 0.0);
    }

    private Rotation2d getOperatorForwardDirection() {
        if (DriverStation.getAlliance().orElse(null).equals(DriverStation.Alliance.Red)) {
            return Rotation2d.fromDegrees(180.0);
        }
        return Rotation2d.fromDegrees(0.0);
    }

    public boolean isAimed() {
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getPose().getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(getOperatorForwardDirection());
        return GeometryUtil.isNear(targetDirection, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

   // ...existing code...
    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();

        // compute hub direction in field (blue-alliance) frame
        /*inal Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();*/

        // get chassis speeds from SwerveSubsystem helper (expects joystick-scale inputs and field angle)
        // NOTE: do NOT scale inputs by max speed here — the helper uses Constants.MAX_SPEED internally
        final ChassisSpeeds speeds = swerve.getTargetSpeeds(input.forward, input.left, /*hubDirectionInBlueAlliancePerspective*/getDirectionToHub());

        // send speeds to whichever method in your SwerveSubsystem applies ChassisSpeeds/module states
        // replace with your actual drive method name if different
        swerve.setChassisSpeeds(speeds);
    }
// ...existing code...
    @Override
    public boolean isFinished() {
        return false;
    }
}
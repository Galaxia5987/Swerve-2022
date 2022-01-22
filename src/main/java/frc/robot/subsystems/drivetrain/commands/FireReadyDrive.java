package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.io.Console;
import java.util.function.DoubleSupplier;

/**
 * The commands
 */
public class FireReadyDrive extends HolonomicDrive {
    public FireReadyDrive(SwerveDrive swerve, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier) {
        super(swerve, forwardSupplier, strafeSupplier, () -> 0);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        double forward = speeds.vxMetersPerSecond, strafe = speeds.vyMetersPerSecond;
        Translation2d diff = Constants.Field.HUB_POSITION.minus(swerveDrive.getPose().getTranslation());
        Rotation2d angle = new Rotation2d(Math.atan2(diff.getY(), diff.getX()));
        swerveDrive.holonomicDriveKeepSetpoint(forward, strafe, angle);
        log(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}

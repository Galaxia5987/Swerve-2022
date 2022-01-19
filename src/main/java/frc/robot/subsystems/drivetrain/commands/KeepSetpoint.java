package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class KeepSetpoint extends HolonomicDrive {
    private final Supplier<Translation2d> targetSetpoint;

    public KeepSetpoint(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, Supplier<Translation2d> targetSetpoint) {
        super(swerveDrive, forwardSupplier, strafeSupplier, () -> 0);
        this.targetSetpoint = targetSetpoint;
    }

    @Override
    public void execute() {
        var speeds =  calculateVelocities();
        var delta = targetSetpoint.get().minus(swerveDrive.getPose().getTranslation());
        double angle = Robot.getAngle().getRadians()  - (Math.PI / 2 - Math.atan2(delta.getY(), delta.getX()));
        swerveDrive.holonomicDriveKeepSetpoint(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, new Rotation2d(angle));

    }
}

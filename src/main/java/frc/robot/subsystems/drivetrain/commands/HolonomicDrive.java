package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;
import webapp.FireLog;

import java.util.function.DoubleSupplier;

/**
 * The command is responsible for moving the robot holonomically.
 * The commands retrieve the velocities in every axis (including rotational) and moves the robot accordingly.
 */
public class HolonomicDrive extends CommandBase {
    protected final SwerveDrive swerveDrive;
    protected final DoubleSupplier forwardSupplier;
    protected final DoubleSupplier strafeSupplier;
    protected final DoubleSupplier rotationSupplier;

    public HolonomicDrive(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        swerveDrive.holonomicDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        FireLog.log("current_angle", Robot.getAngle().getRadians());
        FireLog.log("forward", speeds.vxMetersPerSecond);
        FireLog.log("strafe", speeds.vyMetersPerSecond);
        FireLog.log("rotation", speeds.omegaRadiansPerSecond);
    }

    /**
     * Calculates the speeds from the suppliers after filtering.
     */
    protected ChassisSpeeds calculateVelocities() {
        // get the values
        double forward = Utils.deadband(forwardSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double strafe = Utils.deadband(strafeSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double rotation = Utils.rotationalDeadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD) * Constants.SwerveDrive.ROTATION_MULTIPLIER;

        // recalculate - update based on the angle and the magnitude
        double alpha = Math.atan2(forward, strafe); // direction of movement
        double magnitude = Math.hypot(forward, strafe) * Constants.SwerveDrive.VELOCITY_MULTIPLIER;
        forward = Math.sin(alpha) * magnitude;
        strafe = Math.cos(alpha) * magnitude;
        return new ChassisSpeeds(forward, strafe, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
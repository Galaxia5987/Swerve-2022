package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;
import webapp.FireLog;

import java.util.function.DoubleSupplier;

public class HolonomicDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;


    public HolonomicDrive(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // get the values
        double forward = Utils.deadband(forwardSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double strafe = Utils.deadband(strafeSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double rotation = Utils.rotationalDeadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD) * Constants.SwerveDrive.ROTATION_MULTIPLIER;

        // recalculate - update based on the angle and the magnitude
        double alpha = Math.atan2(forward, strafe); // direction of movement
        double magnitude = Math.hypot(forward, strafe) * Constants.SwerveDrive.VELOCITY_MULTIPLIER;
        forward = Math.sin(alpha) * magnitude;
        strafe = Math.cos(alpha) * magnitude;

        swerveDrive.holonomicDrive(forward, strafe, rotation);

        FireLog.log("current_angle", Robot.getAngle().getRadians());
        FireLog.log("forward", forward);
        FireLog.log("strafe", strafe);
        FireLog.log("rotation", rotation);
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
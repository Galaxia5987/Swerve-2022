package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class DampedDrive extends CommandBase {

    private static final SlewRateLimiter vxFilter = new SlewRateLimiter(16);
    private static final SlewRateLimiter vyFilter = new SlewRateLimiter(16);
    private static final PIDController thetaController = new PIDController(0.1, 0, 0); // important to add ki
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private double storedYaw;
    private final SwerveDrive swerve;

    public DampedDrive(SwerveDrive swerve, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.swerve = swerve;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        double alpha = Math.atan2(forward, strafe);
        double vector = Math.hypot(forward, strafe);
        vector = Utils.joystickDeadband(vector, Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        vector = Utils.outerDeadzone(vector, Constants.SwerveDrive.OUTER_JOYSTICK_THRESHOLD);
        alpha = Math.toRadians(Utils.angleDeadZones(Math.toDegrees(alpha), Constants.SwerveDrive.JOYSTICK_ANGLE_DEADZONE));
        forward = Math.sin(alpha) * vector;
        strafe = Math.cos(alpha) * vector;

        double rotation = Utils.joystickDeadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        rotation = Utils.outerDeadzone(rotation, Constants.SwerveDrive.OUTER_JOYSTICK_THRESHOLD);

        // turns the joystick values into the heading of the robot
        forward *= Constants.SwerveDrive.VELOCITY_MULTIPLIER;
        strafe *= Constants.SwerveDrive.VELOCITY_MULTIPLIER;
        rotation *= Constants.SwerveDrive.ROTATION_MULTIPLIER;

        forward = vxFilter.calculate(forward);
        strafe = vyFilter.calculate(strafe);

        if (rotation != 0)
            storedYaw = Robot.navx.getYaw();
        else if (forward != 0 || strafe != 0) {
            rotation = thetaController.calculate(Robot.navx.getYaw(), storedYaw);
        }

        swerve.holonomicDrive(forward, strafe, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}

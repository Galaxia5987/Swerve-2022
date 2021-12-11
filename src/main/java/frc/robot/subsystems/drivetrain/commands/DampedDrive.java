package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class DampedDrive extends CommandBase {

    private static final SlewRateLimiter vxFilter = new SlewRateLimiter(0.5);
    private static final SlewRateLimiter vyFilter = new SlewRateLimiter(0.5);
    private static final PIDController omegaController = new PIDController(0, 0, 0); // important to add ki
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private double lastAngle;
    private double lastTime;
    private Timer timer = new Timer();
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
        lastAngle = Robot.navx.getYaw();
        lastTime = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double forward = vxFilter.calculate(forwardSupplier.getAsDouble());
        double strafe = vyFilter.calculate(strafeSupplier.getAsDouble());

        double alpha = Math.atan2(forward, strafe);
        double vector = Math.hypot(forward, strafe);
        forward = Math.sin(alpha) * vector;
        strafe = Math.cos(alpha) * vector;

        double rotationSetpoint = Utils.joystickDeadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        if (vector < Constants.SwerveDrive.JOYSTICK_THRESHOLD) {
            forward = 0;
            strafe = 0;
        }

        // turns the joystick values into the heading of the robot
        forward *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        strafe *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        rotationSetpoint *= Constants.SwerveDrive.ROTATION_MULTIPLIER;

        double currentAngle = Robot.navx.getYaw();
        double currentTime = timer.get();

        double rotation = omegaController.calculate(Math.toRadians(currentAngle - lastAngle) / (currentTime - lastTime), rotationSetpoint);

        swerve.holonomicDrive(forward, strafe, rotation);
        lastAngle = currentAngle;
        lastTime = currentTime;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}

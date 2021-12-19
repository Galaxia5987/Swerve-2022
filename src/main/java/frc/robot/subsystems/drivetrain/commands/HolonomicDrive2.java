package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;
import frc.robot.valuetuner.WebConstant;
import webapp.FireLog;

import java.util.function.DoubleSupplier;

public class HolonomicDrive2 extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;

    private final SlewRateLimiter vxFilter = new SlewRateLimiter(1);
    private final SlewRateLimiter vyFilter = new SlewRateLimiter(1);

    private final PIDController forwardController = new PIDController(0.12, 0, 0);
    private final PIDController strafeController = new PIDController(0.12, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(50, 0, 2, new TrapezoidProfile.Constraints(Constants.SwerveDrive.ROTATION_MULTIPLIER, Constants.SwerveDrive.ROTATION_MULTIPLIER / 2));

    private final WebConstant kp = new WebConstant("kp", 0.12);
    private final WebConstant ki = new WebConstant("ki", 0);
    private final WebConstant kd = new WebConstant("kd", 0);
    private double storedYaw;

    public HolonomicDrive2(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        storedYaw = Robot.getAngle().getRadians();
        thetaController.reset(Robot.getAngle().getRadians(), swerveDrive.getChassisSpeeds().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        forwardController.setPID(kp.get(), ki.get(), kd.get());
        strafeController.setPID(kp.get(), ki.get(), kd.get());
        // get the values
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();
        double rotation = Utils.deadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD) * Constants.SwerveDrive.ROTATION_MULTIPLIER;
        ChassisSpeeds currentState = swerveDrive.getChassisSpeeds();

        // recalculate - update based on the angle and the magnitude
        double alpha = Math.atan2(forward, strafe); // direction of movement
        double magnitude = Utils.deadband(Math.hypot(forward, strafe), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        forward = vxFilter.calculate(Math.sin(alpha) * magnitude) * Constants.SwerveDrive.VELOCITY_MULTIPLIER;
        strafe = vyFilter.calculate(Math.cos(alpha) * magnitude) * Constants.SwerveDrive.VELOCITY_MULTIPLIER;

        if (rotation != 0 || Math.abs(currentState.omegaRadiansPerSecond) > 0.05) {
            storedYaw = Robot.getAngle().getRadians();
        } else if (forward != 0 || strafe != 0) { // moves without rotating, keep the same angle.
            rotation += thetaController.calculate(Robot.getAngle().getRadians(), storedYaw);
        }
        FireLog.log("current_forward", currentState.vxMetersPerSecond);
        FireLog.log("target_angle", forward);

        forward += forwardController.calculate(currentState.vxMetersPerSecond, forward);
        strafe += strafeController.calculate(currentState.vyMetersPerSecond, strafe);

        // drive
        swerveDrive.holonomicDrive(forward, strafe, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
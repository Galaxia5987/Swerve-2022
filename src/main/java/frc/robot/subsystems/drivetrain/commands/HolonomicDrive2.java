package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;
import webapp.FireLog;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/*TODO: multiply the vector instead
        try timer
        better pid for theta
        commented out stuff
        mess with acceleration
*/
public class HolonomicDrive2 extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier resetClicked;

    private final ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 1, new TrapezoidProfile.Constraints(Constants.SwerveDrive.ROTATION_MULTIPLIER, Constants.SwerveDrive.ROTATION_MULTIPLIER / 2));
    private double storedYaw;
    private double lastRotationTime;

    public HolonomicDrive2(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier resetClicked) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.resetClicked = resetClicked;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        storedYaw = Robot.getAngle().getRadians();
        thetaController.reset(0, swerveDrive.getChassisSpeeds().omegaRadiansPerSecond);
        lastRotationTime = Timer.getFPGATimestamp();
        thetaController.setTolerance(Math.toRadians(0.05));
    }

    @Override
    public void execute() {
        //TODO: Fix the bug where the angle motors go crazy after the robot stops moving.
        if (resetClicked.getAsBoolean()) {
            Robot.resetAngle();
            thetaController.reset(0, swerveDrive.getChassisSpeeds().omegaRadiansPerSecond);
            storedYaw = Robot.getAngle().getRadians();
        }
        // get the values
        double forward = Utils.deadband(forwardSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double strafe = Utils.deadband(strafeSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double rotation = Utils.rotationalDeadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD) * Constants.SwerveDrive.ROTATION_MULTIPLIER;

        // recalculate - update based on the angle and the magnitude
        double alpha = Math.atan2(forward, strafe); // direction of movement
        double magnitude = Math.hypot(forward, strafe) * Constants.SwerveDrive.VELOCITY_MULTIPLIER;
        forward = Math.sin(alpha) * magnitude;
        strafe = Math.cos(alpha) * magnitude;


        if (rotation == 0) {
            if (Timer.getFPGATimestamp() - lastRotationTime > 0.1) {
                double rotationFF = Utils.clamp(thetaController.calculate(Robot.getAngle().getRadians(), storedYaw), -6, 6);
                if (forward == 0 && strafe == 0)
                    rotationFF = Utils.deadband(rotationFF, 1.4);
                rotation += rotationFF;
            } else {
                storedYaw = Robot.getAngle().getRadians();
            }
        } else {
            storedYaw = Robot.getAngle().getRadians();
            lastRotationTime = Timer.getFPGATimestamp();
        }
        swerveDrive.holonomicDrive(forward, strafe, rotation);

        FireLog.log("current_angle", Robot.getAngle().getRadians());
        FireLog.log("target_angle", storedYaw);

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
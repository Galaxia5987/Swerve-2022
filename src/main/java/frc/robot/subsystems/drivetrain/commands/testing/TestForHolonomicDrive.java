package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj.Timer;
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/*TODO: multiply the vector instead
        try timer
        better pid for theta
        commented out stuff
        mess with acceleration
*/
public class TestForHolonomicDrive2 extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier resetClicked;

    private final PIDController forwardController = new PIDController(0.12, 0, 0);
    private final PIDController strafeController = new PIDController(0.12, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(50, 0, 2, new TrapezoidProfile.Constraints(Constants.SwerveDrive.ROTATION_MULTIPLIER, Constants.SwerveDrive.ROTATION_MULTIPLIER / 2));

    private final WebConstant kp = new WebConstant("kp", 0.12);
    private final WebConstant ki = new WebConstant("ki", 0);
    private final WebConstant kd = new WebConstant("kd", 0);
    private final Timer timer = new Timer();
    private double storedYaw;

    public TestForHolonomicDrive2(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier resetClicked) {
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
    }

    @Override
    public void execute() {
        if (resetClicked.getAsBoolean()) {
            Robot.resetAngle();
            thetaController.reset(0, swerveDrive.getChassisSpeeds().omegaRadiansPerSecond);
        }
        forwardController.setPID(kp.get(), ki.get(), kd.get());
        strafeController.setPID(kp.get(), ki.get(), kd.get());
        // get the values
        double forward = Utils.deadband(forwardSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double strafe = Utils.deadband(strafeSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double rotation = Utils.rotationalDeadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD) * Constants.SwerveDrive.ROTATION_MULTIPLIER;
        ChassisSpeeds currentState = swerveDrive.getChassisSpeeds();

        // recalculate - update based on the angle and the magnitude
        double alpha = Math.atan2(forward, strafe); // direction of movement

        double magnitude = Math.hypot(forward, strafe) * Constants.SwerveDrive.VELOCITY_MULTIPLIER;
        forward = Math.sin(alpha) * magnitude;
        strafe = Math.cos(alpha) * magnitude;

        if (rotation != 0 || Math.abs(currentState.omegaRadiansPerSecond) > 0.05) {
            storedYaw = Robot.getAngle().getRadians();
        } else if (forward != 0 || strafe != 0) { // moves without rotating, keep the same angle.
//            rotation += thetaController.calculate(Robot.getAngle().getRadians(), storedYaw);
//            forward += forwardController.calculate(currentState.vxMetersPerSecond, forward);
//            strafe += strafeController.calculate(currentState.vyMetersPerSecond, strafe);
        }
        // drive
        swerveDrive.holonomicDrive(forward, strafe, rotation);

        System.out.println("angle to be at: " + storedYaw);

        System.out.println("error for rotation: " + (thetaController.calculate(Robot.getAngle().getRadians(), storedYaw)));

        System.out.println("error for forward: " + (forwardController.calculate(currentState.vxMetersPerSecond, forward)));

        System.out.println("error for strafe: " + (strafeController.calculate(currentState.vyMetersPerSecond, strafe)));
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
package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class LockOnTarget extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final Supplier<Rotation2d> rotationSupplier;
    private final ProfiledPIDController thetaController = new ProfiledPIDController(10, 0, 2, new TrapezoidProfile.Constraints(Constants.SwerveDrive.ROTATION_MULTIPLIER, Constants.SwerveDrive.ROTATION_MULTIPLIER / 2));
    Rotation2d desiredRotation = new Rotation2d(0);


    public LockOnTarget(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, Supplier<Rotation2d> rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveDrive);
    }


    @Override
    public void execute() {
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        double alpha = Math.atan2(forward, strafe);
        double vector = Math.hypot(forward, strafe);


        vector *= Constants.SwerveDrive.VELOCITY_MULTIPLIER;

        forward = Math.sin(alpha) * vector;
        strafe = Math.cos(alpha) * vector;

        if (rotationSupplier.get() != null) {
            desiredRotation = rotationSupplier.get();
        }

        double rotation = thetaController.calculate(Robot.getAngle().getRadians(), desiredRotation.getRadians());

        if (vector < Constants.SwerveDrive.JOYSTICK_THRESHOLD) {
            forward = 0;
            strafe = 0;
        }

        if (forward != 0 || strafe != 0 || rotation != 0) {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
        } else {
            swerveDrive.terminate();
        }
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

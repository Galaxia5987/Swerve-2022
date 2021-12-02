package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class FineTunedDrive extends CommandBase {
    public static final PIDController driveController = new PIDController(0.08, 0, 0);
    public static final PIDController rotationController = new PIDController(0.1, 0, 0);
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private double referenceAngle;
    private boolean isAngleSet = false;


    public FineTunedDrive(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        rotationController.enableContinuousInput(-180, 180);
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    public static double getForwardFlex(double desiredForward, double realForward) {
        return desiredForward + driveController.calculate(realForward, desiredForward);
    }

    public static double getStrafeFlex(double desiredStrafe, double realStrafe) {
        return desiredStrafe + driveController.calculate(realStrafe, desiredStrafe);
    }

    public static double getRotationFlex(double desiredFlex) {
        return desiredFlex;
    }

    @Override
    public void execute() {
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();
        double alpha = Math.atan2(forward, strafe);
        double vector = Math.hypot(forward, strafe);
        if (Math.abs(vector) < 0.1) vector = 0;
        if (Math.abs(rotation) < 0.1) rotation = 0;
        vector = vector < 0 ? -Math.pow(vector, 2) : Math.pow(vector, 2);
        double checking_vector = vector;
        vector *= Constants.SwerveDrive.SPEED_MULTIPLIER * 0.35;
        rotation *= Constants.SwerveDrive.ROTATION_MULTIPLIER * 0.75;
        forward = Math.sin(alpha) * vector;
        strafe = Math.cos(alpha) * vector;

        if (Utils.isPaused(forward, strafe, rotation)) {
            swerveDrive.terminate();
            isAngleSet = false;
            return;
        }

        if (Utils.isProblematic(checking_vector, rotation)) {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
            isAngleSet = false;
            return;
        }

        if (Utils.isStraightLine(forward, strafe, rotation)) {
            if (!isAngleSet) {
                referenceAngle = Robot.navx.getYaw();
                isAngleSet = true;
            }
            swerveDrive.holonomicDrive(
                    getForwardStraightLine(forward, swerveDrive.getChassisSpeeds().vxMetersPerSecond),
                    getStrafeStraightLine(strafe, swerveDrive.getChassisSpeeds().vyMetersPerSecond),
                    getRotationStraightLine(referenceAngle, Robot.navx.getYaw()));
            return;
        }
        if (Utils.isRotationOnly(forward, strafe, rotation)) {
            swerveDrive.holonomicDrive(
                    getForwardRotationOnly(swerveDrive.getChassisSpeeds().vxMetersPerSecond),
                    getStrafeRotationOnly(swerveDrive.getChassisSpeeds().vyMetersPerSecond),
                    rotation);
            isAngleSet = false;
            return;
        }
        if (Utils.isFlex(forward, strafe, rotation)) {
            swerveDrive.holonomicDrive(
                    getForwardFlex(forward, swerveDrive.getChassisSpeeds().vxMetersPerSecond),
                    getStrafeFlex(strafe, swerveDrive.getChassisSpeeds().vyMetersPerSecond),
                    getRotationFlex(rotation));
            isAngleSet = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }

    public double getForwardStraightLine(double desiredForward, double realForward) {
        return desiredForward + driveController.calculate(realForward, desiredForward);
    }

    public double getStrafeStraightLine(double desiredStrafe, double realStrafe) {
        return desiredStrafe + driveController.calculate(realStrafe, desiredStrafe);
    }

    public double getRotationStraightLine(double referenceAngle, double currentAngle) {
        return rotationController.calculate(currentAngle, referenceAngle);
    }

    public double getForwardRotationOnly(double realForward) {
        return driveController.calculate(realForward, 0);
    }

    public double getStrafeRotationOnly(double realStrafe) {
        return driveController.calculate(realStrafe, 0);
    }
}

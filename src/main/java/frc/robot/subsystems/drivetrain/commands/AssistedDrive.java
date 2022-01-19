package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class AssistedDrive extends CommandBase {
    private static final double SPEED_MULTIPLIER = 3;
    private static final double ROTATION_MULTIPLIER = 3;
    private static final double MIN_CARGO_DISTANCE = 5;
    private static final Translation2d HUB_POSITION = new Translation2d(6, 3);
    private final SwerveDrive swerve;
    private final DoubleSupplier cargoYaw;
    private final DoubleSupplier estimatedCargoDistance;
    private final DoubleSupplier hubYaw;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier pickupSupplier;
    private final BooleanSupplier aimSupplier;
    private final BooleanSupplier rungAdjustSupplier;

    private final PIDController pidController;
    private final ProfiledPIDController profiledPIDController;

    public AssistedDrive(SwerveDrive swerve, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier pickupSupplier, BooleanSupplier aimSupplier, BooleanSupplier rungAdjustSupplier, DoubleSupplier cargoYaw, DoubleSupplier estimatedCargoDistance, DoubleSupplier hubYaw, PIDController pidController, ProfiledPIDController profiledPIDController) {
        this.swerve = swerve;
        this.cargoYaw = cargoYaw;
        this.estimatedCargoDistance = estimatedCargoDistance;
        this.hubYaw = hubYaw;
        this.pidController = pidController;
        this.profiledPIDController = profiledPIDController;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.pickupSupplier = pickupSupplier;
        this.aimSupplier = aimSupplier;
        this.rungAdjustSupplier = rungAdjustSupplier;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double forward = -forwardSupplier.getAsDouble();
        double strafe = -strafeSupplier.getAsDouble();
        double magnitude = Math.hypot(forward, strafe);
        double alpha = Math.atan2(strafe, forward);
        if (Math.abs(magnitude) < 0.1) magnitude = 0;
        magnitude *= SPEED_MULTIPLIER;
        forward = Math.cos(alpha) * magnitude;
        strafe = Math.sin(alpha) * magnitude;
        double rotation = -rotationSupplier.getAsDouble();
        if (Math.abs(rotation) < 0.1) rotation = 0;
        rotation *= ROTATION_MULTIPLIER;

        if (forward == 0 && strafe == 0 && rotation == 0) {
            swerve.terminate();
        } else {
            if (pickupSupplier.getAsBoolean()) {
                if (cargoYaw != null && estimatedCargoDistance != null) {
                    if (estimatedCargoDistance.getAsDouble() <= MIN_CARGO_DISTANCE && rotation == 0) {
                        swerve.holonomicDrive(forward, strafe, pidController.calculate(cargoYaw.getAsDouble()));
                        return;
                    }
                }
            }
            if (aimSupplier.getAsBoolean()) {
                if (hubYaw == null) {
                    if (rotation == 0) {
                        Translation2d diff = swerve.getPose().getTranslation().minus(HUB_POSITION);
                        double angle = Math.toDegrees(Math.atan2(diff.getY(), diff.getX()));
                        swerve.holonomicDrive(forward, strafe, profiledPIDController.calculate(-Robot.getAngle().getDegrees(), angle));
                        return;
                    }
                } else {
                    if (rotation == 0) {
                        swerve.holonomicDrive(forward, strafe, pidController.calculate(hubYaw.getAsDouble()));
                        return;
                    }
                }
            }
            if (rungAdjustSupplier.getAsBoolean()) {
                if (rotation == 0) {
                    swerve.holonomicDrive(forward, strafe, profiledPIDController.calculate(-Robot.getAngle().getDegrees(), 90));
                    return;
                }
            }
            swerve.holonomicDrive(forward, strafe, rotation);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.terminate();
    }


}

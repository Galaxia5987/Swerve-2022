package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;


public class PickupCargo extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier speed;
    private final DoubleSupplier cargoYaw;

    /**
     * Pickup a cargo.
     *
     * @param swerveDrive the swerveDrive.
     * @param speed       the speed we should come to the cargo. [m/s]
     * @param cargoYaw    the yaw of the cargo relative to the robot. [degrees]
     */
    public PickupCargo(SwerveDrive swerveDrive, DoubleSupplier speed, DoubleSupplier cargoYaw) {
        this.swerveDrive = swerveDrive;
        this.speed = speed;
        this.cargoYaw = cargoYaw;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double currentSpeed = speed.getAsDouble();
        Rotation2d currentCargoYaw = Rotation2d.fromDegrees(cargoYaw.getAsDouble())
                .plus(Robot.getAngle()); // field relative angle
        double forward = currentSpeed * currentCargoYaw.getCos();
        double strafe = currentSpeed * currentCargoYaw.getSin();
        swerveDrive.holonomicDriveKeepSetpoint(forward, strafe, currentCargoYaw);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}

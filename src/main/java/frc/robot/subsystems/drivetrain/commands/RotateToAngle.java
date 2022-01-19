package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * Rotates the entire chassis to a specified angle.
 */
public class RotateToAngle extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final Rotation2d targetAngle;

    /**
     * Initialize rotate to angle command.
     *
     * @param swerveDrive the SwerveDrive subsystem
     * @param targetAngle the target angle. [rad]
     */
    public RotateToAngle(SwerveDrive swerveDrive, double targetAngle) {
        this.swerveDrive = swerveDrive;
        this.targetAngle = new Rotation2d(targetAngle);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.holonomicRotation(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return Robot.getAngle().minus(targetAngle).getRadians() < Constants.SwerveDrive.ALLOWABLE_THETA_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}

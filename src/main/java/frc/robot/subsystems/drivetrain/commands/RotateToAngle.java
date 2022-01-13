package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * Rotates the entire chassis to a specified angle.
 */
public class RotateToAngle extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final ProfiledPIDController anglePID = new ProfiledPIDController(Constants.SwerveDrive.KP_TURN,
            Constants.SwerveDrive.KI_TURN, Constants.SwerveDrive.KD_TURN,
            new TrapezoidProfile.Constraints(Constants.SwerveDrive.ROTATION_MULTIPLIER, Constants.SwerveDrive.ROTATION_MULTIPLIER / 2)
    );
    private final double targetAngle;

    /**
     * Initialize rotate to angle command.
     *
     * @param swerveDrive the SwerveDrive subsystem
     * @param targetAngle the target angle. [rad]
     */
    public RotateToAngle(SwerveDrive swerveDrive, double targetAngle) {
        this.swerveDrive = swerveDrive;
        this.targetAngle = targetAngle;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        anglePID.reset(Robot.getAngle().getRadians());
        anglePID.setGoal(targetAngle);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        anglePID.setTolerance(Math.toRadians(1));
    }

    @Override
    public void execute() {
        swerveDrive.holonomicDrive(0, 0, anglePID.calculate(Robot.getAngle().getRadians()));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }

    @Override
    public boolean isFinished() {
        return anglePID.atSetpoint();
    }
}

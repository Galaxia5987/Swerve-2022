package frc.robot.subsystems.drivetrain.commands.tuning;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.WebConstant;
import webapp.FireLog;

public class Rotate extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final WebConstant targetAngle = new WebConstant("targetAngle", 0);

    public Rotate(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
//        double rotation = -RobotContainer.Xbox.getY();
//        rotation = Utils.joystickDeadband(rotation, Constants.SwerveDrive.JOYSTICK_THRESHOLD);

        swerveDrive.getModule(0).setAngle(Rotation2d.fromDegrees(targetAngle.get()));
        swerveDrive.getModule(1).setAngle(Rotation2d.fromDegrees(targetAngle.get()));
        swerveDrive.getModule(2).setAngle(Rotation2d.fromDegrees(targetAngle.get()));
        swerveDrive.getModule(3).setAngle(Rotation2d.fromDegrees(targetAngle.get()));

        FireLog.log("angle-setpoint", targetAngle.get());
        FireLog.log("module FR", swerveDrive.getModule(0).getAngle().getDegrees());
        FireLog.log("module FL", swerveDrive.getModule(1).getAngle().getDegrees());
        FireLog.log("module RR", swerveDrive.getModule(2).getAngle().getDegrees());
        FireLog.log("module RL", swerveDrive.getModule(3).getAngle().getDegrees());
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

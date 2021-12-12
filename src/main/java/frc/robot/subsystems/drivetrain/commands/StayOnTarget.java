package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.BooleanSupplier;

public class StayOnTarget extends CommandBase {
    private SwerveDrive swerveDrive;
    private Pose2d target_pose;
    private Pose2d current_pose;
    private double curr_angle;
    private double req_angle;
    private BooleanSupplier isActive;

    public StayOnTarget(SwerveDrive swerveDrive, Pose2d target_pose, BooleanSupplier isActive) {
        this.swerveDrive = swerveDrive;
        this.target_pose = target_pose;
        this.isActive = isActive;
    }

    @Override
    public void execute() {
        curr_angle = Robot.navx.getAngle() % 360;
        current_pose = swerveDrive.getPose();

        req_angle = Math.atan(
                current_pose.getY() - target_pose.getY() /
                (current_pose.getX() - target_pose.getX())
        );

        double d_angle = Math.toRadians(req_angle - curr_angle);
        double rotation_velocity = d_angle / Constants.LOOP_PERIOD;

        if (isActive.getAsBoolean()) {
            swerveDrive.setRotation(rotation_velocity);
        }
    }
}

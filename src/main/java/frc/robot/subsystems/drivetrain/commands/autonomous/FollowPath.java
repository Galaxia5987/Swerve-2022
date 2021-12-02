package frc.robot.subsystems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class FollowPath extends SwerveControllerCommand {

    /**
     * For clamped cubic splines, this method accepts two Pose2d objects, one for
     * the starting waypoint and one for the ending waypoint. The method takes in a
     * vector of Translation2d objects which represent the interior waypoints. The
     * headings at these interior waypoints are determined automatically to ensure
     * continuous curvature. For quintic splines, the method simply takes in a list
     * of Pose2d objects, with each Pose2d representing a point and heading on the
     * field.
     */

    private final SwerveDrive swerveDrive;

    public FollowPath(SwerveDrive swerveDrive, Trajectory trajectory) {
        super(
                trajectory,
                swerveDrive::getPoseForTrajectory,
                swerveDrive.kinematics,
                new PIDController(Constants.Autonomous.kPXController, 0, 0),
                new PIDController(Constants.Autonomous.kPYController, 0, 0),
                new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0, Constants.Autonomous.kThetaControllerConstraints) {{
                    enableContinuousInput(-Math.PI, Math.PI);
                }},
                (SwerveModuleState[] states) -> {
                    for (int i = 0; i < states.length; i++) {
                        swerveDrive.getModule(i).setState(
                                new SwerveModuleState(states[i].speedMetersPerSecond,
                                        new Rotation2d(-states[i].angle.getRadians())));
                    }
                },
                swerveDrive);
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveDrive.terminate();
    }
}

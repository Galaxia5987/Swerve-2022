package frc.robot.subsystems.drivetrain.commands.autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.pathplanner.PathPlanner;
import frc.robot.pathplanner.PathPlannerTrajectory;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class FollowPath extends CommandBase {
    private final Timer timer = new Timer();
    private final SwerveDrive swerveDrive;
    private final PathPlannerTrajectory target;
    private final HolonomicDriveController hController;

    public FollowPath(SwerveDrive swerveDrive, String path, double maxVel, double maxAcceleration,
                      PIDController forwardPID, PIDController strafePID, ProfiledPIDController rotationPID) {
        this.swerveDrive = swerveDrive;
        target = PathPlanner.loadPath(path, maxVel, maxAcceleration);
        hController = new HolonomicDriveController(forwardPID, strafePID, rotationPID);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        Robot.resetAngle();
        swerveDrive.resetOdometry(target.getInitialPose(), target.getInitialState().holonomicRotation);
    }

    @Override
    public void execute() {
        var curTime = timer.get();
        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) target.sample(curTime);
        Pose2d currentPosition = swerveDrive.getPose();
        ChassisSpeeds speeds = hController.calculate(currentPosition, state, state.holonomicRotation);
        swerveDrive.setStates(swerveDrive.getKinematics().toSwerveModuleStates(speeds));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(target.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerveDrive.terminate();
    }

}
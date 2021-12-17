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
    private final PIDController forwardPID;
    private final PIDController strafePID;
    private final ProfiledPIDController rotationPID;
    private final PathPlannerTrajectory target;
    private HolonomicDriveController hController;

    public FollowPath(SwerveDrive swerveDrive, String path, double maxVel, double maxAcceleration,
                      PIDController forwardPID, PIDController strafePID, ProfiledPIDController rotationPID) {
        this.swerveDrive = swerveDrive;

        this.forwardPID = forwardPID;
        this.strafePID = strafePID;
        this.rotationPID = rotationPID;

        target = PathPlanner.loadPath(path, maxVel, maxAcceleration);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        hController = new HolonomicDriveController(forwardPID, strafePID, rotationPID);
        timer.reset();
        timer.start();
        Robot.navx.reset();
        swerveDrive.resetOdometry(target.getInitialPose(), target.getInitialState().holonomicRotation.getDegrees());
    }

    @Override
    public void execute() {
        var curTime = timer.get();
        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) target.sample(curTime);
        Pose2d currentPosition = swerveDrive.getPose();
        ChassisSpeeds speeds = hController.calculate(currentPosition, state, state.holonomicRotation);
        swerveDrive.setStates(swerveDrive.getKinematics().toSwerveModuleStates(speeds));
//        System.out.println(speeds);
//        System.out.println(target.getTotalTimeSeconds());
//        System.out.println(swerveDrive.getPose());
        System.out.println(currentPosition + " " + state.poseMeters);
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
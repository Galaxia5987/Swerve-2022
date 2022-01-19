package frc.robot.subsystems.drivetrain.commands.autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.pathplanner.PathPlanner;
import frc.robot.pathplanner.PathPlannerTrajectory;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class FollowPath extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();
    private final PIDController forwardPID;
    private final PIDController strafePID;
    private final ProfiledPIDController rotationPID;
    private final PathPlannerTrajectory target;
    private HolonomicDriveController hController;

    public FollowPath(SwerveDrive swerve, String path, double maxVel, double maxAcceleration,
                      PIDController forwardPID, PIDController strafePID, ProfiledPIDController rotationPID) {
        this.swerve = swerve;

        this.forwardPID = forwardPID;
        this.strafePID = strafePID;
        this.rotationPID = rotationPID;

        target = PathPlanner.loadPath(path, maxVel, maxAcceleration);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        hController = new HolonomicDriveController(forwardPID, strafePID, rotationPID);
        timer.reset();
        timer.start();
        Robot.resetAngle(Rotation2d.fromDegrees(target.getInitialState().holonomicRotation.getDegrees()));
        swerve.resetOdometry(new Pose2d(target.getInitialState().poseMeters.getTranslation(), target.getInitialState().holonomicRotation), target.getInitialState().holonomicRotation);
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) target.sample(currentTime);
        Pose2d currentPosition = swerve.getPose();
        ChassisSpeeds speeds = hController.calculate(currentPosition, state, state.holonomicRotation);
        swerve.setStates(swerve.getKinematics().toSwerveModuleStates(speeds));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(target.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.terminate();
    }
}
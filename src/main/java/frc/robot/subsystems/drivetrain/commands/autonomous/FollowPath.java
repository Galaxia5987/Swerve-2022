package frc.robot.subsystems.drivetrain.commands.autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.pathplanner.PathPlanner;
import frc.robot.pathplanner.PathPlannerTrajectory;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class FollowPath extends CommandBase {
    private final Timer timer = new Timer();
    private final SwerveDrive swerveDrive;
    private final boolean useRamsete;
    private final PathPlannerTrajectory target;
    private final HolonomicDriveController hController;
    private final RamseteController controller = new RamseteController(Constants.Autonomous.kB, Constants.Autonomous.kZeta);

    public FollowPath(SwerveDrive swerveDrive, String path, double maxVel, double maxAcceleration,
                      PIDController forwardPID, PIDController strafePID, ProfiledPIDController rotationPID,
                      boolean useRamsete) {
        this.swerveDrive = swerveDrive;
        this.useRamsete = useRamsete;

        target = PathPlanner.loadPath(path, maxVel, maxAcceleration);

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        hController = new HolonomicDriveController(forwardPID, strafePID, rotationPID);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Robot.resetAngle();
        timer.reset();
        timer.start();

        swerveDrive.resetOdometry(target.getInitialPose(), target.getInitialState().holonomicRotation);
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        var state = (PathPlannerTrajectory.PathPlannerState) target.sample(curTime);
        Pose2d currentPosition = swerveDrive.getPose();


        ChassisSpeeds speeds = hController.calculate(currentPosition, state, state.holonomicRotation);
        if (useRamsete) {
            ChassisSpeeds ramseteSpeeds = controller.calculate(currentPosition, state);
            speeds = new ChassisSpeeds(ramseteSpeeds.vxMetersPerSecond, ramseteSpeeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
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
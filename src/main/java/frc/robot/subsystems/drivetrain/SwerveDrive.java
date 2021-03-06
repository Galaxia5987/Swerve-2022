package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.TimeDelayedBoolean;


/**
 * The {@code SwerveDrive} Subsystem is responsible for the integration of modules together in order to move the robot honolomicaly.
 * The class contains several convenient methods for controlling the robot and retrieving information about his state.
 * <p>
 * The subsystem has the capability to work in both field oriented and robot oriented mode.
 */
public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive FIELD_ORIENTED_INSTANCE = null;
    private static SwerveDrive ROBOT_ORIENTED_INSTANCE = null;

    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.SwerveDrive.SWERVE_POSITIONS);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Robot.getAngle(), new Pose2d());
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.SwerveDrive.THETA_KP.get(),
            Constants.SwerveDrive.THETA_KI.get(),
            Constants.SwerveDrive.THETA_KD.get(),
            new TrapezoidProfile.Constraints(Constants.SwerveDrive.ROTATION_MULTIPLIER, Constants.SwerveDrive.ROTATION_MULTIPLIER / 2)
    );
    private final TimeDelayedBoolean rotationDelay = new TimeDelayedBoolean();
    private final boolean fieldOriented;

    public SwerveDrive(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
        modules[0] = new SwerveModule(Constants.SwerveModule.frConfig);
        modules[1] = new SwerveModule(Constants.SwerveModule.flConfig);
        modules[2] = new SwerveModule(Constants.SwerveModule.rrConfig);
        modules[3] = new SwerveModule(Constants.SwerveModule.rlConfig);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.reset(0, 0);
        thetaController.setTolerance(Constants.SwerveDrive.ALLOWABLE_THETA_ERROR);
    }

    /**
     * @return the swerve in robot oriented mode.
     */
    public static SwerveDrive getRobotOrientedInstance() {
        if (ROBOT_ORIENTED_INSTANCE == null) {
            ROBOT_ORIENTED_INSTANCE = new SwerveDrive(false);
        }
        return ROBOT_ORIENTED_INSTANCE;
    }

    /**
     * @return the swerve in field oriented mode.
     */
    public static SwerveDrive getFieldOrientedInstance() {
        if (FIELD_ORIENTED_INSTANCE == null) {
            FIELD_ORIENTED_INSTANCE = new SwerveDrive(true);
        }
        return FIELD_ORIENTED_INSTANCE;
    }

    /**
     * Gets the kinematics of the swerve.
     *
     * @return the kinematics of the swerve.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Move the swerve in the specified direction, rotation and velocity.
     *
     * @param forward  the velocity on the Y-axis. [m/s]
     * @param strafe   the velocity on the X-axis. [m/s]
     * @param rotation rhe rotational velocity. [rad/s]
     */
    public void holonomicDrive(double forward, double strafe, double rotation) {
        if (rotation == 0 &&
                rotationDelay.update(Math.abs(thetaController.getGoal().position - Robot.getAngle()
                                .getRadians()) < Constants.SwerveDrive.ALLOWABLE_THETA_ERROR,
                        Constants.SwerveDrive.ROTATION_DELAY)
        ) {
            rotation = thetaController.calculate(Robot.getAngle().getRadians());
        } else {
            thetaController.reset(Robot.getAngle().getRadians());
        }
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Robot.getAngle()) :
                new ChassisSpeeds(forward, strafe, rotation);

        setStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Rotates the robot to a desired angle.
     *
     * @param desiredAngle the desired angle of the robot.
     */
    public void holonomicDriveKeepSetpoint(double forward, double strafe, Rotation2d desiredAngle) {
        thetaController.setGoal(desiredAngle.getRadians());
        double output = thetaController.calculate(Robot.getAngle().getRadians());

        setStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, output)));
    }
    /**
     * Gets te states of every module.
     *
     * @return the states of every module.
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[modules.length];
        for (SwerveModule module : modules) {
            swerveModuleState[module.getWheel()] = module.getState();
        }
        return swerveModuleState;
    }

    /**
     * Sets the state of the modules.
     *
     * @param states the states of the modules.
     */
    public void setStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.SwerveDrive.VELOCITY_MULTIPLIER);
        for (SwerveModule module : modules) {
            SwerveModuleState state = states[module.getWheel()];
            state = SwerveModuleState.optimize(state, module.getAngle());
            module.setState(state);
        }
    }

    /**
     * Gets the chassis speeds of the entire robot.
     *
     * @return the speed of the robot in each axis.
     */
    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getStates());
        if (fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    chassisSpeeds.omegaRadiansPerSecond,
                    Robot.getAngle().unaryMinus()
            );
        }
        return chassisSpeeds;
    }

    /**
     * Gets a specific module, shouldn't be used for regular cases.
     *
     * @param index the index of the module.
     * @return the corresponding module.
     */
    public SwerveModule getModule(int index) {
        return modules[index];
    }

    /**
     * Gets the pose of the robot.
     *
     * @return the pose of the robot.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry.
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d(), new Rotation2d());
    }

    /**
     * Resets the odometry to a specified position.
     *
     * @param pose     the current pose.
     * @param rotation the holonomic rotation.
     */
    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        odometry.resetPosition(new Pose2d(pose.getTranslation(), rotation), rotation);
    }

    /**
     * Resets the theta controller target angle.
     */
    public void resetThetaController() {
        thetaController.reset(0, getChassisSpeeds().omegaRadiansPerSecond);
    }

    /**
     * Terminates the modules from moving.
     */
    public void terminate() {
        for (SwerveModule module : modules) {
            module.stopDriveMotor();
            module.stopAngleMotor();
        }
    }

    @Override
    public void periodic() {
        odometry.updateWithTime(Timer.getFPGATimestamp(),
                Robot.getAngle(),
                getStates()
        );
/*
        thetaController.setP(Constants.SwerveDrive.THETA_KP.get());
        thetaController.setI(Constants.SwerveDrive.THETA_KI.get());
        thetaController.setD(Constants.SwerveDrive.THETA_KD.get());
*/
    }
}

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends SubsystemBase {
    private static final double Rx = Constants.SwerveDrive.ROBOT_WIDTH / 2;
    private static final double Ry = Constants.SwerveDrive.ROBOT_LENGTH / 2;
    private static final double[] signX = {1, 1, -1, -1};
    private static final double[] signY = {-1, 1, -1, 1};
    private static SwerveDrive FIELD_RELATIVE_INSTANCE = null;
    private static SwerveDrive INSTANCE = null;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(signX[0] * Rx, -signY[0] * Ry),
            new Translation2d(signX[1] * Rx, -signY[1] * Ry),
            new Translation2d(signX[2] * Rx, -signY[2] * Ry),
            new Translation2d(signX[3] * Rx, -signY[3] * Ry)
    );
    private final SwerveModule[] modules = new SwerveModule[4];
    private final Timer timer = new Timer();
    private final DoubleSupplier angleSupplier = Robot.navx::getYaw;
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
            Rotation2d.fromDegrees(angleSupplier.getAsDouble()), new Pose2d()
    );
    private final boolean fieldOriented;

    public SwerveDrive(boolean fieldOriented) {
        timer.start();
        this.fieldOriented = fieldOriented;
        modules[0] = new SwerveModule(Constants.SwerveModule.frConfig);
        modules[1] = new SwerveModule(Constants.SwerveModule.flConfig);
        modules[2] = new SwerveModule(Constants.SwerveModule.rrConfig);
        modules[3] = new SwerveModule(Constants.SwerveModule.rlConfig);
    }

    /**
     * @return the swerve in robot oriented mode.
     */
    public static SwerveDrive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDrive(false);
        }
        return INSTANCE;
    }

    /**
     * @return the swerve in field oriented mode.
     */
    public static SwerveDrive getFieldRelativeInstance() {
        if (FIELD_RELATIVE_INSTANCE == null) {
            FIELD_RELATIVE_INSTANCE = new SwerveDrive(true);
        }
        return FIELD_RELATIVE_INSTANCE;
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
     * Sets the wheels of the robot to the calculated angle and speed.
     *
     * @param speeds the chassis speeds.
     */
    public void holonomicDrive(ChassisSpeeds speeds) {
        holonomicDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Sets the wheels of the robot to the calculated angle and speed.
     *
     * @param forward  the Y value of the joystick
     * @param strafe   the X value of the joystick
     * @param rotation the rotation Z of the joystick
     */
    public void holonomicDrive(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(angleSupplier.getAsDouble())) :
                new ChassisSpeeds(forward, strafe, rotation);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.SwerveDrive.VELOCITY_MULTIPLIER);
        setStates(states);
    }

    /**
     * Sets the state of the motors.
     *
     * @param states the states of the motors.
     */
    public void setStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], getModule(i).getAngle());
            getModule(i).setState(states[i]);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            swerveModuleStates[i] = new SwerveModuleState(getModule(i).getVelocity(), getModule(i).getAngle());
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(swerveModuleStates);
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(angleSupplier.getAsDouble()));
        return chassisSpeeds;
    }

    /**
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
        odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
    }

    /**
     * Resets the odometry to a specified position.
     *
     * @param pose the current pose.
     */
    public void resetOdometry(Pose2d pose, double holonomicRotation) {
        odometry.resetPosition(new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(holonomicRotation)), Rotation2d.fromDegrees(holonomicRotation));
        ;
    }

    /**
     * Terminates the motors.
     */
    public void terminate() {
        for (var module : modules) {
            module.stopDriveMotor();
            module.stopAngleMotor();
        }
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        for (int i = 0; i < 4; i++) {
            modules[i].setNeutralMode(neutralMode);
        }
    }

    @Override
    public void periodic() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            swerveModuleState[i] = new SwerveModuleState(modules[i].getVelocity(), modules[i].getAngle());
        }
        odometry.updateWithTime(timer.get(),
                new Rotation2d(Math.toRadians(Robot.navx.getYaw())),
                swerveModuleState
        );
    }
}

package frc.robot.utils;

public interface SwerveModuleConfigBase {
    int wheel();

    int driveMotorPort();

    int angleMotorPort();

    boolean driveMotorInverted();

    boolean angleMotorInverted();

    boolean driveMotorSensorPhase();

    boolean angleMotorSensorPhase();

    double angle_kp();

    double angle_ki();

    double angle_kd();

    double angle_kf();

    double j();

    double zeroPosition();

    int motionAcceleration();

    int motionCruiseVelocity();

    int curveStrength();

    double ticksPerMeter();

    double wheelRadius();

    double ticksPerRadian();

    int angleMaxCurrent();

    double velocityTolerance();

    double modelTolerance();

    double encoderTolerance();

    double driveMotorGearRatio();

    boolean debug();

}

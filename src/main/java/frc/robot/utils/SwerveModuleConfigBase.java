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

    boolean debug();
}

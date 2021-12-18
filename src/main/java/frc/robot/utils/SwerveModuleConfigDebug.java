package frc.robot.utils;

import frc.robot.valuetuner.WebConstant;

public class SwerveModuleConfigDebug implements SwerveModuleConfigBase {
    private final int wheel;

    // ports
    private final int driveMotorPort;
    private final int angleMotorPort;

    // inversions
    private final boolean driveMotorInverted;
    private final boolean angleMotorInverted;
    private final boolean driveMotorSensorPhase;
    private final boolean angleMotorSensorPhase;

    // PID
    private final WebConstant angle_kp;
    private final WebConstant angle_ki;
    private final WebConstant angle_kd;
    private final WebConstant angle_kf;

    private final WebConstant j; // moment of inertia [kg * m^2]

    private final double zeroPosition; // [ticks]

    // Motion Magic
    private final int motionAcceleration;
    private final int motionCruiseVelocity;
    private final int curveStrength;

    // drive motor
    private final double ticksPerMeter;
    private final double wheelRadius; // [m]

    // angle motor
    private final double ticksPerRadian;

    // constraints
    private final int angleMaxCurrent; // [amps]
    private final double velocityTolerance; // [RPS]
    private final double modelTolerance;
    private final double encoderTolerance; // [ticks]
    private final double driveMotorGearRatio;

    public SwerveModuleConfigDebug(int wheel, int driveMotorPort, int angleMotorPort,
                                   boolean driveMotorInverted, boolean angleMotorInverted,
                                   boolean driveMotorSensorPhase, boolean angleMotorSensorPhase,
                                   WebConstant angle_kp, WebConstant angle_ki, WebConstant angle_kd, WebConstant angle_kf,
                                   WebConstant j, double zeroPosition, int motionAcceleration, int motionCruiseVelocity, int curveStrength, CommonSwerveConfig commonConfig) {
        this.wheel = wheel;
        this.driveMotorPort = driveMotorPort;
        this.angleMotorPort = angleMotorPort;
        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;
        this.driveMotorSensorPhase = driveMotorSensorPhase;
        this.angleMotorSensorPhase = angleMotorSensorPhase;
        this.angle_kp = angle_kp;
        this.angle_ki = angle_ki;
        this.angle_kd = angle_kd;
        this.angle_kf = angle_kf;
        this.j = j;
        this.zeroPosition = zeroPosition;
        this.motionAcceleration = motionAcceleration;
        this.motionCruiseVelocity = motionCruiseVelocity;
        this.curveStrength = curveStrength;
        this.ticksPerMeter = commonConfig.ticksPerMeter;
        this.wheelRadius = commonConfig.wheelRadius;
        this.ticksPerRadian = commonConfig.ticksPerRadian;
        this.angleMaxCurrent = commonConfig.angleMaxCurrent;
        this.velocityTolerance = commonConfig.velocityTolerance;
        this.modelTolerance = commonConfig.modelTolerance;
        this.encoderTolerance = commonConfig.encoderTolerance;
        this.driveMotorGearRatio = commonConfig.driveMotorGearRatio;
    }

    @Override
    public int wheel() {
        return wheel;
    }

    @Override
    public int driveMotorPort() {
        return driveMotorPort;
    }

    @Override
    public int angleMotorPort() {
        return angleMotorPort;
    }

    @Override
    public boolean driveMotorInverted() {
        return driveMotorInverted;
    }

    @Override
    public boolean angleMotorInverted() {
        return angleMotorInverted;
    }

    @Override
    public boolean driveMotorSensorPhase() {
        return driveMotorSensorPhase;
    }

    @Override
    public boolean angleMotorSensorPhase() {
        return angleMotorSensorPhase;
    }

    @Override
    public double angle_kp() {
        return angle_kp.get();
    }

    @Override
    public double angle_ki() {
        return angle_ki.get();
    }

    @Override
    public double angle_kd() {
        return angle_kd.get();
    }

    @Override
    public double angle_kf() {
        return angle_kf.get();
    }

    @Override
    public double j() {
        return j.get();
    }

    @Override
    public double zeroPosition() {
        return zeroPosition;
    }

    @Override
    public int motionAcceleration() {
        return motionAcceleration;
    }

    @Override
    public int motionCruiseVelocity() {
        return motionCruiseVelocity;
    }

    @Override
    public int curveStrength() {
        return curveStrength;
    }

    @Override
    public double ticksPerMeter() {
        return ticksPerMeter;
    }

    @Override
    public double wheelRadius() {
        return wheelRadius;
    }

    @Override
    public double ticksPerRadian() {
        return ticksPerRadian;
    }

    @Override
    public int angleMaxCurrent() {
        return angleMaxCurrent;
    }

    @Override
    public double velocityTolerance() {
        return velocityTolerance;
    }

    @Override
    public double modelTolerance() {
        return modelTolerance;
    }

    @Override
    public double encoderTolerance() {
        return encoderTolerance;
    }

    @Override
    public double driveMotorGearRatio() {
        return driveMotorGearRatio;
    }

    @Override
    public boolean debug() {
        return true;
    }
}

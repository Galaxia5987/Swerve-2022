package frc.robot.utils;

import frc.robot.valuetuner.WebConstant;

public class SwerveModuleConfig implements SwerveModuleConfigBase {
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
    private final double angle_kp;
    private final double angle_ki;
    private final double angle_kd;
    private final double angle_kf;

    private final double j; // moment of inertia [kg * m^2]

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

    public SwerveModuleConfig(int wheel, int driveMotorPort, int angleMotorPort,
                              boolean driveMotorInverted, boolean angleMotorInverted,
                              boolean driveMotorSensorPhase, boolean angleMotorSensorPhase,
                              double angle_kp, double angle_ki, double angle_kd, double angle_kf,
                              double j, double zeroPosition, int motionAcceleration, int motionCruiseVelocity, int curveStrength, CommonSwerveConfig commonConfig) {
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
        return angle_kp;
    }

    @Override
    public double angle_ki() {
        return angle_ki;
    }

    @Override
    public double angle_kd() {
        return angle_kd;
    }

    @Override
    public double angle_kf() {
        return angle_kf;
    }

    @Override
    public double j() {
        return j;
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
        return false;
    }

    public static final class Builder {
        private final int wheel;
        private CommonSwerveConfig commonConfig;
        private int zeroPosition;
        // ports
        private int driveMotorPort;
        private int angleMotorPort;
        // inversions
        private boolean driveMotorInverted;
        private boolean angleMotorInverted;
        private boolean driveMotorSensorPhase;
        private boolean angleMotorSensorPhase;
        // PID
        private double angle_kp;
        private double angle_ki;
        private double angle_kd;
        private double angle_kf;
        private double j; // moment of inertia
        // Motion Magic
        private int motionAcceleration;
        private int motionCruseVelocity;
        private int curveStrength;

        private boolean debug;

        public Builder(int wheel) {
            this.wheel = wheel;
        }

        public Builder configZeroPosition(int zeroPosition) {
            this.zeroPosition = zeroPosition;
            return this;
        }

        public Builder configCommonConfig(CommonSwerveConfig commonConfig) {
            this.commonConfig = commonConfig;
            return this;
        }

        public Builder configPorts(int drive, int angle) {
            this.driveMotorPort = drive;
            this.angleMotorPort = angle;
            return this;
        }

        public Builder configInversions(boolean driveMotorInverted, boolean angleMotorInverted,
                                        boolean driveMotorSensorPhase, boolean angleMotorSensorPhase) {
            this.driveMotorInverted = driveMotorInverted;
            this.angleMotorInverted = angleMotorInverted;
            this.driveMotorSensorPhase = driveMotorSensorPhase;
            this.angleMotorSensorPhase = angleMotorSensorPhase;

            return this;
        }

        public Builder configAnglePID(double kp, double ki, double kd, double kf) {
            this.angle_kp = kp;
            this.angle_ki = ki;
            this.angle_kd = kd;
            this.angle_kf = kf;
            return this;
        }

        public Builder configJ(double j) {
            this.j = j;
            return this;
        }

        public Builder configMotionMagic(int motionAcceleration, int motionCruseVelocity, int curveStrength) {
            this.motionAcceleration = motionAcceleration;
            this.motionCruseVelocity = motionCruseVelocity;
            this.curveStrength = curveStrength;
            return this;
        }

        public Builder enableDebug() {
            this.debug = true;
            return this;
        }

        public SwerveModuleConfigBase build() {
            if (debug) {
                return new SwerveModuleConfigDebug(wheel, driveMotorPort, angleMotorPort,
                        driveMotorInverted, angleMotorInverted, driveMotorSensorPhase, angleMotorSensorPhase,
                        new WebConstant("Swerve_" + wheel + "_kp", angle_kp), new WebConstant("Swerve_" + wheel + "_ki", angle_ki),
                        new WebConstant("Swerve_" + wheel + "_kd", angle_kd), new WebConstant("Swerve_" + wheel + "_kf", angle_kf),
                        new WebConstant("Swerve_" + wheel + "_j", j), zeroPosition, motionAcceleration, motionCruseVelocity, curveStrength, commonConfig);
            }
            return new SwerveModuleConfig(wheel, driveMotorPort, angleMotorPort,
                    driveMotorInverted, angleMotorInverted, driveMotorSensorPhase, angleMotorSensorPhase,
                    angle_kp, angle_ki, angle_kd, angle_kf,
                    j, zeroPosition, motionAcceleration, motionCruseVelocity, curveStrength, commonConfig);
        }
    }
}

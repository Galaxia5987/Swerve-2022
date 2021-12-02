package frc.robot.utils;

public class SwerveModuleConfig {
    // ports
    public final int driveMotorPort;
    public final int angleMotorPort;

    // inversions
    public final boolean driveMotorInverted;
    public final boolean angleMotorInverted;
    public final boolean driveMotorSensorPhase;
    public final boolean angleMotorSensorPhase;

    // PID
    public final double angle_kp;
    public final double angle_ki;
    public final double angle_kd;
    public final double angle_kf;

    public final double j; // moment of inertia [kg * m^2]

    public final double zeroPosition; // [ticks]

    // drive motor
    public final double ticksPerMeter;
    public final double wheelRadius; // [m]

    // angle motor
    public final double ticksPerRadian;
    public final int allowableAngleError; // [ticks]

    // constrains
    public final int angleMaxCurrent; // [amps]
    public final double velocityTolerance; // [RPS]
    public final double modelTolerance;
    public final double encoderTolerance; // [ticks]

    public SwerveModuleConfig(int driveMotorPort, int angleMotorPort,
                              boolean driveMotorInverted, boolean angleMotorInverted,
                              boolean driveMotorSensorPhase, boolean angleMotorSensorPhase,
                              double angle_kp, double angle_ki, double angle_kd, double angle_kf,
                              double j, double zeroPosition, CommonSwerveConfig commonConfig) {
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
        this.ticksPerMeter = commonConfig.ticksPerMeter;
        this.wheelRadius = commonConfig.wheelRadius;
        this.ticksPerRadian = commonConfig.ticksPerRadian;
        this.allowableAngleError = commonConfig.allowableAngleError;
        this.angleMaxCurrent = commonConfig.angleMaxCurrent;
        this.velocityTolerance = commonConfig.velocityTolerance;
        this.modelTolerance = commonConfig.modelTolerance;
        this.encoderTolerance = commonConfig.encoderTolerance;
    }

    public static final class Builder {
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

        public Builder() {
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

        public SwerveModuleConfig build() {
            return new SwerveModuleConfig(driveMotorPort, angleMotorPort,
                    driveMotorInverted, angleMotorInverted, driveMotorSensorPhase, angleMotorSensorPhase,
                    angle_kp, angle_ki, angle_kd, angle_kf,
                    j, zeroPosition, commonConfig);
        }
    }
}

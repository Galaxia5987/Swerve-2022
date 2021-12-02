package frc.robot.utils;

public class CommonSwerveConfig {
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
    public final double driveMotorGearRatio;

    public CommonSwerveConfig(double ticksPerMeter, double wheelRadius,
                              double ticksPerRadian, double velocityTolerance,
                              int allowableAngleError, int angleMaxCurrent,
                              double modelTolerance, double encoderTolerance,
                              double driveMotorGearRatio) {
        this.ticksPerMeter = ticksPerMeter;
        this.wheelRadius = wheelRadius;
        this.ticksPerRadian = ticksPerRadian;
        this.allowableAngleError = allowableAngleError;
        this.angleMaxCurrent = angleMaxCurrent;
        this.velocityTolerance = velocityTolerance;
        this.modelTolerance = modelTolerance;
        this.encoderTolerance = encoderTolerance;
        this.driveMotorGearRatio = driveMotorGearRatio;
    }

    public static final class Builder {
        // drive motor
        private double ticksPerMeter;
        private double wheelRadius; // [m]

        // angle motor
        private double ticksPerRadian;
        private int allowableAngleError; // [ticks]

        // constrains
        private int angleMaxCurrent; // [amps]
        private double velocityTolerance; // [RPS]
        private double modelTolerance;
        private double encoderTolerance; // [ticks]
        public double driveMotorGearRatio;


        public Builder() {
        }

        public Builder configTicksPerUnit(double driveMotorTicksPerMeter, double angleMotorTicksPerRad) {
            this.ticksPerMeter = driveMotorTicksPerMeter;
            this.ticksPerRadian = angleMotorTicksPerRad;
            return this;
        }

        public Builder configWheelRadius(double wheelRadius) {
            this.wheelRadius = wheelRadius;
            return this;
        }

        public Builder configAllowableAngleError(int allowableAngleError) {
            this.allowableAngleError = allowableAngleError;
            return this;
        }

        public Builder configDriveMotorGearRatio(double driveMotorGearRatio) {
            this.driveMotorGearRatio = driveMotorGearRatio;
            return this;
        }

        public Builder configConstrains(int angleMaxCurrent, double velocityTolerance, double modelTolerance, double encoderTolerance) {
            this.angleMaxCurrent = angleMaxCurrent;
            this.velocityTolerance = velocityTolerance;
            this.modelTolerance = modelTolerance;
            this.encoderTolerance = encoderTolerance;
            return this;
        }

        public CommonSwerveConfig build() {
            return new CommonSwerveConfig(ticksPerMeter, wheelRadius,
                    ticksPerRadian, velocityTolerance,
                    allowableAngleError, angleMaxCurrent,
                    modelTolerance, encoderTolerance, driveMotorGearRatio);
        }
    }
}

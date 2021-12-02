package frc.robot.utils;

public class CommonSwerveConfig {
    // drive motor
    public final double ticksPerMeter;
    public final double wheelRadius; // [m]

    // angle motor
    public final double ticksPerRadian;
    public final double angleGearRatio;
    public final int allowableAngleError; // [ticks]

    // constrains
    public final int angleMaxCurrent; // [amps]
    public final double velocityTolerance; // [RPS]
    public final double modelTolerance;
    public final double encoderTolerance; // [ticks]

    public CommonSwerveConfig(double ticksPerMeter, double wheelRadius,
                              double ticksPerRadian, double angleGearRatio,
                              int allowableAngleError, int angleMaxCurrent,
                              double velocityTolerance, double modelTolerance,
                              double encoderTolerance) {
        this.ticksPerMeter = ticksPerMeter;
        this.wheelRadius = wheelRadius;
        this.angleGearRatio = angleGearRatio;
        this.ticksPerRadian = ticksPerRadian;
        this.allowableAngleError = allowableAngleError;
        this.angleMaxCurrent = angleMaxCurrent;
        this.velocityTolerance = velocityTolerance;
        this.modelTolerance = modelTolerance;
        this.encoderTolerance = encoderTolerance;
    }

    public static final class Builder {
        // drive motor
        private double ticksPerMeter;
        private double wheelRadius; // [m]

        // angle motor
        private double ticksPerRadian;
        private double angleGearRatio;
        private int allowableAngleError; // [ticks]

        // constrains
        private int angleMaxCurrent; // [amps]
        private double velocityTolerance; // [RPS]
        private double modelTolerance;
        private double encoderTolerance; // [ticks]

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

        public Builder configAngleGearRatio(double angleGearRatio) {
            this.angleGearRatio = angleGearRatio;
            return this;
        }

        public Builder configAllowableAngleError(int allowableAngleError) {
            this.allowableAngleError = allowableAngleError;
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
                    ticksPerRadian, angleGearRatio,
                    allowableAngleError, angleMaxCurrent,
                    velocityTolerance, modelTolerance,
                    encoderTolerance);
        }
    }
}

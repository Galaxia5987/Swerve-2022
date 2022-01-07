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

    public SwerveModuleConfig(int wheel, int driveMotorPort, int angleMotorPort,
                              boolean driveMotorInverted, boolean angleMotorInverted,
                              boolean driveMotorSensorPhase, boolean angleMotorSensorPhase,
                              double angle_kp, double angle_ki, double angle_kd, double angle_kf,
                              double j, double zeroPosition) {
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
    public boolean debug() {
        return false;
    }

    public static final class Builder {
        private final int wheel;
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

        private boolean debug;

        public Builder(int wheel) {
            this.wheel = wheel;
        }

        public Builder configZeroPosition(int zeroPosition) {
            this.zeroPosition = zeroPosition;
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
                        new WebConstant("Swerve_" + wheel + "_j", j), zeroPosition);
            }
            return new SwerveModuleConfig(wheel, driveMotorPort, angleMotorPort,
                    driveMotorInverted, angleMotorInverted, driveMotorSensorPhase, angleMotorSensorPhase,
                    angle_kp, angle_ki, angle_kd, angle_kf, j, zeroPosition);
        }
    }
}

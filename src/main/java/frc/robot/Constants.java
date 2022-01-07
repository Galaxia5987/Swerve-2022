package frc.robot;


import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.utils.SwerveModuleConfigBase;

import static frc.robot.Ports.SwerveDrive.*;

public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].
    public static final double NOMINAL_VOLTAGE = 12; // [volts]
    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public enum Motor {
        TalonFX(12, 4.69, 257, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6380.0)),
        TalonSRX(12, 0.7, 130, 3.8, Units.rotationsPerMinuteToRadiansPerSecond(21020.0)),
        NEO(12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5657.0)),
        NEO500(12, 0.97, 100, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(1100));

        public final double nominalVoltage; // [volts]
        public final double stallTorque; // [N * m]
        public final double stallCurrent; // [amps]
        public final double freeCurrent; // [amps]
        public final double freeSpeed; // [rad/sec]
        public final double omega; // [ohms]
        public final double Kv; // [rad/(sec*Volt)]
        public final double Kt; // [n * m/ amps]

        Motor(double nominalVoltageVolts, double stallTorqueNewtonMeters, double stallCurrentAmps, double freeCurrentAmps, double freeSpeedRadPerSec) {
            this.nominalVoltage = nominalVoltageVolts;
            this.stallTorque = stallTorqueNewtonMeters;
            this.stallCurrent = stallCurrentAmps;
            this.freeCurrent = freeCurrentAmps;
            this.freeSpeed = freeSpeedRadPerSec;
            this.omega = nominalVoltageVolts / stallCurrentAmps;
            this.Kv = freeSpeedRadPerSec / (nominalVoltageVolts - omega * freeCurrentAmps);
            this.Kt = stallTorqueNewtonMeters / stallCurrentAmps;
        }
    }

    public static final class SwerveDrive {
        public static final int TICKS_PER_ROTATION_DRIVE_MOTOR = 2048;
        public static final int TICKS_PER_ROTATION_ANGLE_MOTOR = 1024;
        public static final double GEAR_RATIO_DRIVE_MOTOR = 7.5;
        public static final double GEAR_RATIO_ANGLE_MOTOR = 1;
        public static final double DRIVE_MOTOR_TICKS_PER_METER = GEAR_RATIO_DRIVE_MOTOR * TICKS_PER_ROTATION_DRIVE_MOTOR / (4 * 0.0254 * Math.PI);
        public static final double ANGLE_MOTOR_TICKS_PER_RADIAN = GEAR_RATIO_ANGLE_MOTOR * TICKS_PER_ROTATION_ANGLE_MOTOR / (2 * Math.PI);

        public static final int MAX_CURRENT = 15; // [amps]

        // State Space
        public static final double VELOCITY_TOLERANCE = 5 / (2 * Math.PI * Constants.SwerveDrive.WHEEL_RADIUS); // [m/s]
        public static final double COST_LQR = 11;
        public static final double MODEL_TOLERANCE = 0.01;
        public static final double ENCODER_TOLERANCE = 0.01; // [ticks]

        public static final double ALLOWABLE_ANGLE_ERROR = Math.toRadians(8); // [rad]
        public static final double WHEEL_RADIUS = 0.04688; // [m]

        public static final double KP_TURN = 0.05;
        public static final double KI_TURN = 0.02;
        public static final double KD_TURN = 0;

        public static final double ROBOT_LENGTH = 0.5924; // [m]
        public static final double ROBOT_WIDTH = 0.5924; // [m]
        public static final double JOYSTICK_THRESHOLD = 0.1;
        public static final double VELOCITY_MULTIPLIER = 4 / Math.sqrt(2);

        // the rotational speed of the robot, this constant multiplies the rotation output of the joystick
        public static final double ROTATION_MULTIPLIER = Math.PI;
        public static final double OUTER_JOYSTICK_THRESHOLD = 0.95;
        public static final double JOYSTICK_ANGLE_DEADZONE = 5; // [degrees]
        public static final int ANGLE_CURVE_STRENGTH = 4;
        // angle motion magic
        private static final float magicSafety = 0.7f;
        public static final int ANGLE_MOTION_ACCELERATION = Math.round(2800 * magicSafety);
        public static final int ANGLE_CRUISE_VELOCITY = Math.round((550 * magicSafety));
    }

    public static final class SwerveModule {

        public static final int[] ZERO_POSITIONS = {880, 465, 1328, 913}; // fr, fl, rr, rl
        public static final int TRIGGER_THRESHOLD_CURRENT = 2; // [amps]
        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // [secs]
        public static final double RAMP_RATE = 1; // seconds from neutral to max
        public static final SwerveModuleConfigBase frConfig = new SwerveModuleConfig.Builder(0).configPorts(DRIVE_MOTOR_FR, ANGLE_MOTOR_FR)
                .configInversions(DRIVE_INVERTED_FR, ANGLE_INVERTED_FR, DRIVE_SENSOR_PHASE_FR, ANGLE_SENSOR_PHASE_FR)
                .configAnglePID(4.5, 0.0045, 1, 0)
                .configZeroPosition(ZERO_POSITIONS[0])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase flConfig = new SwerveModuleConfig.Builder(1).configPorts(DRIVE_MOTOR_FL, ANGLE_MOTOR_FL)
                .configInversions(DRIVE_INVERTED_FL, ANGLE_INVERTED_FL, DRIVE_SENSOR_PHASE_FL, ANGLE_SENSOR_PHASE_FL)
                .configAnglePID(13, 0.0045, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[1])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rrConfig = new SwerveModuleConfig.Builder(2).configPorts(DRIVE_MOTOR_RR, ANGLE_MOTOR_RR)
                .configInversions(DRIVE_INVERTED_RR, ANGLE_INVERTED_RR, DRIVE_SENSOR_PHASE_RR, ANGLE_SENSOR_PHASE_RR)
                .configAnglePID(8, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[2])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rlConfig = new SwerveModuleConfig.Builder(3).configPorts(DRIVE_MOTOR_RL, ANGLE_MOTOR_RL)
                .configInversions(DRIVE_INVERTED_RL, ANGLE_INVERTED_RL, DRIVE_SENSOR_PHASE_RL, ANGLE_SENSOR_PHASE_RL)
                .configAnglePID(10, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[3])
                .configJ(0.115)
                .build();

        // TODO: Calibrate
        public static final double DRIVE_NEUTRAL_DEADBAND = 0.1;
        public static final double ANGLE_NEUTRAL_DEADBAND = 0.1;
    }

    public static class Autonomous {
        public static final double kPThetaController = 12;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints();
        public static final double kPXController = 8;
        public static final double kPYController = 10;
    }

}
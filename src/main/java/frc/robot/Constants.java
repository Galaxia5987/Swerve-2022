package frc.robot;


import frc.robot.utils.CommonSwerveConfig;
import frc.robot.utils.SwerveModuleConfig;

import static frc.robot.Ports.SwerveDrive.*;

public final class Constants {
    public static final class SwerveDrive {
        public static final int TICKS_PER_ROTATION_DRIVE_MOTOR = 2048;
        public static final int TICKS_PER_ROTATION_ANGLE_MOTOR = 1024;
        public static final double GEAR_RATIO_DRIVE_MOTOR = 7.5;
        public static final double GEAR_RATIO_ANGLE_MOTOR = 1;

        public static final int MAX_CURRENT = 15; // [amps]

        // State Space
        public static final double VELOCITY_TOLERANCE = 20; // [rps]
        public static final double MODEL_TOLERANCE = 4;
        public static final double ENCODER_TOLERANCE = 4; // [ticks]

        public static final int ALLOWABLE_ANGLE_ERROR = 3; // [ticks]
        public static final double WHEEL_RADIUS = 0.04688; // [m]


        public static final CommonSwerveConfig commonConfig = new CommonSwerveConfig.Builder()
                .configTicksPerUnit(GEAR_RATIO_DRIVE_MOTOR * TICKS_PER_ROTATION_DRIVE_MOTOR / (4 * 0.0254 * Math.PI),
                        GEAR_RATIO_ANGLE_MOTOR * TICKS_PER_ROTATION_ANGLE_MOTOR / (2 * Math.PI))
                .configConstrains(MAX_CURRENT, VELOCITY_TOLERANCE, MODEL_TOLERANCE, ENCODER_TOLERANCE)
                .configAllowableAngleError(ALLOWABLE_ANGLE_ERROR)
                .configWheelRadius(WHEEL_RADIUS)
                .build();

    }

    public static final class SwerveModule {
        public static final int[] ZERO_POSITIONS = {923, 492, 986, 318}; // fr, fl, rr, rl

        public static final SwerveModuleConfig frConfig = new SwerveModuleConfig.Builder()
                .configCommonConfig(SwerveDrive.commonConfig)
                .configPorts(DRIVE_MOTOR_FR, ANGLE_MOTOR_FR)
                .configInversions(DRIVE_INVERTED_FR, ANGLE_INVERTED_FR, DRIVE_SENSOR_PHASE_FR, ANGLE_SENSOR_PHASE_FR)
                .configAnglePID(4.5, 0.0045, 1, 0)
                .configZeroPosition(ZERO_POSITIONS[0])
                .configJ(0.0043)
                .build();
        public static final SwerveModuleConfig flConfig = new SwerveModuleConfig.Builder()
                .configCommonConfig(SwerveDrive.commonConfig)
                .configPorts(DRIVE_MOTOR_FL, ANGLE_MOTOR_FL)
                .configInversions(DRIVE_INVERTED_FL, ANGLE_INVERTED_FL, DRIVE_SENSOR_PHASE_FL, ANGLE_SENSOR_PHASE_FL)
                .configAnglePID(4, 0.0045, 3, 0)
                .configZeroPosition(ZERO_POSITIONS[1])
                .configJ(0.0043)
                .build();
        public static final SwerveModuleConfig rrConfig = new SwerveModuleConfig.Builder()
                .configCommonConfig(SwerveDrive.commonConfig)
                .configPorts(DRIVE_MOTOR_RR, ANGLE_MOTOR_RR)
                .configInversions(DRIVE_INVERTED_RR, ANGLE_INVERTED_RR, DRIVE_SENSOR_PHASE_RR, ANGLE_SENSOR_PHASE_RR)
                .configAnglePID(4.35, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[2])
                .configJ(0.0043)
                .build();
        public static final SwerveModuleConfig rlConfig = new SwerveModuleConfig.Builder()
                .configCommonConfig(SwerveDrive.commonConfig)
                .configPorts(DRIVE_MOTOR_RL, ANGLE_MOTOR_RL)
                .configInversions(DRIVE_INVERTED_RL, ANGLE_INVERTED_RL, DRIVE_SENSOR_PHASE_RL, ANGLE_SENSOR_PHASE_RL)
                .configAnglePID(4.5, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[3])
                .configJ(0.0043)
                .build();
    }
}

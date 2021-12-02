package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModuleConfig;

public class SwerveModule extends SubsystemBase {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX angleMotor;

    public SwerveModule(SwerveModuleConfig config) {
        driveMotor = new WPI_TalonFX(config.driveMotorPort);
        angleMotor = new WPI_TalonSRX(config.angleMotorPort);

        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);

        angleMotor.setNeutralMode(NeutralMode.Brake);

        // inversions
        driveMotor.setInverted(config.driveMotorInverted);
        angleMotor.setInverted(config.angleMotorInverted);

        driveMotor.setSensorPhase(config.driveMotorSensorPhase);
        angleMotor.setSensorPhase(config.angleMotorSensorPhase);

        // Set amperage limits
        SupplyCurrentLimitConfiguration currLimitConfig = new SupplyCurrentLimitConfiguration(
                Constants.ENABLE_CURRENT_LIMIT,
                Constants.SwerveDrive.MAX_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_TIME
        );

        driveMotor.configSupplyCurrentLimit(currLimitConfig);

        angleMotor.configSupplyCurrentLimit(currLimitConfig);
        angleMotor.enableCurrentLimit(Constants.ENABLE_CURRENT_LIMIT);

        // set PIDF - angle motor
        angleMotor.config_kP(0, config.angle_kp, Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, config.angle_ki, Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, config.angle_kd, Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, config.angle_kf, Constants.TALON_TIMEOUT);
        angleMotor.config_IntegralZone(0, 5);

        // set voltage compensation and saturation
        driveMotor.enableVoltageCompensation(Constants.ENABLE_VOLTAGE_COMPENSATION);
        driveMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        angleMotor.enableVoltageCompensation(Constants.ENABLE_VOLTAGE_COMPENSATION);
        angleMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        angleMotor.selectProfileSlot(0, 0);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.setSelectedSensorPosition(0);

    }
}

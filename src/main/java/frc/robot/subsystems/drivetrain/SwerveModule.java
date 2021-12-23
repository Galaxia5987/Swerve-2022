package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.StateSpaceUtils;
import frc.robot.utils.SwerveModuleConfigBase;
import webapp.FireLog;

public class SwerveModule extends SubsystemBase {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX angleMotor;
    private final UnitModel driveUnitModel;
    private final UnitModel angleUnitModel;

    private final SwerveModuleConfigBase config;
    private final Timer timer = new Timer();
    private final LinearSystemLoop<N1, N1, N1> stateSpace;
    private double currentTime, lastTime;
    private double lastJ;

    public SwerveModule(SwerveModuleConfigBase config) {
        this.config = config;
        driveMotor = new WPI_TalonFX(config.driveMotorPort());
        angleMotor = new WPI_TalonSRX(config.angleMotorPort());
        driveUnitModel = new UnitModel(config.ticksPerMeter());
        angleUnitModel = new UnitModel(config.ticksPerRadian());
        stateSpace = constructLinearSystem(config.j());
        lastJ = config.j();


        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(false, Constants.TALON_TIMEOUT);

        angleMotor.setNeutralMode(NeutralMode.Brake);

        // inversions
        driveMotor.setInverted(config.driveMotorInverted());
        angleMotor.setInverted(config.angleMotorInverted());

        driveMotor.setSensorPhase(config.driveMotorSensorPhase());
        angleMotor.setSensorPhase(config.angleMotorSensorPhase());

        // Set amperage limits
        SupplyCurrentLimitConfiguration currLimitConfig = new SupplyCurrentLimitConfiguration(Constants.ENABLE_CURRENT_LIMIT, Constants.SwerveDrive.MAX_CURRENT, Constants.SwerveModule.TRIGGER_THRESHOLD_CURRENT, Constants.SwerveModule.TRIGGER_THRESHOLD_TIME);

        driveMotor.configSupplyCurrentLimit(currLimitConfig);

        angleMotor.configSupplyCurrentLimit(currLimitConfig);
        angleMotor.enableCurrentLimit(Constants.ENABLE_CURRENT_LIMIT);

        // set PIDF - angle motor
        angleMotor.config_kP(0, config.angle_kp(), Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, config.angle_ki(), Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, config.angle_kd(), Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, config.angle_kf(), Constants.TALON_TIMEOUT);
        angleMotor.config_IntegralZone(0, 5);

        // set voltage compensation and saturation
        driveMotor.enableVoltageCompensation(Constants.ENABLE_VOLTAGE_COMPENSATION);
        driveMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        angleMotor.enableVoltageCompensation(Constants.ENABLE_VOLTAGE_COMPENSATION);
        angleMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        angleMotor.selectProfileSlot(0, 0);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.setSelectedSensorPosition(0);

        angleMotor.configMotionAcceleration(config.motionAcceleration());
        angleMotor.configMotionCruiseVelocity(config.motionCruiseVelocity());
        angleMotor.configMotionSCurveStrength(config.curveStrength());

        driveMotor.configOpenloopRamp(Constants.SwerveModule.RAMP_RATE, Constants.TALON_TIMEOUT);
    }

    /**
     * Initialize the linear system to the default values in order to use the state-space.
     *
     * @return an object that represents the model to reach the velocity at the best rate.
     */
    private LinearSystemLoop<N1, N1, N1> constructLinearSystem(double J) {
        if (J == 0) throw new RuntimeException("J must have non-zero value");
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf Page 76
        LinearSystem<N1, N1, N1> stateSpace = StateSpaceUtils.createVelocityLinearSystem(Constants.Motor.TalonFX, config.driveMotorGearRatio(), J);
        KalmanFilter<N1, N1, N1> kalman = new KalmanFilter<>(Nat.N1(), Nat.N1(), stateSpace,
                VecBuilder.fill(config.modelTolerance()),
                VecBuilder.fill(config.encoderTolerance()),
                Constants.LOOP_PERIOD
        );
        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<>(stateSpace,
                VecBuilder.fill(config.velocityTolerance()),
                VecBuilder.fill(Constants.NOMINAL_VOLTAGE),
                Constants.LOOP_PERIOD // time between loops, DON'T CHANGE
        );
        lqr.latencyCompensate(stateSpace, Constants.LOOP_PERIOD, Constants.TALON_TIMEOUT * 0.001);

        return new LinearSystemLoop<>(stateSpace, lqr, kalman, Constants.NOMINAL_VOLTAGE, Constants.LOOP_PERIOD);
    }

    /**
     * @return the speed of the wheel. [m/s]
     */
    public double getVelocity() {
        return driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity(0));
    }

    /**
     * Sets the speed of the wheel.
     *
     * @param speed the speed of the wheel.[m/s]
     */
    public void setVelocity(double speed) {
        double timeInterval = Math.max(Constants.LOOP_PERIOD, currentTime - lastTime);
        double currentSpeed = getVelocity() / (2 * Math.PI * config.wheelRadius()); // [rps]
        double targetSpeed = speed / (2 * Math.PI * config.wheelRadius()); // [rps]

        stateSpace.setNextR(VecBuilder.fill(targetSpeed)); // r = reference (setpoint)
        stateSpace.correct(VecBuilder.fill(currentSpeed));
        stateSpace.predict(timeInterval);

        double voltageToApply = stateSpace.getU(0); // u = input, calculated by the input.
        // returns the voltage to apply (between -12 and 12)
        driveMotor.set(ControlMode.PercentOutput, voltageToApply / Constants.NOMINAL_VOLTAGE);
    }

    /**
     * @return the angle of the wheel. [rad]
     */
    public Rotation2d getAngle() {
        return new Rotation2d(Math.IEEEremainder(angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition() - config.zeroPosition()), 2 * Math.PI));
    }

    /**
     * Sets the angle of the wheel, in consideration of the shortest path to the target angle.
     *
     * @param angle the target angle.
     */
    public void setAngle(Rotation2d angle) {
        var currentAngle = getAngle();
        var error = angle.minus(currentAngle);
        if (Math.abs(error.getRadians()) < Constants.SwerveDrive.ALLOWABLE_ANGLE_ERROR) return;

        angleMotor.set(ControlMode.MotionMagic, angleMotor.getSelectedSensorPosition() + angleUnitModel.toTicks(error.getRadians()));
    }

    /**
     * @return the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Update the state of the module.
     *
     * @param state the desired state.
     */
    public void setState(SwerveModuleState state) {
        setVelocity(state.speedMetersPerSecond);
        setAngle(state.angle);
    }

    /**
     * Stops the angle motor.
     */
    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    /**
     * Stops the drive motor.
     */
    public void stopDriveMotor() {
        driveMotor.stopMotor();
    }

    /**
     * Runs the motor at full power.
     */
    public void setMaxOutput() {
        angleMotor.set(ControlMode.PercentOutput, 1);
        driveMotor.set(ControlMode.PercentOutput, 1);
    }

    /**
     * Resets the angle motor encoder position back to 0.
     */
    public void resetAngleMotor() {
        angleMotor.setSelectedSensorPosition(0);
    }

    /**
     * Gets the wheel number.
     *
     * @return the wheel number.
     */
    public int getWheel() {
        return config.wheel();
    }

    public void configPID(double kp, double ki, double kd, double kf) {
        angleMotor.config_kP(0, kp, Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, ki, Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, kd, Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, kf, Constants.TALON_TIMEOUT);
    }

    @Override
    public void periodic() {
        if (config.debug()) {
            configPID(config.angle_kp(), config.angle_ki(), config.angle_kd(), config.angle_kf());
            if (config.j() != lastJ) {
                constructLinearSystem(config.j());
                lastJ = config.j();
            }
        }
        FireLog.log("angle " + config.wheel(), getAngle().getDegrees());
        FireLog.log("velocity " + config.wheel(), getVelocity());
        lastTime = currentTime;
        currentTime = timer.get();
    }
}

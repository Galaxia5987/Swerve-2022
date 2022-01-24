package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.LOOP_PERIOD;

public class VoltageTest extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final double maxCycles;
    private double cycles = 0;
    private double initVoltage;

    public VoltageTest(SwerveDrive swerveDrive, int cycles) {
        this.swerveDrive = swerveDrive;
        this.maxCycles = cycles;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        initVoltage = RobotController.getBatteryVoltage();
    }

    @Override
    public void execute() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setMaxOutput();
        }
        cycles += LOOP_PERIOD;
    }

    @Override
    public boolean isFinished() {
        return cycles >= maxCycles;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
        System.out.println(initVoltage - RobotController.getBatteryVoltage());
    }
}

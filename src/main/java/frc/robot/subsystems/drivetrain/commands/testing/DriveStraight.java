package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveStraight extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final boolean mode;
    private final Timer timer = new Timer();
    private final double run_time = 2;
    private final double current_run;
    private double init_yaw;

    public DriveStraight(SwerveDrive swerveDrive, boolean mode, double current_run) {
        this.swerveDrive = swerveDrive;
        this.mode = mode;
        this.current_run = current_run;
    }

    @Override
    public void initialize() {
        init_yaw = Robot.navx.getYaw();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double output = mode ? 1 : -1;
        swerveDrive.holonomicDrive(output, 0, 0);

        double yaw = Robot.navx.getYaw();
        System.out.println("Delta yaw = " + (Math.abs(yaw) - Math.abs(init_yaw)));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
        System.out.println("Run #" + current_run + " completed.");
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(run_time);
    }
}

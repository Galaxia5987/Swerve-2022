package frc.robot.subsystems.drivetrain.commands.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

public class Rotate extends CommandBase {

    private final SwerveDrive swerveDrive;
    //    private WebConstant target = new WebConstant("targetAngle", 0);

    // the target angles for each wheel by numbered index
    private WebConstant target0 = new WebConstant("target0", 0);
    private WebConstant target1 = new WebConstant("target1", 0);
    private WebConstant target2 = new WebConstant("target2", 0);
    private WebConstant target3 = new WebConstant("target3", 0);


    public Rotate(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double rotation = -RobotContainer.Xbox.getY();
        rotation = Utils.joystickDeadband(rotation, Constants.SwerveDrive.JOYSTICK_THRESHOLD);

        swerveDrive.getModule(0).setAngle(Math.toRadians(target0.get()));
        swerveDrive.getModule(1).setAngle(Math.toRadians(target0.get()));
        swerveDrive.getModule(2).setAngle(Math.toRadians(target0.get()));
        swerveDrive.getModule(3).setAngle(Math.toRadians(target0.get()));

        SmartDashboard.putNumber("module FR", Math.toDegrees(swerveDrive.getModule(0).getAngle()));
        SmartDashboard.putNumber("module FL", Math.toDegrees(swerveDrive.getModule(1).getAngle()));
        SmartDashboard.putNumber("module RR", Math.toDegrees(swerveDrive.getModule(2).getAngle()));
        SmartDashboard.putNumber("module RL", Math.toDegrees(swerveDrive.getModule(3).getAngle()));
        FireLog.log("angle-setpoint", target0.get());
        FireLog.log("module FR", Math.toDegrees(swerveDrive.getModule(0).getAngle()));
        FireLog.log("module FL", Math.toDegrees(swerveDrive.getModule(1).getAngle()));
        FireLog.log("module RR", Math.toDegrees(swerveDrive.getModule(2).getAngle()));
        FireLog.log("module RL", Math.toDegrees(swerveDrive.getModule(3).getAngle()));

        FireLog.log("angle ", swerveDrive.getModule(2).getAngle());
        FireLog.log("swerve direction", Robot.navx.getYaw());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }

}

package frc.robot.subsystems.drivetrain.commands.tuning;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.WebConstant;

public class DriveForward extends CommandBase {

    private final SwerveDrive swerveDrive;
    private WebConstant target = new WebConstant("targetSpeed", 0);

    public DriveForward(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
//        double forward = Utils.joystickDeadband(-RobotContainer.Xbox.getY(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);

//        swerveDrive.stayAtAngle();
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setState(new SwerveModuleState(target.get(), new Rotation2d(0)));
//            FireLog.log("speed " + i, Math.abs(swerveDrive.getModule(i).getSpeed()));
//            swerveDrive.getModule(i).configPIDF();
        }
//        swerveDrive.holonomicDrive(0, 0, forward);

/*
        FireLog.log("target speed", target.get());
        FireLog.log("swerve direction", Robot.navx.getYaw());
*/
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
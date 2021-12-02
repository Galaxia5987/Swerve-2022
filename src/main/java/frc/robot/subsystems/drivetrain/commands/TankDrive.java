package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

public class TankDrive extends CommandBase {
    private SwerveDrive swerveDrive;
    private final DoubleSupplier rightVelocity;
    private final DoubleSupplier leftVelocity;

    public TankDrive(SwerveDrive swerveDrive, DoubleSupplier rightVelocity, DoubleSupplier leftVelocity) {
        this.swerveDrive = swerveDrive;
        this.rightVelocity = rightVelocity;
        this.leftVelocity = leftVelocity;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setAngle(0);
        }
    }

    @Override
    public void execute() {
        double rightForward = rightVelocity.getAsDouble();
        double leftForward = leftVelocity.getAsDouble();

        swerveDrive.getModule(0).setVelocity(rightForward);
        swerveDrive.getModule(2).setVelocity(rightForward);

        swerveDrive.getModule(1).setVelocity(leftForward);
        swerveDrive.getModule(3).setVelocity(leftForward);

        swerveDrive.getModule(0).setAngle(0);
        swerveDrive.getModule(2).setAngle(0);
        swerveDrive.getModule(1).setAngle(0);
        swerveDrive.getModule(3).setAngle(0);
    }
}

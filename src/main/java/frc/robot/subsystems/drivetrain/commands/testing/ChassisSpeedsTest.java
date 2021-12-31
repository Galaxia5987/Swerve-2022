package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class ChassisSpeedsTest extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;

    public ChassisSpeedsTest(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(swerveDrive);
    }


    @Override
    public void execute() {
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        double alpha = Math.atan2(forward, strafe);
        double vector = Math.hypot(forward, strafe);


        vector *= Constants.SwerveDrive.VELOCITY_MULTIPLIER;

        forward = Math.sin(alpha) * vector;
        strafe = Math.cos(alpha) * vector;

        double rotation = Utils.deadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        if (vector < Constants.SwerveDrive.JOYSTICK_THRESHOLD) {
            forward = 0;
            strafe = 0;
        }

        rotation *= Constants.SwerveDrive.ROTATION_MULTIPLIER;
        if (forward != 0 || strafe != 0 || rotation != 0) {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
        } else {
            swerveDrive.terminate();
        }
        System.out.println(swerveDrive.getChassisSpeeds());
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

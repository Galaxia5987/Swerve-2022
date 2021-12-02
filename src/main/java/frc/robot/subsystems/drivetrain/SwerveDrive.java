package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] modules = new SwerveModule[4];
    private final boolean fieldOriented;

    public SwerveDrive(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
        modules[0] = new SwerveModule(Constants.SwerveModule.frConfig);
        modules[1] = new SwerveModule(Constants.SwerveModule.flConfig)  ;
        modules[2] = new SwerveModule(Constants.SwerveModule.rrConfig);
        modules[3] = new SwerveModule(Constants.SwerveModule.rlConfig);
    }
}

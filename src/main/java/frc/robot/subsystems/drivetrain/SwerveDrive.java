package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private static final SwerveDrive INSTANCE = new SwerveDrive(true);
    private final SwerveModule[] modules = new SwerveModule[4];
    private final boolean fieldOriented;

    private SwerveDrive(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
        modules[0] = new SwerveModule(Constants.SwerveModule.frConfig);
        modules[1] = new SwerveModule(Constants.SwerveModule.flConfig);
        modules[2] = new SwerveModule(Constants.SwerveModule.rrConfig);
        modules[3] = new SwerveModule(Constants.SwerveModule.rlConfig);
    }

    /**
     * @return the swerve.
     */
    public static SwerveDrive getInstance() {
        return INSTANCE;
    }

    /**
     * @param index the index of the module.
     * @return the corresponding module.
     */
    public SwerveModule getModule(int index) {
        return modules[index];
    }
}

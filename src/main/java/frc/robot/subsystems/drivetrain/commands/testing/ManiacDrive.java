package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class ManiacDrive extends SequentialCommandGroup {
    private final SwerveDrive swerveDrive;
    private boolean mode = true;

    public ManiacDrive(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        for (int i = 1; i <= 10; i++) {
            addCommands(
                    new DriveStraight(swerveDrive, mode, i)
            );
            mode = !mode;
        }
    }
}

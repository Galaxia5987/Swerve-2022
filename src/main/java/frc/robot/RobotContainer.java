package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private boolean hasSwerveEncoderReset = false;


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

    private void configureDefaultCommands() {
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // An ExampleCommand will run in autonomous
        return null;
    }

    public void teleopInit() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setEncoderRelative(!hasSwerveEncoderReset);
        }
        hasSwerveEncoderReset = true;

    }

    public void autoInit() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setEncoderRelative(!hasSwerveEncoderReset);
        }
        hasSwerveEncoderReset = true;
    }

    public void disableInit() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setEncoderAbsolute();
        }
    }
}

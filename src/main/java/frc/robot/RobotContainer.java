package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
  private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);

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
}

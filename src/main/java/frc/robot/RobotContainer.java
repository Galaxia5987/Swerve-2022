package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.drivetrain.commands.autonomous.FollowPath;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final SwerveDrive swerveDrive = new SwerveDrive(true);
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
        a.whenPressed(new InstantCommand(Robot.navx::reset));
    }

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(new HolonomicDrive(swerveDrive,
                () -> -xbox.getY(GenericHID.Hand.kLeft),
                () -> xbox.getX(GenericHID.Hand.kLeft),
                () -> xbox.getX(GenericHID.Hand.kRight)
        ));

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // An ExampleCommand will run in autonomous
        return new FollowPath(swerveDrive, "The Orbit", 0.5, 0.25,   new PIDController(Constants.Autonomous.kPXController, 0, 0),
                new PIDController(Constants.Autonomous.kPYController, 0, 0),
                new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0, Constants.Autonomous.kThetaControllerConstraints) {{
                    enableContinuousInput(-Math.PI, Math.PI);
                }});
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

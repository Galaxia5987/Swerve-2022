package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DampedDrive;
import frc.robot.subsystems.drivetrain.commands.autonomous.FollowPath;
import frc.robot.valuetuner.ValueTuner;
import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public static XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final SwerveDrive swerveDrive = SwerveDrive.getFieldRelativeInstance();
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();

        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        a.whenPressed((Runnable) Robot::resetAngle);
    }

    private void configureDefaultCommands() {
        /*        swerveDrive.setDefaultCommand(new FineTunedDrive(swerveDrive,
                () -> -xbox.getY(GenericHID.Hand.kLeft),
                () -> xbox.getX(GenericHID.Hand.kLeft),
                () -> xbox.getX(GenericHID.Hand.kRight)
        ));*/
        swerveDrive.setDefaultCommand(new DampedDrive(swerveDrive,
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
        return new FollowPath(swerveDrive, "New New New New New New New New Path", 1, 0.75, new PIDController(Constants.Autonomous.kPXController, 0, 0),
                new PIDController(Constants.Autonomous.kPYController, 0, 0),
                new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0, Constants.Autonomous.kThetaControllerConstraints) {{
                    enableContinuousInput(-Math.PI, Math.PI);
                }});
    }

    /**
     * Initiates the value tuner.
     */
    private void startValueTuner() {
        new ValueTuner().start();
    }

    /**
     * Initiates the port of team 225s Fire-Logger.
     */
    private void startFireLog() {

        try {
            new Webserver();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

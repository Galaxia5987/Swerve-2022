package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.valuetuner.ValueTuner;
import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public static XboxController xbox = new XboxController(Ports.Controls.XBOX);
    public static Joystick joystick = new Joystick(2);
    public static Joystick joystick2 = new Joystick(3);
    private final SwerveDrive swerveDrive = SwerveDrive.getFieldOrientedInstance();
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();

        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }
    }

    private void configureButtonBindings() {
        a.whenPressed(() -> {
            Robot.resetAngle();
            swerveDrive.resetThetaController();
        });
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
        return null;
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

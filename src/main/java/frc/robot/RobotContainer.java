package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.FineTunedDrive;
import frc.robot.subsystems.drivetrain.commands.StayOnTarget;
import frc.robot.subsystems.drivetrain.commands.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.commands.testing.ManiacDrive;

import java.util.function.BooleanSupplier;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public static XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final SwerveDrive swerveDrive = new SwerveDrive(true);
    private final JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private boolean hasSwerveEncoderReset = false;
    public final Pose2d target_pose = new Pose2d(
            new Translation2d(0,0), new Rotation2d(Math.toRadians(Robot.navx.getAngle()))
    );


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        a.whenPressed(Robot.navx::reset);
        b.whenPressed(new ManiacDrive(swerveDrive));
    }

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(new FineTunedDrive(swerveDrive,
                () -> -xbox.getY(GenericHID.Hand.kLeft),
                () -> xbox.getX(GenericHID.Hand.kLeft),
                () -> xbox.getX(GenericHID.Hand.kRight)
        ));

        swerveDrive.setDefaultCommand(new StayOnTarget(
                swerveDrive,
                target_pose,
                x::get)
        );
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
}

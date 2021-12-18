// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final boolean debug = true;
    private static final AHRS navx = new AHRS(SPI.Port.kMXP);
    private static double startAngle;
    //    private final Compressor compressor = new Compressor(0);
    public PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    // LEDs
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue = 0;

    /**
     * Gets the current angle of the robot in respect to the start angle.
     *
     * @return the current angle of the robot in respect to the start angle.
     */
    public static Rotation2d getAngle() {
        double currentAngle = (navx.getYaw() + 360) % 360;
        return Rotation2d.fromDegrees(Math.IEEEremainder(currentAngle - startAngle, 360));
    }

    /**
     * Resets the angle of the navx to the current angle.
     */
    public static void resetAngle() {
        resetAngle(navx.getYaw());
    }

    /**
     * Resets the angle of the navx to the current angle.
     *
     * @param angle the angle in -180 to 180 degrees coordinate system.
     */
    public static void resetAngle(double angle) {
        startAngle = (angle + 360) % 360;
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        resetAngle();
        m_robotContainer = new RobotContainer();
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
//        m_led.setData(m_ledBuffer);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
//        FalconDashboard.INSTANCE.setFollowingPath(true);
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(1);
        System.out.println("ikojasfjkasdkjaskljdaskljd");

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(5);
        m_led.setLength(m_ledBuffer.getLength());

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}

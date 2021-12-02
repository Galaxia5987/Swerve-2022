package frc.robot.utils;

public class Utils {
    /**
     * Gets the minimal error between two desired angles.
     *
     * @param targetAngle the desired angle. [rad]
     * @param currentAngle current angle. [rad]
     * @return the distance between the angles. [rad]
     */
    public static double getTargetError(double targetAngle, double currentAngle) {
        double cwDistance = targetAngle - currentAngle;
        double ccwDistance = 2 * Math.PI - (Math.abs(cwDistance));
        if (Math.abs(cwDistance) < ccwDistance) {
            return cwDistance;
        } else if (cwDistance < 0) {
            return ccwDistance;
        }
        return -ccwDistance;
    }
}

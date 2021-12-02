package frc.robot.utils;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants;

public class StateSpaceUtils {

    /**
     * Creates a velocity linear system for the state space model.
     * @param motor the type of the motor for the plant.
     * @param gearRatio The gear ratio of the plant.
     * @param J the moment of inertia of the system.
     * @return A {@link edu.wpi.first.wpilibj.system.LinearSystem} for the system.
     */
    public static LinearSystem<N1, N1, N1> createVelocityLinearSystem(Constants.Motor motor, double gearRatio, double J) {
        if (J == 0) throw new RuntimeException("J must have non-zero value");
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf Page 76
        Vector<N1> A = VecBuilder.fill(-Math.pow(gearRatio, 2) * motor.Kt / (motor.Kv * motor.omega * J));
        Vector<N1> B = VecBuilder.fill((gearRatio * motor.Kt) / (motor.omega * J));
        return new LinearSystem<>(A, B, Matrix.eye(Nat.N1()), new Matrix<>(Nat.N1(), Nat.N1()));
    }
}

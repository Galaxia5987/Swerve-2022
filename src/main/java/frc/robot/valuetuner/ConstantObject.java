package frc.robot.valuetuner;

/**
 * This class holds a constant value by key.
 */
public class ConstantObject {
    private final String key;
    private double value;

    public ConstantObject(String key, double value) {
        this.key = key;
        this.value = value;
    }

    public double getValue() {
        return value;
    }

    public void setValue(double value) {
        this.value = value;
    }

    public String getKey() {
        return key;
    }

}

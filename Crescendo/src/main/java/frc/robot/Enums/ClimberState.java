
package frc.robot.enums;

public enum ClimberState {
    // TODO Update values
    PovUp(-0.15, -0.15),
    PovDown(0.30, 0.30),
    // TODO possibly invert the rest of these?
    PovLeft(0.15, -0.15),
    PovRight(-0.15, 0.15),

    PovDownLeft(0.15, 0),
    PovDownRight(0, 0.15),

    PovUpLeft(-0.15, 0),
    PovUpRight(0, -0.15);

    private final double leftSpeedPercent;
    private final double rightSpeedPercent;

    private ClimberState(double leftSpeedPercent, double rightSpeedPercent) {
        this.leftSpeedPercent = leftSpeedPercent;
        this.rightSpeedPercent = rightSpeedPercent;
    }

    public double getLeftSpeedPercent() {
        return leftSpeedPercent;
    }

    public double getRightSpeedPercent() {
        return rightSpeedPercent;
    }
}

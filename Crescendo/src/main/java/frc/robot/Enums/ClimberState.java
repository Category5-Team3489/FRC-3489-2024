
package frc.robot.enums;

public enum ClimberState {
    PovUp(1, 1),
    PovDown(-1, -1),
    PovLeft(-1, 1),
    PovRight(1, -1),

    PovDownLeft(-0.5, 0),
    PovDownRight(0, -0.5),
    PovUpLeft(0.5, 0),
    PovUpRight(0, 0.5);

    private final double aSpeedPercent;
    private final double bSpeedPercent;

    private ClimberState(double aSpeedPercent, double bSpeedPercent) {
        this.aSpeedPercent = aSpeedPercent;
        this.bSpeedPercent = bSpeedPercent;
    }

    public double getLeftSpeedPercent() {
        return aSpeedPercent;
    }

    public double getRightSpeedPercent() {
        return bSpeedPercent;
    }
}


package frc.robot.enums;

public enum ClimberState {
    //TODO Update values
    PovUp(0.15, 0.15),
    PovDown(-0.15, -0.15),
    PovLeft(-0.15, 0.15),
    PovRight(0.15, -0.15),

    PovDownLeft(-0.15, 0),
    PovDownRight(0, -0.15),
    PovUpLeft(0.15, 0),
    PovUpRight(0, 0.15);

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

package frc.robot.enums;

public enum ClimberState {
    Up(0.25),
    Down(-0.25),
    Off(0);

    private final double climberSpeed;

    private ClimberState(double climberSpeed) {
        this.climberSpeed = climberSpeed;
    }

    public double getSpeed() {
        return climberSpeed;
    }

}

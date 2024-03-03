package frc.robot.enums;

public enum IntakeState {
    // TODO update values
    Out(-0.80),
    In(0.80),
    Off(0);

    private final double intakeSpeed;

    private IntakeState(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
    }

    public double getSpeed() {
        return intakeSpeed;
    }

}

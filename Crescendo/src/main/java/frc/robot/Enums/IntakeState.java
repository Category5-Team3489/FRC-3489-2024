package frc.robot.Enums;

public enum IntakeState {
    In(0.25),
    Out(-0.25),
    Off(0);

    private final double intakeSpeed;

    private IntakeState(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
    }

    public double getSpeed() {
        return intakeSpeed;
    }

}

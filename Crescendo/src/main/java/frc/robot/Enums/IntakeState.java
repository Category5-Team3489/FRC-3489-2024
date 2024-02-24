package frc.robot.enums;

public enum IntakeState {
    //TODO update values
    In(0.70),
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

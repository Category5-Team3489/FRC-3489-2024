package frc.robot.enums;

public enum IntakeState {
    // TODO update values
    centerOut(1),
    centerIn(-1),
    Off(0),
    falconIn(-0.6),
    falconOut(0.6);

    private final double intakeSpeed;

    private IntakeState(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
    }

    public double getSpeed() {
        return intakeSpeed;
    }

    public String getStateName(IntakeState intakeState) {

        return intakeState.toString();
    }

}

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

public class IntakeUntilDetection extends Command {

    private final Intake intake;
    private final Index belt;

    public IntakeUntilDetection(Intake intake, Index belt) {
        this.intake = intake;
        this.belt = belt;

        addRequirements(intake, belt);
    }

    @Override
    public void execute() {
        // start intake/belt
        intake.setSpeedState(IntakeState.In);
        belt.moveIndex(IndexState.Belt_1);
    }

    @Override
    public void end(boolean interrupted) {
        belt.moveIndex(IndexState.StopIndex);
        intake.stopMotors();
    }

    @Override
    public boolean isFinished() {
        // wait to get sensor value (could have this start shooter motor)
        return belt.isNoteDetected();
    }
}

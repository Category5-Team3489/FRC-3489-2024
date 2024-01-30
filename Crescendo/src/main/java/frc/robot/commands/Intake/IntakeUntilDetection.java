package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Enums.BeltState;
import frc.robot.Enums.IntakeState;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;

public class IntakeUntilDetection extends Command {

    private final Intake intake;
    private final Belt belt;

    public IntakeUntilDetection(Intake intake, Belt belt) {
        this.intake = intake;
        this.belt = belt;

        addRequirements(intake, belt);
    }

    @Override
    public void execute() {
        // start intake/belt
        intake.setBoth(IntakeState.In);
        belt.moveBelt(BeltState.Belt_1);
    }

    @Override
    public void end(boolean interrupted) {
        belt.moveBelt(BeltState.Belt_nomove);
        intake.stopMotors();
    }

    @Override
    public boolean isFinished() {
        // wait to get sensor value (could have this start shooter motor)
        return belt.isNoteDetected();
    }
}

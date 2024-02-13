package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

public class Outtake extends Command {

    private final Intake intake;
    private final Index belt;

    public Outtake(Intake intake, Index belt) {
        this.intake = intake;
        this.belt = belt;

        addRequirements(intake, belt);
    }

    @Override
    public void execute() {
        // set intake to out
        intake.intakeCommand(IntakeState.Out);
        // set belt to out
        belt.indexCommand(IndexState.Outtake);
        // belt.moveIndex(IndexState.Outtake);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.ExampleSubsystem;
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
    public boolean isFinished() {
        return belt.isNoteDetected();
    }
}

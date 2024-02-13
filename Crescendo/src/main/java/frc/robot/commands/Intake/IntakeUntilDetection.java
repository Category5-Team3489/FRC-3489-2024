package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LedState;

public class IntakeUntilDetection extends Command {

    private final Intake intake;
    private final Index belt;
    private final LEDs leds;
    private static IntakeUntilDetection instance = new IntakeUntilDetection();

    public static IntakeUntilDetection get() {
        return instance;
    }

    public IntakeUntilDetection() {
        this.intake = Intake.get();
        this.belt = Index.get();
        this.leds = LEDs.get();

        addRequirements(intake, belt);
    }

    @Override
    public void execute() {

        // Check to see if note is loaded, via index class
        if (belt.isNoteDetected()) {
            return;
        }

        // start intake/belt
        intake.intakeCommand(IntakeState.In);
        belt.indexCommand(IndexState.Belt_1);
    }

    @Override
    public void end(boolean interrupted) {
        belt.stop();
        intake.stop();
        leds.setLeds(LedState.Intaked);
    }

    @Override
    public boolean isFinished() {
        // wait to get sensor value (could have this start shooter motor)
        return belt.isNoteDetected();
    }
}

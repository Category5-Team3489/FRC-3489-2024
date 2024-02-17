package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.LEDs.LedState;

public class IntakeUntilDetection extends Command {

    private final Intake intake;
    private final Index belt;
    private final LEDs leds;
    private final ShooterAngle shooterAngle;

    public IntakeUntilDetection() {
        this.intake = Intake.get();
        this.belt = Index.get();
        this.leds = LEDs.get();
        this.shooterAngle = ShooterAngle.get();


        addRequirements(intake, belt, leds, shooterAngle);
    }

    // TODO Maybe spin up shooter motors after a piece is detected

    @Override
    public void execute() {

        // Check to see if note is loaded, via index class
        if (belt.isNoteDetected()) {
            return;
        }

        // start intake/belt
        intake.intakeCommand(IntakeState.In);
        belt.indexCommand(IndexState.Intake);
        shooterAngle.setAngle(ShooterAngleState.Max.getAngle());

        System.out.println("------Intake Until detection");
    }

    @Override
    public void end(boolean interrupted) {
        belt.stop();
        intake.stop();
        leds.setLeds(LedState.Intaked);
        //TODO reset shooter angle to home
    }

    @Override
    public boolean isFinished() {
        // wait to get sensor value (could have this start shooter motor)
        return belt.isNoteDetected();
    }
}

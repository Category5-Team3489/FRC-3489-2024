package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.LEDs.LedState;

public class Outtake extends Command {

    private final Intake intake;
    private final Index belt;
    private final LEDs leds;

    public boolean hasOuttakeBeenSet = false;

    public Outtake() {
        this.intake = Intake.get();
        this.belt = Index.get();
        this.leds = LEDs.get();

        addRequirements(intake, belt, leds);
    }

    @Override
    public void execute() {

        // start intake/belt
        intake.intakeCommand(IntakeState.Out).schedule();
        // TODO fix outtake enum
        belt.indexCommand(IndexState.Intake).schedule();
        hasOuttakeBeenSet = true;

        System.out.println("----Outtake Command");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("End");
        leds.setLeds(LedState.Outtake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

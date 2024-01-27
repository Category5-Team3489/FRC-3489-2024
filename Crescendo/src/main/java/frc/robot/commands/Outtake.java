package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Enums.BeltState;
import frc.robot.Enums.IntakeState;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;

public class Outtake extends Command {
    private final Intake intake;
    private final Belt belt;

    public Outtake(Intake intake, Belt belt) {
        this.intake = intake;
        this.belt = belt;
        
        addRequirements(intake, belt);
    }

    @Override
    public void execute() {
        //set intake to out
        intake.setBoth(IntakeState.Out);
        //set belt to out
        belt.moveBelt(BeltState.BeltOut);
    }

}

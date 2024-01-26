package frc.robot.commands;

import frc.robot.Enums.IntakeState;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakeBoth extends Command {

  private final Intake intake;
  private final IntakeState state;

  public SetIntakeBoth(Intake intake, IntakeState state ) {
    this.intake = intake;
    this.state = state;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setBoth(state);
    //move belts
  }
}


//start intake/belt
//wait to get sensor value (could have this start shooter motor)
//stop
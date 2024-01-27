package frc.robot.commands;

import frc.robot.Enums.IntakeState;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakeBoth extends Command {

  private final Intake intake;
  private final Belt belt;
  private double speed;

  public SetIntakeBoth(Intake intake,Belt belt, double speed) {
    this.intake = intake;
    this.belt = belt;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setDoubleBoth(speed);
    //move belts
    belt.moveDoubleBelt(speed);
  }
}


//start intake/belt
//wait to get sensor value (could have this start shooter motor)
//stop
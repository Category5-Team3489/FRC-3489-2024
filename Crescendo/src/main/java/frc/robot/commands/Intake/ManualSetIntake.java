package frc.robot.commands.Intake;

import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ManualSetIntake extends Command {

  private final Intake intake;
  private final Index belt;
  private double speed;

  public ManualSetIntake(Intake intake, Index belt, double speed) {
    this.intake = intake;
    this.belt = belt;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, belt);
  }

  public Command manualSetIntake() {
    return Commands.run(() -> {
      intake.setDoubleSpeed(speed);
      // belt.moveDoubleIndex(speed);
    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setDoubleSpeed(speed);
    // move belts
    // belt.moveDoubleIndex(speed);
  }
}
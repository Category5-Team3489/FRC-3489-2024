package frc.robot.commands.Intake;

import frc.robot.enums.IntakeState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ManualSetIntake extends Command {

  private final Intake intake;
  private final Index belt;

  public ManualSetIntake(Intake intake, Index belt) {
    this.intake = intake;
    this.belt = belt;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, belt);
  }

  public Command manualSetIntake() {
    return Commands.run(() -> {
      intake.intakeCommand(IntakeState.In);
      // belt.moveDoubleIndex(speed);
    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeCommand(IntakeState.In);
    // move belts

    // belt.moveDoubleIndex(speed);
  }
}
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.autoShooting.AutoDrive;
import frc.robot.subsystems.ShooterSpeed;
import frc.robot.subsystems.ShooterAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

public class AutonomousCoralIntake extends ParallelCommandGroup {
    private final CoralDrive coralDrive = new CoralDrive();
    private final CoralAim coralAim = new CoralAim(coralDrive);
    private final IntakeUntilDetectionAngle intake = new IntakeUntilDetectionAngle();

    private final Command indexCommand = Index.get().indexCommand(IndexState.Intake);
    private final Command intakeAngle = ShooterAngle.get().updateCommand(ShooterAngleState.Max.getAngle());
    private final Command intakeCommand = Intake.get().intakeCommand(IntakeState.centerIn, IntakeState.falconIn);

    // TODO test
    public AutonomousCoralIntake() {
        addCommands(
                // schedule drive
                coralDrive,
                Commands.print("Coral Drive started"),

                // start intake
                intake,
                // Commands.print("Intake started"),

                // aim to game piece and drive to it // drive forward robot centric
                coralAim
        // Commands.print("Coral Aim Done")

        );
    }
}
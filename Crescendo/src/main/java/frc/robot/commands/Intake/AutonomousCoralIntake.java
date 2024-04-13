package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.autoShooting.AutoDrive;
import frc.robot.subsystems.ShooterSpeed;
import frc.robot.subsystems.ShooterAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

public class AutonomousCoralIntake extends SequentialCommandGroup {
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
                coralDrive.withTimeout(8),
                Commands.print("Coral Drive started"),

                // start intake
                intake.withTimeout(4),
                Commands.print("Intake started"),

                // indexCommand,
                // intakeAngle,
                // intakeCommand,

                // aim to game piece and drive to it // drive forward robot centric
                coralAim.withTimeout(4),
                Commands.print("Coral Aim Done")

        );
    }
}
package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class IntakeUntilDetectionAngle extends SequentialCommandGroup {

    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final Index index = Index.get();
    private final Intake intake = Intake.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

    public IntakeUntilDetectionAngle() {
        System.out.println("Intake Command scheduled");
        Command angleCommand = shooterAngle.updateCommand(() -> ShooterAngleState.Max.getAngle());

        addCommands(
                Commands.runOnce(() -> {
                    angleCommand.schedule();
                    intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn).schedule();
                    index.indexCommand(IndexState.Intake).schedule();
                    shooterSpeed.stopCommand().schedule();
                    System.out.println("++++++++++Set Commands Scheduled");
                    intake.hasIntakeBeenSet = true;
                }),
                Commands.waitSeconds(0.75));
        Commands.runOnce(() -> {
            angleCommand.cancel();
        });
    }
}
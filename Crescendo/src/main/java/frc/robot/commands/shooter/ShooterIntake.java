package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterSpeed;
import edu.wpi.first.wpilibj.Timer;

public class ShooterIntake extends Command {
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

    private final Index index = Index.get();
    private final Timer timer = new Timer();

    public ShooterIntake() {
        addRequirements(shooterSpeed, index);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
        shooterSpeed.setSpeedPercent(ShooterSpeed.ShooterIntakeSpeedPercent);
        index.indexCommand(IndexState.Intake);

        if (index.isNoteDetected()) {
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        // This needs to wait a couple seconds because it wil hit the sensor before
        // being properly loaded (probably)

        // index.moveIndex(IndexState.StopIndex);
        shooterSpeed.setSpeedPercent(0); // Just a placeholder value
        index.stop();
    }
}

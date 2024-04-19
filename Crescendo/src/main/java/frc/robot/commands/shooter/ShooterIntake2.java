package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class ShooterIntake2 extends Command {

    private final ShooterSpeed shooterSpeed;
    private final Index belt;
    private final ShooterAngle shooterAngle;

    public boolean hasShooterIntakeBeenSet = false;

    public ShooterIntake2() {
        this.shooterSpeed = ShooterSpeed.get();
        this.belt = Index.get();
        this.shooterAngle = ShooterAngle.get();

        addRequirements(belt, shooterAngle, shooterSpeed);
    }

    @Override
    public void execute() {

        // start intake/belt
        shooterSpeed.updateCommand(() -> -0.2).schedule();
        // TODO outtake
        belt.indexCommand(IndexState.Outtake).schedule();

        hasShooterIntakeBeenSet = true;

        System.out.println("----Shooter2 Until detection Command");
    }

    @Override
    public void end(boolean interrupted) {
        // belt.stop();
        // intake.stop();
        // belt.indexCommand(IndexState.Stop).cancel();
        // intake.intakeCommand(IntakeState.Off).cancel();
        System.out.println("End");
        // TODO reset shooter angle to home?
    }

    @Override
    public boolean isFinished() {
        // wait to get sensor value (could have this start shooter motor)
        // return belt.isNoteDetected();
        return false;
    }
}

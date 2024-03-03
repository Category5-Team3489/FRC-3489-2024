package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DrivetrainConstants;
import frc.robot.commands.DriveSeconds;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.subsystems.Drivetrain;

public class Leave extends Command {
    private final Drivetrain drivetrain;



    public Leave(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        final DriveSeconds driveSeconds = new DriveSeconds(drivetrain, 3, 0.15);
        driveSeconds.schedule();
        System.out.println("AUTO INIT!");
    }

    
}

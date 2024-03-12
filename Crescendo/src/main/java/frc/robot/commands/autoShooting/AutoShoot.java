package frc.robot.commands.autoShooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class AutoShoot extends SequentialCommandGroup {
    private final AprilLimelight aprilLimelight = AprilLimelight.get();
    private final Drivetrain drivetrain = Drivetrain.get();
    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

    public AutoShoot() {
        // In Parallel:
        // - Set shooter to constant speed
        // - Adjust robot heading towards value calculated from limelight
        // - Adjust shooter angle towards value calculated from lookup table
        // When Done:
        // - Index
        // - Delay Seconds
        // - Stop shooter
        // - Home shooter angle

        addCommands(
            // TODO Continue
            Commands.runOnce(() -> 
                Commands.parallel(
                shooterSpeed.updateCommand(60)
                
                )
                
            )
            
        );
    }
}
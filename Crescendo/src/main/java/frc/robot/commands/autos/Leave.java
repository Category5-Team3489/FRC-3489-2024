package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Leave extends SequentialCommandGroup {
    private final Drivetrain drivetrain;

    public Leave(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // addCommands(
        // new DrivePercentAngleSeconds(actions.drivetrain, 0.12, 0, 6.75)
        // );
        // addCommands(
        //         drivetrain.applyRequestOnce(() -> driveFacingAngle
        //                 .withVelocityX()
        //                 .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond * drivetrain.getSpeedLimit())
        //                 .withTargetDirection(Rotation2d.fromDegrees(0))));
    }
}

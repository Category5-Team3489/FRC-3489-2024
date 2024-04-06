package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Cat5Utils;
import frc.robot.Constants;
import frc.robot.commands.Intake.AutoCoralIntake;
import frc.robot.commands.Intake.CoralIntake;
import frc.robot.commands.autoShooting.AutoShoot;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.CoralLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

public class SourceCenterLineManual {
    private Index index = Index.get();
    private ShooterAngle shooterAngle = ShooterAngle.get();
    private Intake intake = Intake.get();
    private Drivetrain drivetrain = Drivetrain.get();
    private CoralLimelight coralLimelight = CoralLimelight.get();

    private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
    private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

    Trigger laserTrigger = new Trigger(index.laserSensor::get);

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxMetersPerSecond * 0.1)
            .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    Command autoShoot = new AutoShoot();

    public boolean hasRunBefore = false;

    // Command autoIntakeCommand = new CoralIntake();

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]

    // Rotate/drive side
    double percentY = Cat5Utils.Red(-0.6);
    double percentX = 0;
    double percentOmega = Cat5Utils.Red(-0.2);
    double driveTimeSeconds = 0.8;

    // wall to note center 30.5 in amp side
    // intake drive
    double percentY2 = Cat5Utils.Red(-0.3); // Blue alliance hit stage (make this larger?) //22 was good for stage
                                            // but too left of ring
    double percentX2 = 0.6;
    double percentOmega2 = 0.0;
    double driveTimeSeconds2 = 6;

    // drive back
    double percentY3 = Cat5Utils.Red(0.3);
    double percentX3 = -0.6;
    double percentOmega3 = 0;
    double driveTimeSeconds3 = 5;

    // drive forward intake
    double percentY4 = 0;
    double percentX4 = -0.3;
    double percentOmega4 = 0;
    double driveTimeSeconds4 = 1;

    double speedMultiplier = 0.5; // [0, 1]

    Command shooterIndex = index.indexCommand(IndexState.Intake);

    AutoCoralIntake autoCoralIntake = new AutoCoralIntake();

    Command autoCoralIntakeCommand = autoCoralIntake.onlyWhile(() -> autoCoralIntake.shouldAutoIntake);

    Command closeShootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

    Command driveCommandSideRotate = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandDriveCenter = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX2 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY2 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega2 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandIntake = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX4 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY4 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega4 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandBack = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX3 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY3 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega3 * MaxRadiansPerSecond * speedMultiplier));

    // Command driveCommandForward = drivetrain.applyRequest(() -> drive
    // .withVelocityX(percentX4 * MaxMetersPerSecond * speedMultiplier)
    // .withVelocityY(-percentY4 * MaxMetersPerSecond * speedMultiplier)
    // .withRotationalRate(-percentOmega4 * MaxRadiansPerSecond * speedMultiplier));

    public Command sourceCenterLineManual() {
        return Commands.parallel(closeShootCommand, Commands.waitSeconds(3))

                .andThen(Commands.parallel(shooterIndex), Commands.waitSeconds(2))

                // rotate to forward and drive to the side away from the speaker
                .andThen(driveCommandSideRotate.withTimeout(driveTimeSeconds))

                // Start intake
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))

                // Drive forward
                .andThen(driveCommandDriveCenter.withTimeout(driveTimeSeconds2))
                // Cancle the drive command
                .andThen(() -> driveCommandDriveCenter.cancel())

                .andThen(Commands.print("Drive to Center Done"))

                // Drive forward to intake
                .andThen(driveCommandIntake.withTimeout(driveTimeSeconds4))

                .andThen(() -> laserTrigger
                        .onTrue(Commands.runOnce(() -> {
                            if (intake.hasIntakeBeenSet) {
                                index.stop();
                                intake.stop();
                            }
                        })))

                .andThen(driveCommandBack.withTimeout(driveTimeSeconds3))

                .andThen(() -> autoShoot.schedule())

                // .andThen(() -> autoCoralIntakeCommand.schedule())
                .withName("SourceCenterLineManual");

    }

    // TODO Test if the other one does not work
    // .andThen(autoCoralIntakeCommand.withTimeout(4))

    // TODO possibly update time to stop auto intake
    // Stop intake when piece detected

    // // drive back to see apriltag
    // .andThen(driveCommandBack.withTimeout(driveTimeSeconds3))

    // // Auto shoot
    // .andThen(() -> autoShoot.schedule())

}

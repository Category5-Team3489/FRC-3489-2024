// package frc.robot.commands.autos;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

// import edu.wpi.first.math.filter.Debouncer.DebounceType;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants;
// import frc.robot.commands.Intake.IntakeUntilDetectionAngle;
// import frc.robot.commands.autoShooting.AutoShoot;
// import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
// import frc.robot.enums.IndexState;
// import frc.robot.enums.IntakeState;
// import frc.robot.enums.ShooterAngleState;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Index;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.ShooterAngle;
// import frc.robot.subsystems.ShooterSpeed;

// public class SideShootIntakeShoot extends Command {
//     private final Drivetrain drivetrain = Drivetrain.get();
//     private final ShooterAngle shooterAngle = ShooterAngle.get();
//     private final Index index = Index.get();
//     private final Intake intake = Intake.get();

//     private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
//     private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

//     Trigger laserTrigger = new Trigger(index.laserSensor::get);

//     final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//             .withDeadband(MaxMetersPerSecond * 0.1)
//             .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
//             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//     Command closeShootCommand = new SetShooterSpeedAndAngle(
//             Constants.ShooterAngle.CloseShooterAngle,
//             Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

//     Command autoShoot = new AutoShoot().withTimeout(2);
//     Command autoShoot2 = new AutoShoot().withTimeout(2);


//     Command shooterIndex = index.indexCommand(IndexState.Intake);
//     Command shooterIndex2 = index.indexCommand(IndexState.Intake);

//     // Ensure percentages are greater than the 0.1 percent deadband above
//     // Domain is [-1, 1]
//     double percentY = 0.3;
//     double percentX = 0;
//     double percentOmega = 2;
//     double driveTimeSeconds = 2.6; // 3 was to far for limelight-- 2 was not enough for intake
//     double driveTimeSeconds2 = 2;

//     double speedMultiplier = 0.5; // [0, 1]

//     Command driveCommandForward = drivetrain.applyRequest(() -> drive
//             .withVelocityX(percentX * MaxMetersPerSecond * speedMultiplier)
//             .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
//             .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

//     Command driveCommandForward2 = drivetrain.applyRequest(() -> drive
//             .withVelocityX(percentX * MaxMetersPerSecond * speedMultiplier)
//             .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
//             .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

//     Command driveCommandForward3 = drivetrain.applyRequest(() -> drive
//             .withVelocityX(percentX * MaxMetersPerSecond * speedMultiplier)
//             .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
//             .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

//     final IntakeUntilDetectionAngle intakeUntilDetection = new IntakeUntilDetectionAngle();

    
//     public Command sideShootIntakeShoot() {

//         // start facing april tag
//         // auto stoot
//         // rotate to straight
//         // drive forward and intake
//         // drive back to see april tag
//         // auto shoot

//         return Commands.runOnce(() -> autoShoot.schedule())
//                 .andThen(driveCommandForward.withTimeout(driveTimeSeconds))

//                 .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
//                 .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
//                 .andThen(index.indexCommand(IndexState.Intake))

//                 .andThen(driveCommandForward2.withTimeout(driveTimeSeconds2))


//                 .andThen(() -> laserTrigger
//                         .debounce(0.29, DebounceType.kRising)
//                         .onTrue(Commands.runOnce(() -> {
//                             if (intake.hasIntakeBeenSet) {
//                                 intake.stop();
//                                 index.stop();
//                             }
//                         })))

//                 .andThen(driveCommandForward3.withTimeout(1))

//                 .andThen(autoShoot2)

//                 .withName("ShootIntakeAutoShoot");
//     }
// }

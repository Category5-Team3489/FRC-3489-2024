// package frc.robot.commands.autoShooting;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.AprilLimelight;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Index;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.ShooterAngle;
// import frc.robot.subsystems.ShooterSpeed;

// public class AutoShootTest extends Command {

//     private Drivetrain drivetrain = Drivetrain.get();

//     private final ShooterAngle shooterAngle = ShooterAngle.get();
//     private final AprilLimelight limelight = AprilLimelight.get();


//     public AutoShootTest() {
//         this.drivetrain = drivetrain;
//         this.shooterAngle = shooterAngle;
//         this.limelight = limelight;

//         addRequirements(drivetrain, shooterAngle, limelight);

//         double targetX = limelight.getTargetX();
//         double targetY = limelight.getTargetY();

//         Shuffleboard.getTab("Testing")
//                 .addDouble("Limelight X", () -> targetX)
//                 .withSize(1, 1)
//                 .withPosition(5, 2);

//         Shuffleboard.getTab("Testing")
//                 .addDouble("Limelight Y", () -> targetY)
//                 .withSize(1, 1)
//                 .withPosition(2, 2);
//     }

//     @Override
//     public void execute() {
//         double targetX = limelight.getTargetX();
//         double targetY = limelight.getTargetY();
//     }

//     // rotate to target based on tx/pid
//     // set shooter angle based on distance from apriltag
//     // set shooter speed to constant speed
//     // index piece

// }

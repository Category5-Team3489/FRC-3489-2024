package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.CoralLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

public class CoralIntake extends Command {
    private final Drivetrain drivetrain = Drivetrain.get();
    private final CoralLimelight coralLimelight = CoralLimelight.get();
    private final Intake intake = Intake.get();
    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final Index index = Index.get();

    private final IntakeUntilDetection intakeCommand = new IntakeUntilDetection();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    Command driveCommandForward = drivetrain.applyRequest(() -> drive
            .withVelocityX(getDrivetrainVelocityX())
            .withVelocityY(getDrivetrainVelocityY())
            .withRotationalRate(getDrivetrainAngleRate()));

    //TODO Test
    Command intakeCommandm = intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn);
    Command shooterAngleCommand = shooterAngle.updateCommand(ShooterAngleState.Max.getAngle());
    Command indexCommand = index.indexCommand(IndexState.Intake);


    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    private double speed = (16.5 / 3.281) * 0.3;

    private boolean isFinished = false;

    private double rotationSpeed = 0.12 * Constants.Drivetrain.MaxRadiansPerSecond;

    // TODO Test Ranges
    private final double tXRange = 16;       //3 -- 5
    private final double tYRange = 5;

    private double currentDegrees;

    public CoralIntake() {
        Commands.runOnce(() -> Commands.parallel(
                drivetrain.applyFieldCentricFacingAngle(() -> 0, () -> 0, () -> 0)));
    }

    @Override
    public void initialize() {
        //TODO Test
        intakeCommandm.schedule();
        indexCommand.schedule();
        shooterAngleCommand.schedule();
    }

    @Override
    public void execute() {
        double targetX = coralLimelight.getTargetX();
        double targetY = coralLimelight.getTargetY();
        double targetV = coralLimelight.getTargetVisible();

        // TODO Test Acuracy
        // currentDegrees = drivetrain.getCurrentHeading();
        // System.out.println("Current degree: " + currentDegrees);
        

        // Return if april tag is not visible
        if (targetV == 0) {
            System.out.println("Note is not visible");
            return;
        }

        // If within ty range
        // if (targetY <= tYRange) {
            // if within tx range
            if (Math.abs(targetX) < tXRange) {
                System.out.println("----EQUAL------");
                // isFinished = true;
                // intake.schedule();
                driveCommandForward.cancel();
                // drivetrainAngleRate = 0;
                // drivetrainVelocityX = getXVelocityCalculation(currentDegrees);
                // drivetrainVelocityY = getYVelocityCalculation(currentDegrees);
                // driveCommandForward.schedule();
                return;

            } else if (targetX < 0) { // if less than target
                drivetrainAngleRate = -rotationSpeed;
                driveCommandForward.schedule();
                System.out.println("target < 0");
                // isFinished = false;
            } else if (targetX > 0) { // if greater than target
                drivetrainAngleRate = rotationSpeed;
                driveCommandForward.schedule();
                System.out.println("target > 0");
                // isFinished = false;

            }
        // }
    }

    //This works but does not allow the robot to corect if it overshoots
    @Override
    public boolean isFinished() {
        //  return isFinished;
        return false;
    }

    //This works but does not allow the robot to corect if it overshoots
    @Override
    public void end(boolean interrupted) {
        intakeCommand.schedule();
        driveCommandForward.cancel();
        // drivetrainAngleRate = 0;
        // drivetrainVelocityX = getXVelocityCalculation(currentDegrees);
        // drivetrainVelocityY = getYVelocityCalculation(currentDegrees);
        // driveCommandForward.schedule();
        System.out.println("===========================End");
    }

    private double getDrivetrainAngleRate() {
        return drivetrainAngleRate;
    }

    private double getDrivetrainVelocityX() {
        return drivetrainVelocityX;
    }

    private double getDrivetrainVelocityY() {
        return drivetrainVelocityY;
    }

    private double getXVelocityCalculation(double angle) {
        return Math.sin(angle) * speed;
    }

    private double getYVelocityCalculation(double angle) {
        return Math.cos(angle) * speed;
    }
}

// press button
// check if note is visible
// if with in ty degree range
// rotate depending on tx value until tx == 0
// start intake
// drive forward
// wait for laser value then shooter = home
// if outside
// print error
// return

// is the current heading output correct?
//

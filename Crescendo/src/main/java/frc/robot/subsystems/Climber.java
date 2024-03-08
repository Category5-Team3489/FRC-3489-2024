package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.ClimberState;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    // Singleton
    private static final Climber instance = new Climber();

    public boolean isClimberLocked = false;

    public static Climber get() {
        return instance;
    }

    // Constants
    private static final int LeftMotorId = 15;
    private static final int RightMotorId = 16;
    private static final int rightServoChannel = 8;
    private static final int leftServoChannel = 9;

    // Devices
    private final TalonFX leftMotor = new TalonFX(LeftMotorId);
    private final TalonFX rightMotor = new TalonFX(RightMotorId);

    private final Servo rightServo = new Servo(rightServoChannel);
    private final Servo leftServo = new Servo(leftServoChannel);

    private Climber() {
        rightMotor.setInverted(true);

        setDefaultCommand(Commands.runOnce(() -> {
            // System.out.println("Stopped");
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }, this));

        Shuffleboard.getTab("Main")
                    .addDouble("right position", () -> rightMotor.getPosition().getValueAsDouble())
                    .withSize(1, 1)
                    .withPosition(2, 5);
    }

    public Command setServos(double rightAngle, double leftAngle) {
        return Commands.run(() -> {
            // TODO Set the correct angles
            rightServo.setAngle(rightAngle);
            leftServo.setAngle(leftAngle);


            if (rightAngle == 120 && leftAngle == 0) {
                isClimberLocked = false;
            } else {
                isClimberLocked = true;
            }
            System.out.println("=========Right Servo Angle: " + rightAngle + " ----Left: " + leftAngle);
        }, this);
    }

    public Command climberCommand(ClimberState state) {
        return Commands.run(() -> {
        
            if (rightMotor.getPosition().getValueAsDouble() >= 20 && state.getRightSpeedPercent() >= 0) {
                rightMotor.stopMotor();
            }

            else if (leftMotor.getPosition().getValueAsDouble() >= 20 && state.getLeftSpeedPercent() >= 0) {
                leftMotor.stopMotor();
            }
            else {
                leftMotor.set(state.getLeftSpeedPercent());
                rightMotor.set(state.getRightSpeedPercent());

            }

            System.out.println("+++++++++++Right motor position: " + rightMotor.getPosition());
            System.out.println("+++++++++++Left motor position: " + leftMotor.getPosition());


           
        }, this);
    }
}
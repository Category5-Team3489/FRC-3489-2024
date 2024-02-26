package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.ClimberState;

import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {
    // Singleton
    private static final Climber instance = new Climber();

    public static Climber get() {
        return instance;
    }

    // Constants
    private static final int LeftMotorId = 15;
    private static final int RightMotorId = 16;

    // Devices
    private final TalonFX leftMotor = new TalonFX(LeftMotorId);
    private final TalonFX rightMotor = new TalonFX(RightMotorId);

    private Climber() {
        rightMotor.setInverted(true);

        setDefaultCommand(Commands.runOnce(() -> {
            // System.out.println("Stopped");
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }, this));
    }

    public Command climberCommand(ClimberState state) {
        return Commands.run(() -> {
            leftMotor.set(state.getLeftSpeedPercent());
            rightMotor.set(state.getRightSpeedPercent());
        }, this);
    }
}
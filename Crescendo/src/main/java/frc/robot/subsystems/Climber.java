package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.ClimberState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    // Singleton
    private static final Climber instance = new Climber();

    public static Climber get() {
        return instance;
    }

    // Constants
    private static final int LeftMotorId = 10;
    private static final int RightMotorId = 11;

    // Devices
    private final CANSparkMax leftMotor = new CANSparkMax(LeftMotorId, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(RightMotorId, MotorType.kBrushless);

    private Climber() {
        setDefaultCommand(Commands.run(() -> {
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
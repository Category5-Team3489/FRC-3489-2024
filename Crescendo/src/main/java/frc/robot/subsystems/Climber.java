package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.Utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    // Constants
    private static final int MotorAId = 10;
    private static final int MotorBId = 11;

    private static final double AscendSpeedPercent = 0.25;
    private static final double DescendSpeedPercent = -0.25;

    // Devices
    private final CANSparkMax motorA = new CANSparkMax(MotorAId, MotorType.kBrushless);
    private final CANSparkMax motorB = new CANSparkMax(MotorBId, MotorType.kBrushless);

    // Singleton
    private static final Climber instance = new Climber();
    
    public static Climber get() {
        return instance;
    }

    private Climber() {
        setDefaultCommand(Commands.run(() -> {
            System.out.println("Stopped");
            setSpeed(0);
        }, this));

        if (Utils.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motorA, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorB, DCMotor.getNEO(1));
        }
    }

    private void setSpeed(double speedPercent) {

        //TODO: Add gyro code
        motorA.set(speedPercent);
        motorB.set(-speedPercent);
    }

    public Command ascendCommand() {
        return Commands.run(() -> {
            System.out.println("Ascend");
            setSpeed(AscendSpeedPercent);
        }, this);
    }

    public Command descendCommand() {
        return Commands.run(() -> {
            System.out.println("Descend");
            setSpeed(DescendSpeedPercent);
        }, this);
    }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IndexState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Index extends SubsystemBase {
    private static Index instance = new Index();

    public static Index get() {
        return instance;
    }

    // Constants
    private static final int MotorId = 14;
    private static final int LaserSensorChannel = 9;

    // Motor Declaration- NEO 550
    private final CANSparkMax motor;
    public final DigitalInput laserSensor;

    private Index() {
        
        motor = new CANSparkMax(MotorId, MotorType.kBrushless);
        laserSensor = new DigitalInput(LaserSensorChannel);
    }

    public double getMotorSpeed() {
        return motor.get();
    }

    @Override
    public void periodic() {}

    // runs the index once
    public Command indexCommand(IndexState state) {
        return Commands.runOnce(() -> {
            System.out.println("index");
            motor.set(state.getSpeedPercent());

        }, this);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isNoteDetected() {
        return !laserSensor.get();
    }
}
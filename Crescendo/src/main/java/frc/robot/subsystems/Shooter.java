package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

    public Shooter() {}

    private final CANSparkMax beltmotor = new CANSparkMax(0, MotorType.kBrushless);

    private final TalonFX bottomShooter = new TalonFX (0); 
    private final TalonFX topShooter = new TalonFX (1);

    //Rev up motors
    public void setShooterSpeed(double shooterSpeed) {
        bottomShooter.set(-shooterSpeed);
        topShooter.set(shooterSpeed);
        
    }

    //move belt
    public void moveBelt (double beltSpeed) {
        beltmotor.set(beltSpeed);
    }







 
}
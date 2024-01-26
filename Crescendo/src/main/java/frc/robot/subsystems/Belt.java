package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Belt extends SubsystemBase {

    public Belt() {}



    private final CANSparkMax beltmotor = new CANSparkMax(0, MotorType.kBrushless);
    private final DigitalInput laserSensor = new DigitalInput(0);

    //move belt
    public void moveBelt (double beltSpeed) {
        beltmotor.set(beltSpeed);
    }

    public boolean isNoteDetected() {
        return laserSensor.get();
    }

    public void stopBelt() {
        beltmotor.stopMotor();
    }
    
    //encoder position change (constant) PID



    //start intake/belt
    //wait to get sensor value (could have this start shooter motor)
    //stop


    //set shooter speed
    //press shoot button
    //run belt/shooter
    //stop





 
}
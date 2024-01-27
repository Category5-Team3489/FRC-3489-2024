package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Enums.BeltState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Belt extends SubsystemBase {

    private final CANSparkMax beltmotor = new CANSparkMax(0, MotorType.kBrushless);
    private final DigitalInput laserSensor = new DigitalInput(0);

    private final RelativeEncoder encoder = beltmotor.getEncoder();
    
    //move belt
    public void moveBelt (BeltState beltEnumSpeed) {
        beltmotor.set(beltEnumSpeed.getbeltEnumSpeed());
    }

    public void moveBeltShooter (BeltState beltEnumSpeed) {
        beltmotor.set(beltEnumSpeed.getbeltEnumSpeed());
    }
    public void moveDoubleBelt(double beltSpeed) {
        beltmotor.set(beltSpeed);
    }

    public boolean isNoteDetected() {
        return laserSensor.get();
    }

    public void stopBelt() {
        beltmotor.stopMotor();
    }

    public double isAtSpeed(double speed) {
        return encoder.getVelocity();
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
package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;

public class CoralLimelight extends SubsystemBase {

    private static final CoralLimelight instance = new CoralLimelight();

    public static CoralLimelight get() {
        return instance;
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry ta = table.getEntry("ta");

    private CoralLimelight() {
        
        register();

        try {
            HttpCamera limelightFeed = new HttpCamera("limelight", "http://10.34.89.11:5800/stream.mjpg");//http://10.34.89.11:5800/stream.mjpg

            Shuffleboard.getTab("Main")
                    .add(limelightFeed)
                    .withWidget(BuiltInWidgets.kCameraStream)
                    .withSize(5, 4)
                    .withPosition(2, 0);
        } catch (Exception e) {
            System.out.println("Limelight camera had trouble initializing");
        }
        
        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        
    }

    @Override
    public void periodic() {
        

        

    }

}

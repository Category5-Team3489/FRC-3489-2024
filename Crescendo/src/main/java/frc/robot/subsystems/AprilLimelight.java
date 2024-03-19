package frc.robot.subsystems;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilLimelight extends SubsystemBase {
    private static AprilLimelight instance = new AprilLimelight();

    public static AprilLimelight get() {
        return instance;
    }

    // Devices
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-shooter");

    private Timer timer = new Timer();

    // Variables
    private final NetworkTableEntry tagIdEntry = limelight.getEntry("tid");
    private final NetworkTableEntry targetXEntry = limelight.getEntry("tx");
    private final NetworkTableEntry targetYEntry = limelight.getEntry("ty");
    private final NetworkTableEntry targetAreaEntry = limelight.getEntry("ta");
    private final NetworkTableEntry targetVisibleEntry = limelight.getEntry("tv");

    private double lastTargetX;
    private double lastTargetY;

    private AprilLimelight() {
        // setupAprilLimelightFeed();

        Shuffleboard.getTab("Main")
            .addDouble("Tag", () -> getTagId())
            .withSize(1, 1)
            .withPosition(7, 2);

        Shuffleboard.getTab("Testing")
                .addDouble("Limelight X", () -> getTargetX())
                .withSize(1, 1)
                .withPosition(5, 2);

        Shuffleboard.getTab("Testing")
                .addDouble("Limelight Y", () -> getTargetY())
                .withSize(1, 1)
                .withPosition(2, 2);
    }

    // private void setupAprilLimelightFeed() {
    // try {
    // HttpCamera limelightFeed = new HttpCamera("limelight-shooter",
    // "http://10.34.89.103:5800/stream.mjg");
    // Shuffleboard.getTab("Main")
    // .add(limelightFeed)
    // .withWidget(BuiltInWidgets.kCameraStream);
    // } catch (Exception e) {
    // System.out.println("Limelight camera had trouble initializing");
    // }
    // }

    private boolean isTagVisible() {
        return targetVisibleEntry.getInteger(0) == 1;
    }

    public long getTagId() {
        return tagIdEntry.getInteger(-1);
    }

    public double getTargetX() {
        if(isTagVisible()) {
            lastTargetX = targetXEntry.getDouble(Double.NaN);
        }
        return lastTargetX;
    }

    public double getTargetY() {
        if(isTagVisible()) {
            lastTargetY = targetYEntry.getDouble(Double.NaN);
        }
        return lastTargetY;
    }

    public double getTargetArea() {
        return targetAreaEntry.getDouble(Double.NaN);
    }

    public long getTargetVisible() {
        if(isTagVisible()) {
            timer.stop();
            timer.reset();
            return 1;
        }
        timer.start();
        if (timer.hasElapsed(0.5)) {
            return 0;
        }
        return 1;
    }
}
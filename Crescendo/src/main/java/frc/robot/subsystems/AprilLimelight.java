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

    // Used to determine how long
    private Timer timer = new Timer();

    // Variables
    private final NetworkTableEntry tagIdEntry = limelight.getEntry("tid");
    private final NetworkTableEntry targetXEntry = limelight.getEntry("tx");
    private final NetworkTableEntry targetYEntry = limelight.getEntry("ty");
    private final NetworkTableEntry targetAreaEntry = limelight.getEntry("ta");
    private final NetworkTableEntry targetVisibleEntry = limelight.getEntry("tv");
    private final NetworkTableEntry targetAngleEntry = limelight.getEntry("ts");

    private double lastTargetX;
    private double lastTargetY;
    private double lastTargetS;

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

    private boolean isTagVisible() {
        return targetVisibleEntry.getInteger(0) == 1;
    }

    public long getTagId() {
        return tagIdEntry.getInteger(-1);
    }

    public double getTargetX() {
        if (isTagVisible()) {
            lastTargetX = targetXEntry.getDouble(Double.NaN);
        }
        return lastTargetX;
    }

    public double getTargetY() {
        if (isTagVisible()) {
            lastTargetY = targetYEntry.getDouble(Double.NaN);
        }
        return lastTargetY;
    }

    public double getTargetS() {
        if (isTagVisible()) {
            lastTargetS = targetAngleEntry.getDouble(Double.NaN);
        }
        return lastTargetS;
    }

    public double getTargetArea() {
        return targetAreaEntry.getDouble(Double.NaN);
    }

    private boolean targetWasVisibleOnce = false;

    public long getTargetVisible() {
        if (isTagVisible()) {
            timer.stop();
            timer.reset();
            targetWasVisibleOnce = true;
            return 1;
        }
        timer.start();
        if (timer.hasElapsed(0.5) || !targetWasVisibleOnce) {
            return 0;
        }
        return 1;
    }
}
package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Robot;

public final class InterpolatorTables {
    private static InterpolatingDoubleTreeMap bluePivotAngleTable = null;
    private static InterpolatingDoubleTreeMap blueShooterRPMTable = null;
    private static InterpolatingDoubleTreeMap blueIntakeAngleTable = null;

    private static InterpolatingDoubleTreeMap redPivotAngleTable = null;
    private static InterpolatingDoubleTreeMap redShooterRPMTable = null;
    private static InterpolatingDoubleTreeMap redIntakeAngleTable = null;

    private static InterpolatingDoubleTreeMap blueTagPivotAngleTable = null;
    private static InterpolatingDoubleTreeMap blueTagShooterRPMTable = null;
    private static InterpolatingDoubleTreeMap blueTagIntakeAngleTable = null;

    private static InterpolatingDoubleTreeMap redTagPivotAngleTable = null;
    private static InterpolatingDoubleTreeMap redTagShooterRPMTable = null;
    private static InterpolatingDoubleTreeMap redTagIntakeAngleTable = null;

    private static InterpolatingDoubleTreeMap speakerToleranceTable = null;

    // Global offset for pivot angles, used for general readjustment if shots are off during event
    private static double BLUE_ANGLE_OFFSET = 0;
    private static double RED_ANGLE_OFFSET = 0;

    public static void initiateTables() {
        bluePivotAngleTable = new InterpolatingDoubleTreeMap();
        blueShooterRPMTable = new InterpolatingDoubleTreeMap();
        blueIntakeAngleTable = new InterpolatingDoubleTreeMap();

        redPivotAngleTable = new InterpolatingDoubleTreeMap();
        redShooterRPMTable = new InterpolatingDoubleTreeMap();
        redIntakeAngleTable = new InterpolatingDoubleTreeMap();

        blueTagPivotAngleTable = new InterpolatingDoubleTreeMap();
        blueTagShooterRPMTable = new InterpolatingDoubleTreeMap();
        blueTagIntakeAngleTable = new InterpolatingDoubleTreeMap();

        redTagPivotAngleTable = new InterpolatingDoubleTreeMap();
        redTagShooterRPMTable = new InterpolatingDoubleTreeMap();
        redTagIntakeAngleTable = new InterpolatingDoubleTreeMap();

        speakerToleranceTable = new InterpolatingDoubleTreeMap();


        double[][] shots = {
            // distance, apriltag position, shooter pivot angle, shooter rpm, intake angle
            {1.285, 454.4, 246, 3500, 298}, // subwoofer
            {1.781, 338.3, 238, 4000, 298},
            {2.297, 257.6, 230.5, 4000, 298},
            {2.786, 204.9, 226, 4000, 298},
            {3.28, 165.3, 222, 4000, 298},
            {3.77, 135.8, 219.5, 4100, 298},
            {4.29, 107.8, 217, 4100, 298},
            {4.808, 88.5, 215.5, 4100, 298},
            {5.3, 71.6, 214.5, 4100, 298},
            {6.032, 53.3, 213.5, 4100, 298}
        };

        for (double[] shot : shots) {
            bluePivotAngleTable.put(shot[0], shot[2] + BLUE_ANGLE_OFFSET);
            blueShooterRPMTable.put(shot[0], shot[3]);
            blueIntakeAngleTable.put(shot[0], shot[4]);

            redPivotAngleTable.put(shot[0], shot[2] + RED_ANGLE_OFFSET);
            redShooterRPMTable.put(shot[0], shot[3]);
            redIntakeAngleTable.put(shot[0], shot[4]);

            blueTagPivotAngleTable.put(shot[1], shot[2] + BLUE_ANGLE_OFFSET);
            blueTagShooterRPMTable.put(shot[1], shot[3]);
            blueTagIntakeAngleTable.put(shot[1], shot[4]);

            redTagPivotAngleTable.put(shot[1], shot[2] + RED_ANGLE_OFFSET);
            redTagShooterRPMTable.put(shot[1], shot[3]);
            redTagIntakeAngleTable.put(shot[1], shot[4]);
        }

        speakerToleranceTable.put(1.357, 8.0);
        speakerToleranceTable.put(5.06, 4.0);
    }

    public static InterpolatingDoubleTreeMap pivotAngleTable() {
        if (Robot.TAG_HEIGHT_AIM) {
            return Robot.isBlue() ? blueTagPivotAngleTable : redTagPivotAngleTable;
        }

        return Robot.isBlue() ? bluePivotAngleTable : redPivotAngleTable;
    }

    public static InterpolatingDoubleTreeMap shooterRPMTable() {
        if (Robot.TAG_HEIGHT_AIM) {
            return Robot.isBlue() ? blueTagShooterRPMTable : redTagShooterRPMTable;
        }

        return Robot.isBlue() ? blueShooterRPMTable : redShooterRPMTable;
    }

    public static InterpolatingDoubleTreeMap intakeAngleTable() {
        if (Robot.TAG_HEIGHT_AIM) {
            return Robot.isBlue() ? blueTagIntakeAngleTable : redTagIntakeAngleTable;
        }

        return Robot.isBlue() ? blueIntakeAngleTable : redIntakeAngleTable;
    }

    public static double getSpeakerTolerance(double distance) {
        return speakerToleranceTable.get(distance);
    }
}

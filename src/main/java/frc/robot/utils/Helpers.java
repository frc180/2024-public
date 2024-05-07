package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Helper class that contains static methods for common operations that don't fit anywhere else.
 */
public class Helpers {
  
  /**
   * Returns true if the translation is within the specified tolerance.
   * @param translation The Translation2d to check.
   * @param maxErrorMeters The maximum error in meters.
   * @return If the translation is within the specified tolerance.
   */
  public static boolean withinTolerance(Translation2d translation, double maxErrorMeters) {
    return Math.abs(translation.getX()) <= maxErrorMeters && Math.abs(translation.getY()) <= maxErrorMeters;
  }

  /**
   * Returns the angle (in degrees) between the given robot position and the goal.
   * @param robotPose The robot's current position.
   * @param goal The goal position.
   * @return The angle (in degrees) between the robot and the goal.
   */
  public static double angleToGoal(Pose2d robotPose, Translation2d goal) {
    Translation2d robotToGoal = goal.minus(robotPose.getTranslation());
    double angle = Math.atan2(robotToGoal.getY(), robotToGoal.getX());
    return Math.toDegrees(angle);
  }

  /**
   * Sets the inverted status of a Spark motor controller and waits for change to take effect.
   * @param spark
   */
  public static void setSparkInverted(CANSparkBase spark, boolean inverted) {
    do {
      spark.setInverted(inverted);
      sleep(100);
    } while (spark.getInverted() != inverted);
  }

  /**
   * Caps the given value between the specified range.
   * @param value The value to cap.
   * @param cap The maximum value.
   * @return The capped value.
   */
  public static double cap(double value, double cap) {
    return Math.min(Math.max(value, -cap), cap);
  }

  /**
   * Custom version of {@link ChassisSpeeds#toRobotRelativeSpeeds(ChassiSpeeds, Rotation2d)} that overwrites the given ChassisSpeeds
   * instead of returning a new one.
   * @param speeds
   * @param robotAngle
   */
  public static void fromRobotRelativeSpeedsOverwrite(ChassisSpeeds speeds, Rotation2d robotAngle) {
    // CCW rotation out of chassis frame
    var rotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(robotAngle);
    speeds.vxMetersPerSecond = rotated.getX();
    speeds.vyMetersPerSecond = rotated.getY();
  }

  final static Pose2d blankPose = new Pose2d();

  /**
   * Custom version of {@link ChassisSpeeds#discretize(ChassisSpeeds, Rotation2d)} that overwrites the given ChassisSpeeds
   * instead of returning a new one.
   * @param speeds
   * @param dtSeconds
   */
  public static void discretizeOverwrite(ChassisSpeeds speeds, double dtSeconds) {
    var desiredDeltaPose =
        new Pose2d(
            speeds.vxMetersPerSecond * dtSeconds,
            speeds.vyMetersPerSecond * dtSeconds,
            new Rotation2d(speeds.omegaRadiansPerSecond * dtSeconds));
    var twist = blankPose.log(desiredDeltaPose);

    speeds.vxMetersPerSecond = twist.dx / dtSeconds;
    speeds.vyMetersPerSecond = twist.dy / dtSeconds;
    speeds.omegaRadiansPerSecond = twist.dtheta / dtSeconds;
  }

  public static ChoreoTrajectory cropTrajectoryStart(ChoreoTrajectory trajectory, Double startTime) {
    return cropTrajectory(trajectory, startTime, null);
  }

  public static ChoreoTrajectory cropTrajectoryEnd(ChoreoTrajectory trajectory, Double endTime) {
    return cropTrajectory(trajectory, null, endTime);
  }

  public static ChoreoTrajectory cropTrajectory(ChoreoTrajectory trajectory, Double startTime, Double endTime) {
    Double timestampOffset = null;
    List<ChoreoTrajectoryState> states = new ArrayList<>();
    for (ChoreoTrajectoryState state : trajectory.getSamples()) {
      boolean check;
      if (startTime != null && endTime != null) {
        check = state.timestamp >= startTime && state.timestamp <= endTime;
      } else if (startTime != null) {
        check = state.timestamp >= startTime;
      } else if (endTime != null) {
        check = state.timestamp <= endTime;
      } else {
        check = true;
      }
      if (check) {
        if (timestampOffset == null) timestampOffset = state.timestamp;
        ChoreoTrajectoryState newState = new ChoreoTrajectoryState(
          state.timestamp - timestampOffset,
          state.x,
          state.y,
          state.heading,
          state.velocityX,
          state.velocityY,
          state.angularVelocity
        );
        states.add(newState);
      }
    }
    return new ChoreoTrajectory(states);
  }

  /**
   * Sleeps for the specified number of milliseconds, without throwing an exception.
   * Use this with caution, as it can block the entire robot code.
   * @param millis The number of milliseconds to sleep.
   */
  private static void sleep(int millis) {
    try {
      Thread.sleep(millis);
    } catch (Exception e) {}
  }
}

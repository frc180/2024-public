package frc.robot.utils;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Utility class for creating and manipulating Commands. Wraps some functionality of
 * {@link edu.wpi.first.wpilibj2.command.Commands}.
 */
public class CommandsUtil {

  /**
   * Creates a new command that runs multiple commands sequentially. If only one Command is given, then it
   * will be returned unchanged (and not wrapped in another command).
   * @param commands The commands to include in the group.
   * @return The command group.
   */
  public static Command sequence(List<Command> commands) {
    return sequence(commands.stream().toArray(Command[]::new));
  }

  /**
   * Creates a new command that runs multiple commands sequentially. If only one Command is given, then it
   * will be returned unchanged (and not wrapped in another command).
   * @param commands The commands to include in the group.
   * @return The command group.
   */
  public static Command sequence(Command... commands) {
    if (commands.length == 1) return commands[0];

    return Commands.sequence(commands);
  }

  public static Command parallel(List<Command> commands) {
    return Commands.parallel(commands.stream().toArray(Command[]::new));
  }

  public static Command choreoCommandWithRotation(
      ChoreoTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Function<Double, Double> rotationOverride,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      BooleanSupplier mirrorTrajectory,
      Subsystem... requirements) {
    var timer = new Timer();
    var controller = choreoSwerveController(xController, yController, rotationController);
    return new FunctionalCommand(
        timer::restart,
        () -> {
          ;
          outputChassisSpeeds.accept(
              controller.apply(
                  poseSupplier.get(),
                  trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean()),
                  rotationOverride.apply(timer.get())));
        },
        (interrupted) -> {
          timer.stop();
          if (interrupted) {
            outputChassisSpeeds.accept(new ChassisSpeeds());
          } else {
            outputChassisSpeeds.accept(trajectory.getFinalState().getChassisSpeeds());
          }
        },
        () -> timer.hasElapsed(trajectory.getTotalTime()),
        requirements);
  }

  public static TriFunction<Pose2d, ChoreoTrajectoryState, Double, ChassisSpeeds> choreoSwerveController(
      PIDController xController, PIDController yController, PIDController rotationController) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    return (pose, referenceState, rotationOverride) -> {
      double angularVelocity = referenceState.angularVelocity;
      double heading = referenceState.heading;

      if (rotationOverride != null) {
        angularVelocity = 0;
        heading = rotationOverride;
      }

      double xFF = referenceState.velocityX;
      double yFF = referenceState.velocityY;
      double rotationFF = angularVelocity;

      double xFeedback = xController.calculate(pose.getX(), referenceState.x);
      double yFeedback = yController.calculate(pose.getY(), referenceState.y);
      double rotationFeedback =
          rotationController.calculate(pose.getRotation().getRadians(), heading);

      if (rotationOverride != null) {
        rotationFeedback = Helpers.cap(rotationFeedback, DrivetrainSubsystem.MAX_ANGULAR_RATE);
      }

      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
    };
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ChoreoConductor;
import static frc.robot.utils.AutoHelper.*;

/**
 * Original source-side auto ran at our first three regionals.
 * Has been replaced by SourceV2Auto for Champs.
 */
@Deprecated
public class SourceAuto extends SequentialCommandGroup {

  private static final boolean useVision = true;

  public SourceAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IntakeSubsystem intake, IntakePivotSubsystem intakePivot, int... notes) {
    ChoreoConductor conductor = new ChoreoConductor(false)
                                  .toggleVisionOdometry(useVision);

    addCommands(
      Commands.deadline(
        Commands.sequence(
            drivetrain.followChoreoPath("Bottom_UpToShoot", true),
            shootIfNote(),
            vision.allowVisionOdometry(false),
            conductor.scoreNotes(notes)
        ),
        new RunPivot(shooterPivot),
        new RunShooter(shooter, 4300),
        vision.directTagAiming(!useVision),
        vision.allowVisionOdometry(useVision),
        Commands.run(shooter::setShooterThresholdDistanceBased)
      )
    );
  }
}

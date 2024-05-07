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

public class AmpAuto extends SequentialCommandGroup {

  public AmpAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IntakeSubsystem intake, IntakePivotSubsystem intakePivot, boolean useVision, int... notes) {
    useVision = true;
    ChoreoConductor conductor = new ChoreoConductor(true)
                                  .applyDistanceOverrides(true)
                                  .toggleVisionOdometry(useVision);

    addCommands(
      Commands.deadline(
        Commands.sequence(
            vision.distanceOverride(4.746),
            drivetrain.followChoreoPath("Amp_UpToShoot", true),
            vision.clearDistanceOverride(),
            shootIfNote(),
            vision.allowVisionOdometry(false),
            drivetrain.followChoreoPath("Amp_AfterShoot", false),
            conductor.scoreNotes(notes)
        ),
        new RunPivot(shooterPivot),
        new RunShooter(shooter, 4300),
        vision.directTagAiming(true),
        vision.allowVisionOdometry(useVision),
        Commands.run(shooter::setShooterThresholdDistanceBased)
      )
    );
  }
}

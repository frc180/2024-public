package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ChoreoConductor;
import static frc.robot.utils.AutoHelper.*;

/**
 * Original four note auto (i.e. preload + wing notes + centerline) ran at our first three regionals.
 * Has been replaced by FourNoteOTF auto for Champs.
 */
@Deprecated
public class FourNoteAuto extends SequentialCommandGroup {

  public FourNoteAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IntakeSubsystem intake, IntakePivotSubsystem intakePivot, boolean useVision, int... notes) {
    useVision = true;
    ChoreoConductor conductor = new ChoreoConductor(true)
                                .toggleVisionOdometry(useVision);
    
    addCommands(
      Commands.deadline(
        Commands.sequence(
          Commands.deadline(
            drivetrain.followChoreoPath("FourNote.1", true),
            // Index preloaded note that may not be properly aligned in the intake
            new IntakeCommand(intake, true).withEndOnNoteDetected(true)
          ),
          shootIfNote(),
          intakeMove("FourNote.2", false),
          shootIfNote(),
          intakeMove("FourNote.3", false),
          shootIfNote(),
          intakeMove("FourNote.4", false),
          shootIfNote(),
          vision.allowVisionOdometry(false),
          drivetrain.followChoreoPath("FourNoteConductor", false),
          conductor.scoreNotes(notes)
        ),
        new RunPivot(shooterPivot),
        new RunShooter(shooter, Constants.SHOOTER_IDLE),
        vision.directTagAiming(!useVision),
        vision.allowVisionOdometry(useVision),
        Commands.run(shooter::setShooterThresholdDistanceBased)
      )
    );
  }
}

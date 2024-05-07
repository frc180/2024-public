package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ChoreoConductor;
import static frc.robot.utils.AutoHelper.*;

public class AmpRaceAuto extends SequentialCommandGroup {

  public AmpRaceAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IntakeSubsystem intake, IntakePivotSubsystem intakePivot, boolean cleanupWing, int... notes) {
    boolean shootOnTheFlyFar = false;
    ChoreoConductor conductor = new ChoreoConductor(true)
                                .toggleVisionOdometry(false)
                                .noteBypass(true)
                                .shootOnTheFly(shootOnTheFlyFar);

    if (cleanupWing) {
      conductor.noteBypassOverride(notes[1], false);
    }

    Command afterChoreoCommand;
    if (cleanupWing) {
      afterChoreoCommand = Commands.sequence(
          intakeMoveAimBackIntake("Amp_Wing.1", false, 0.3, null, null),
          shootIfNote(),
          intakeMoveAim("Amp_Wing.2", false, 0.4),
          shootIfNote(),
          drivetrain.followChoreoPath("Amp_WingAfter", false)
      );
    } else {
      afterChoreoCommand = drivetrain.followChoreoPath("Amp_After", false);
    }
    
    addCommands(
      Commands.deadline(
        Commands.sequence(
          Commands.deadline(
            drivetrain.followChoreoPath("AmpRace", true, speakerHeading(0.15, 2.55)),
            Commands.sequence(
              Commands.waitSeconds(0.2),
              shootWhenAligned(0.6),
              intakeSequence(),
              shootWhenAligned(0.65)
            )
          ),
          Commands.parallel(
            conductor.scoreNotes(notes),
            shootOnTheFlyFar ? Commands.none() : Commands.runOnce(() -> { Robot.TAG_HEIGHT_AIM = true; }),
            vision.shootOnTheFly(shootOnTheFlyFar)
            
          ),
          afterChoreoCommand
        ),
        new RunPivot(shooterPivot),
        new RunShooter(shooter, Constants.SHOOTER_IDLE),
        Commands.run(shooter::setShooterThresholdDistanceBased),
        vision.directTagAiming(false),
        vision.allowVisionOdometry(true),
        vision.shootOnTheFly(true),
        vision.megatag2Enabled(false)
      )
    );
  }
}
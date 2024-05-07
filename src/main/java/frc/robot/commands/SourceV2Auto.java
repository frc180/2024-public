package frc.robot.commands;

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

public class SourceV2Auto extends SequentialCommandGroup {

  public SourceV2Auto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IntakeSubsystem intake, IntakePivotSubsystem intakePivot, boolean fastEndings, int... notes) {
    boolean shootOnTheFlyFar = false;
    boolean noteBypass = true;
    ChoreoConductor conductor = new ChoreoConductor(false)
                                  .toggleVisionOdometry(false)
                                  .noteBypass(noteBypass)
                                  .fastEndings(fastEndings)
                                  .shootOnTheFly(shootOnTheFlyFar);

    var note1Heading = speakerHeading(0.32, 1.44);
    addCommands(
      Commands.deadline(
        Commands.sequence(
          Commands.deadline(
            drivetrain.followChoreoPath("Bottom_1_OnTheFly", true, note1Heading),
            shootWhenAligned(1.4)
          ),
          Commands.parallel(
            (shootOnTheFlyFar ? 
              intakeMoveShoot("Bottom_1_OnTheFly2", false, 0.3, null, 0, noteBypass ? false : null) 
            :
              intakeMoveAim("Bottom_1_OnTheFly2", false, 0.5, null, noteBypass ? false : null)
            ),
            vision.shootOnTheFly(shootOnTheFlyFar),
            shootOnTheFlyFar ? Commands.none() : Commands.runOnce(() -> { Robot.TAG_HEIGHT_AIM = true; })
          ),
          shootOnTheFlyFar ? conductor.scoreNotes(notes) : shootIfNote().andThen(conductor.scoreNotes(notes)),
          fastEndings ? Commands.none() : drivetrain.followChoreoPath("Bottom_After", false)
        ),
        new RunPivot(shooterPivot),
        new RunShooter(shooter, Constants.SHOOTER_IDLE),
        vision.directTagAiming(false),
        vision.allowVisionOdometry(true),
        vision.shootOnTheFly(true),
        Commands.run(shooter::setShooterThresholdDistanceBased)
        
      )
    );
  }
}

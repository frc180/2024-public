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
import static frc.robot.utils.AutoHelper.*;

public class Middle3Auto extends SequentialCommandGroup {

  public Middle3Auto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IntakeSubsystem intake, IntakePivotSubsystem intakePivot, boolean middle3) {    
    
    Command middle3Sequence;
    if (middle3) {
      middle3Sequence = Commands.sequence(
        intakeMoveAim("Middle_3", false, 0.5, null),
        shootIfNote()
      ).alongWith(Commands.runOnce(() -> { Robot.TAG_HEIGHT_AIM = true; }));
    } else {
      middle3Sequence = Commands.none();
    }

    addCommands(
      Commands.deadline(
        Commands.sequence(
          Commands.parallel(
            drivetrain.followChoreoPath("FourNote_OTF.1", true, speakerHeading(0.4)),
            Commands.sequence(
              Commands.waitSeconds(0.4),
              shootWhenAligned(0.95 - 0.4),
              Commands.parallel(
                intakeSequence().withTimeout(2.5),
                vision.shootOnTheFly(false)
              )
            )
          ),
          shootIfNote(),
          intakeMoveAim("FourNote_OTF.2", false, 0.5, null),
          shootIfNote(),
          middle3Sequence
        ),
        new RunPivot(shooterPivot),
        new RunShooter(shooter, Constants.SHOOTER_IDLE),
        Commands.run(shooter::setShooterThresholdDistanceBased),
        vision.directTagAiming(false),
        vision.allowVisionOdometry(true),
        vision.shootOnTheFly(true)
      )
    );
  }
}

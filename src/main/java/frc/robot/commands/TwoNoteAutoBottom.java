package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.utils.AutoHelper.*;

public class TwoNoteAutoBottom extends SequentialCommandGroup {

  public TwoNoteAutoBottom(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IntakeSubsystem intake, IntakePivotSubsystem intakePivot, int... notes) {
    addCommands(
      Commands.deadline(
        Commands.sequence(
          drivetrain.followChoreoPath("TwoNoteBottom.1", true),
          shootIfNote(),
          intakeMove("TwoNoteBottom.2"),
          shootIfNote()
        ),
        new ReadyShooter(shooter, shooterPivot),
        vision.directTagAiming(true),
        vision.allowVisionOdometry(false)
      )
    );
  }
}

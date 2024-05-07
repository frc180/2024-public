package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RunPivot extends Command {

  ShooterPivotSubsystem shooterPivot;
  Double targetAngle = null;

  public RunPivot(ShooterPivotSubsystem shooterPivotSubsystem) {
    shooterPivot = shooterPivotSubsystem;
    addRequirements(shooterPivotSubsystem);
  }

  public RunPivot(ShooterPivotSubsystem shooterPivotSubsystem, double targetAngle) {
    shooterPivot = shooterPivotSubsystem;
    this.targetAngle = targetAngle;
    addRequirements(shooterPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetAngle != null) {
      shooterPivot.setPosition(targetAngle);
    } else {
      shooterPivot.setPositionDistanceBased();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterPivot.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

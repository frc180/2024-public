package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReadyShooter extends Command {

  ShooterSubsystem shooter;
  ShooterPivotSubsystem shooterPivot;
  Double targetRPM = null;
  Double targetAngle = null;
  boolean stopOnEnd = true;

  public ReadyShooter(ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem shooterPivotSubsystem) {
    shooter = shooterSubsystem;
    shooterPivot = shooterPivotSubsystem;
    addRequirements(shooterSubsystem, shooterPivotSubsystem);
  }

  public ReadyShooter(ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, double targetRPM, double targetAngle) {
    shooter = shooterSubsystem;
    shooterPivot = shooterPivotSubsystem;
    this.targetRPM = targetRPM;
    this.targetAngle = targetAngle;
    addRequirements(shooterSubsystem, shooterPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Enable these to override the given values and use SmartDashboard values instead
    // targetRPM = SmartDashboard.getNumber("Shooter Tuning Speed", 0);
    // targetAngle = SmartDashboard.getNumber("Shooter Tuning Angle", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetRPM != null) {
      shooter.runShooter(targetRPM);
    } else {
      shooter.runShooterDistanceBased();
    }

    if (targetAngle != null) {
      shooterPivot.setPosition(targetAngle);
    } else {
      shooterPivot.setPositionDistanceBased();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopOnEnd) {
      shooter.stopShooter();
      shooterPivot.stopPivot();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

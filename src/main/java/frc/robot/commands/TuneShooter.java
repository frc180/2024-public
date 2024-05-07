package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TuneShooter extends Command {

  ShooterSubsystem shooter;
  ShooterPivotSubsystem shooterPivot;
  Double targetRPM = null;
  Double targetAngle = null;
  boolean stopOnEnd = true;

  public TuneShooter(ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem shooterPivotSubsystem) {
    shooter = shooterSubsystem;
    shooterPivot = shooterPivotSubsystem;
    addRequirements(shooterSubsystem, shooterPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetRPM = SmartDashboard.getNumber("Shooter Tuning Speed", 0);
    targetAngle = SmartDashboard.getNumber("Shooter Tuning Angle", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runShooter(targetRPM);
    shooterPivot.setPosition(targetAngle);
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

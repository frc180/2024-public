package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {

  ShooterSubsystem shooter;
  Double targetRPM = null;

  public RunShooter(ShooterSubsystem shooterSubsystem) {
    shooter = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  public RunShooter(ShooterSubsystem shooterSubsystem, double targetRPM) {
    shooter = shooterSubsystem;
    this.targetRPM = targetRPM;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetRPM != null) {
      shooter.runShooter(targetRPM);
    } else {
      shooter.runShooterDistanceBased();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

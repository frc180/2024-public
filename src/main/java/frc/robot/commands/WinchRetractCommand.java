package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.WinchSubsystem;

public class WinchRetractCommand extends SequentialCommandGroup {

  public WinchRetractCommand(WinchSubsystem winch, double position) {
    addCommands(
      winch.servoLatch(),
      Commands.runOnce(() -> winch.setWinchPosition(position), winch)
    );
  }
}

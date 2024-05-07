package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.WinchSubsystem;

public class WinchExtendCommand extends SequentialCommandGroup {

  public WinchExtendCommand(WinchSubsystem winch, double position) {   
    addCommands(
      winch.servoUnlatch(),
      Commands.runOnce(() -> winch.setWinchPosition(position), winch)
    );
  }
}

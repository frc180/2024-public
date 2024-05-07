package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.WinchSubsystem;

public class WinchCommand_Base extends SequentialCommandGroup {

  public WinchCommand_Base(WinchSubsystem winch, double target, boolean latch, boolean speedMode) {
    addCommands(
      winch.setServoLatchSafe(latch),
      speedMode ? winch.setWinchSpeed(target) : Commands.run(() -> winch.setWinchPosition(target), winch)
    );
  }

  public Command getLatchSequence(WinchSubsystem winch, boolean latch) {
    Command c =  latch ? winch.servoLatch() : winch.servoUnlatch();
    c = c.andThen(Commands.waitSeconds(0.2));
    return Commands.either(c, Commands.none(), ()-> winch.isLatched != latch);
  }
}

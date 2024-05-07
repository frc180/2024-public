package frc.robot.commands;

import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class WinchCommand extends Command {
    XboxController xboxController;
    WinchSubsystem winch;
    private double winchTarget;

    public WinchCommand(WinchSubsystem winchSubsystem, double position) {
        winch = winchSubsystem;
        winchTarget = position;
        addRequirements(winchSubsystem);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        winch.setWinchPosition(winchTarget);
    }

    @Override
    public void end(boolean interrupted) {}
}

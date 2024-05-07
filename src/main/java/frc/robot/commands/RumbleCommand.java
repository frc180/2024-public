package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RumbleCommand extends Command {

    XboxController controller;
    double rumbleStrength;

    public RumbleCommand(double rumbleStrength) {
        this.rumbleStrength = rumbleStrength;
    }

    @Override
    public void initialize() {
        controller = RobotContainer.instance.driverController.getHID();
    }

    @Override 
    public void execute() {
        controller.setRumble(RumbleType.kBothRumble, rumbleStrength);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

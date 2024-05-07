package frc.robot.commands;

import frc.robot.subsystems.AmpSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpPose extends Command {

    private AmpSubsystem m_ampSubsystem;
    private double pose;
    private boolean slowMode = false;
    private boolean runOnce = false;

    public AmpPose(AmpSubsystem ampSubsystem, double pose) {
        this.m_ampSubsystem = ampSubsystem;
        this.pose = pose;
        this.slowMode = pose == AmpSubsystem.STOW;
        addRequirements(ampSubsystem);
    }

    public AmpPose withRunOnce(boolean runOnce) {
        this.runOnce = runOnce;
        return this;
    }

    @Override
    public void initialize() {
        m_ampSubsystem.setAmpPose(pose, slowMode);
    }

    @Override
    public void execute() {
        m_ampSubsystem.setAmpPose(pose, slowMode);
    }

    @Override
    public boolean isFinished() {
        return runOnce;
    }

    @Override
    public void end(boolean interrupted) {}
}

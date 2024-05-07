package frc.robot.commands;

import frc.robot.subsystems.IntakePivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePivotCommand extends Command {
    
    private final IntakePivotSubsystem m_pivotSubsystem;
    private double pivotPose;
    private boolean isShooterFeed = false;
    private boolean runOnce = false;

    public IntakePivotCommand(IntakePivotSubsystem intakePivotSubsystem, double pose) {
        m_pivotSubsystem = intakePivotSubsystem;
        pivotPose = pose;
        isShooterFeed = pose == IntakePivotSubsystem.SHOOTER_FEED;
        addRequirements(intakePivotSubsystem);
    }

    public IntakePivotCommand withRunOnce(boolean runOnce) {
        this.runOnce = runOnce;
        return this;
    }

    @Override
    public void initialize() {
        setPose();
    }

    @Override
    public void execute() {
        setPose();  
    }

    @Override
    public boolean isFinished() {
        return runOnce;
    }

    @Override
    public void end(boolean interrupted) {
    }

    private void setPose() {     
        m_pivotSubsystem.setPose(pivotPose);   
    }
}

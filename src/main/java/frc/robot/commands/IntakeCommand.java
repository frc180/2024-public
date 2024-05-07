package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    public double intakeSpeed;
    public IntakeSubsystem intakeSub;
    public final boolean front;
    public boolean endOnBeamBreak = false;
    public boolean endOnNoteDetected = false;
    public boolean stopOnEnd = true;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean front, double speed) {
        this.intakeSub = intakeSubsystem;
        this.front = front;
        intakeSpeed = speed;
        addRequirements(intakeSubsystem);
    }

    public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean front) {
        this.intakeSub = intakeSubsystem;
        this.front = front;
        intakeSpeed = 1;
        addRequirements(intakeSubsystem);
    }

    public IntakeCommand withEndOnBeamBreak(boolean endOnBeamBreak) {
        this.endOnBeamBreak = endOnBeamBreak;
        return this;
    }

    public IntakeCommand withEndOnNoteDetected(boolean endOnNoteDetected) {
        this.endOnNoteDetected = endOnNoteDetected;
        return this;
    }

    public IntakeCommand withStopOnEnd(boolean stopOnEnd) {
        this.stopOnEnd = stopOnEnd;
        return this;
    }

    public boolean firstBreak(){
        return !front ? intakeSub.backMidBreak() : intakeSub.frontMidBreak();
    }

    public boolean secondBreak(){
        return !front ? intakeSub.frontMidBreak() : intakeSub.backMidBreak();
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        double dir = front ? 1 : -1;
        double rumble = 0;
        boolean runOuterIntake = false;
        if (intakeSub.isNoteDetected()) {
            // If the note is detected, stop the intake
            intakeSub.intakeSetSpeed(0);
            rumble = 1;
        } else if (firstBreak() && !secondBreak()) {
            // Slow down as the note approaches both beam breaks
            intakeSub.intakeSetSpeed(0.2 * dir);
            runOuterIntake = true;
        } else if (!firstBreak() && secondBreak()) {
            // Run backwards if we overshot the first beam break
            intakeSub.intakeSetSpeed(-0.1 * dir); 
        }  else {
            intakeSub.intakeSetSpeed(intakeSpeed * dir);
            runOuterIntake = true;
        }
        if (runOuterIntake) {
            if (front) intakeSub.setSweeper(0.8);
            if (!front) intakeSub.setBackIntakeSpeed(1);
        } else {
            intakeSub.setSweeper(0);
            intakeSub.setBackIntakeSpeed(0);
        }
        RobotContainer.instance.driverController.getHID().setRumble(RumbleType.kBothRumble, rumble);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.instance.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        if (stopOnEnd) intakeSub.stop();
    }

    @Override
    public boolean isFinished() {
        if (endOnBeamBreak) return intakeSub.frontMidBreak();
        if (endOnNoteDetected) return intakeSub.isNoteDetected();

        return false;
    }
}

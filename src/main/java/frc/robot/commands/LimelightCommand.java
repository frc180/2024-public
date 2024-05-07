package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * A command used to turn the robot to face the Speaker using the Limelight.
 * @deprecated use {@link frc.robot.commands.TurnToSpeakerCommand} instead
 */
@Deprecated
public class LimelightCommand extends Command {
  
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    public LimelightCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;
        addRequirements(drivetrainSubsystem);
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Double speakerTurn = m_visionSubsystem.getSpeakerTurning();
        if (speakerTurn == null) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        } else {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, speakerTurn));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }
}

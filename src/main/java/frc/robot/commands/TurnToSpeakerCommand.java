package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;

public class TurnToSpeakerCommand extends Command {
  
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private PIDController altPID;
    private boolean useAltPID = false;

    private double rotationSpeed = 0;
    private HeadingTarget prevHeadingTarget = HeadingTarget.POSE;

    private ChassisSpeeds speeds;

    public TurnToSpeakerCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        drivetrain = drivetrainSubsystem;
        vision = visionSubsystem;
        altPID = new PIDController(10, 0, 0);
        altPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrainSubsystem);

    }

    @Override
    public void initialize() {
        speeds = new ChassisSpeeds(0, 0, 0);
        if (useAltPID) {
            altPID.reset();
        } else {
            drivetrain.resetHeadingPID(prevHeadingTarget);
        }
    }

    @Override
    public void execute() {
        HeadingTarget headingTarget = vision.getSpeakerHeadingType();
        Rotation2d heading = headingTarget == HeadingTarget.POSE ? drivetrain.getPose().getRotation() : drivetrain.getGyroscopeRotation();
        if (headingTarget != prevHeadingTarget) {
            if (useAltPID) {
                // Reset shouldn't be necessary for altPID since it's not a profiled controller
                // altPID.reset();
            } else {
                drivetrain.resetHeadingPID(heading);
            }
        }
        if (useAltPID) {
            rotationSpeed = altPID.calculate(heading.getRadians(), vision.getSpeakerAngle());
        } else {
            rotationSpeed = drivetrain.calculateHeadingPID(heading, vision.getSpeakerAngle());
        }
        speeds.omegaRadiansPerSecond = rotationSpeed;
        drivetrain.drive(speeds);
        prevHeadingTarget = headingTarget;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;

public class DriveToPose extends Command {
  
    private final DrivetrainSubsystem drivetrain;

    private Supplier<Pose2d> targetPoseSupplier = null;
    private Pose2d targetPose = null;

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        drivetrain = drivetrainSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d targetPose) {
        drivetrain = drivetrainSubsystem;
        this.targetPose = targetPose;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        if (targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }
        drivetrain.resetPIDs(HeadingTarget.POSE);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drivetrain.calculateChassisSpeeds(drivetrain.getPose(), targetPose);
        drivetrain.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}

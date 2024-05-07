package frc.robot.utils;

import java.util.HashMap;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.TurnToSpeakerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;

/**
 * Helper class that contains static method for common command compositions used in Autonomous mode.
 */
public class AutoHelper {

    public final static double NOTE_X = 7.921848297119141 - 0.15;
    public final static double NOTE_Y_BASE = 7.472607612609863;
    public final static double NOTE_Y_DIFF = NOTE_Y_BASE - 5.788680553436279;

    public static double BYPASS_STOP_AMP = 0;
    public static double BYPASS_STOP_SOURCE = 9999;

    static DrivetrainSubsystem drivetrain;
    static IntakeSubsystem intake;
    static IntakePivotSubsystem intakePivot;
    static ShooterSubsystem shooter;
    static ShooterPivotSubsystem shooterPivot;
    static VisionSubsystem vision;

    private static HashMap<String, Double> trajectoryReturnTimes = null;
    static final PIDController altHeadingPID = new PIDController(8, 0., 0.);


    public static void loadSubsystems(RobotContainer container) {
        drivetrain = container.drivetrain;
        intake = container.intake;
        intakePivot = container.intakePivot;
        shooter = container.shooter;
        shooterPivot = container.shooterPivot;
        vision = container.vision;

        altHeadingPID.enableContinuousInput(-Math.PI, Math.PI);

        trajectoryReturnTimes = new HashMap<>();
        trajectoryReturnTimes.put("Amp_5_Start", 1.2);
        trajectoryReturnTimes.put("Amp_5", 1.65);
        trajectoryReturnTimes.put("Amp_4_Start", 1.5);
        trajectoryReturnTimes.put("Amp_4", 1.61);
        trajectoryReturnTimes.put("Amp_3", 1.96);
        trajectoryReturnTimes.put("Amp_2", 2.16);
        trajectoryReturnTimes.put("Amp_1", 2.66);
        trajectoryReturnTimes.put("Bottom_1", 2.12);
        trajectoryReturnTimes.put("Bottom_2", 2.08);
        trajectoryReturnTimes.put("Bottom_3", 2.06);
        trajectoryReturnTimes.put("Bottom_3_End", 2.06);
        trajectoryReturnTimes.put("Bottom_4", 2.16);
    }

    public static Command shootIfNote() {
        // Disable skipping shoot command for now, in case note is almost fully loaded but not quite hitting both sensors
        // return Commands.either(shoot(), Commands.none(), intake::isNoteDetected);
        return shoot();
    }

    /**
     * Shoots a note. Will aim at the Speaker and shoot when ready.
     * @return
     */
    public static Command shoot() {        
        return Commands.deadline(
            Commands.sequence(
                new TurnToSpeakerCommand(drivetrain, vision).until(allAligned()),
                intake.feedShooterCommand()
            ), 
            new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED)
        );
    }

    /**
     * Shoots a note regardless of alignment. Will not aim at the Speaker.
     */
    public static Command shootNoAim() {
        BooleanSupplier mechanismsAligned = RobotContainer.instance.shootMechanismsAligned;
        if (Robot.isSimulation()) mechanismsAligned = () -> true;
        
        return Commands.deadline(
            Commands.waitUntil(mechanismsAligned).andThen(intake.feedShooterCommand()),
            new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED)
        );
    }

    /**
     * Shoots a note when the robot is aligned to the Speaker.
     */
    public static Command shootWhenAligned(Double timeoutSeconds) {
        Command waitForAligned;
        if (timeoutSeconds != null) {
            waitForAligned = Commands.race(
                Commands.waitUntil(allAligned()),
                Commands.waitSeconds(timeoutSeconds)
            );
        } else {
            waitForAligned = Commands.waitUntil(allAligned());
        }
        
        return Commands.deadline(
            waitForAligned.andThen(intake.feedShooterCommand()),
            new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED)
        );
    }

    public static Command intakeSequence() {
        return intakeSequence(true);
    }

    public static Command intakeSequence(boolean front) {
        if (Robot.isSimulation()) {
            return intake.runOnce(()-> {
                return;
            });
        }

        return Commands.sequence(
            new IntakePivotCommand(intakePivot, front ? IntakePivotSubsystem.FRONT : IntakePivotSubsystem.BACK).withRunOnce(true),
            new IntakeCommand(intake, front, 0.8).withEndOnNoteDetected(true),
            new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED).withRunOnce(true)  
        );
    }

    public static Command waitForIntake(double timeoutSeconds) {
        return Commands.race(
            Commands.waitUntil(intake.noteDetected()),
            Commands.waitSeconds(timeoutSeconds)
        );
    }

    public static Command intakeMove(String pathName) { 
        return intakeMove(pathName, false);
    }

    public static Command intakeMove(String pathName, boolean resetPosition) { 
        return intakeMove(pathName, resetPosition, null);
    }

    public static Command intakeMove(String pathName, boolean resetPosition, Command afterDrive) {  
        return intakeMove(pathName, resetPosition, afterDrive, null);
    }

    public static Command intakeMove(String pathName, boolean resetPosition, Command afterDrive, Function<Double, Double> heading) {        
        return intakeMove(pathName, resetPosition, afterDrive, heading, Robot.isReal() ? 2.5 : 0.25);
    }

    public static Command intakeMove(String pathName, boolean resetPosition, Command afterDrive, Function<Double, Double> heading, double intakeWaitSeconds) {        
        return intakeMove(pathName, resetPosition, afterDrive, heading, intakeWaitSeconds, null);
    }

    public static Command intakeMove(String pathName, boolean resetPosition, Command afterDrive, Function<Double, Double> heading, double intakeWaitSeconds, Boolean adjustableAmpSide) {        
        Command driveSequence;
        if (adjustableAmpSide == null) {
            driveSequence = buildDriveSequence(Choreo.getTrajectory(pathName), resetPosition, afterDrive, heading, intakeWaitSeconds);
        } else {
            driveSequence = buildAdjustableDriveSequence(pathName, resetPosition, afterDrive, heading, intakeWaitSeconds, adjustableAmpSide);
        }
        return Commands.deadline(
            driveSequence,
            intakeSequence()
        );    
    }

    public static Command intakeMoveAim(String pathName, boolean resetPosition, double aimSecondsBeforeEnd) {
        return intakeMoveAim(pathName, resetPosition, aimSecondsBeforeEnd, null);
    }

    public static Command intakeMoveAim(String pathName, boolean resetPosition, double aimSecondsBeforeEnd, Command afterDrive) {
        return intakeMoveAim(pathName, resetPosition, aimSecondsBeforeEnd, afterDrive, null);
    }

    public static Command intakeMoveAim(String pathName, boolean resetPosition, double aimSecondsBeforeEnd, Command afterDrive, Boolean adjustableAmpSide) {
        ChoreoTrajectory path = Choreo.getTrajectory(pathName);
        Function<Double, Double> aimHeading = speakerHeading(path.getTotalTime() - aimSecondsBeforeEnd);

        Command driveSequence;
        if (adjustableAmpSide == null) {
            driveSequence = buildDriveSequence(path, resetPosition, afterDrive, aimHeading, Robot.isReal() ? 2.5 : 0.25);
        } else {
            driveSequence = buildAdjustableDriveSequence(pathName, resetPosition, afterDrive, aimHeading, Robot.isReal() ? 2.5 : 0.25, adjustableAmpSide);
        }

        return Commands.deadline(
            driveSequence,
            intakeSequence()
        );
    }

    public static Command intakeMoveAimBackIntake(String pathName, boolean resetPosition, double aimSecondsBeforeEnd, Command afterDrive, Boolean adjustableAmpSide) {
        ChoreoTrajectory path = Choreo.getTrajectory(pathName);
        Function<Double, Double> aimHeading = speakerHeading(path.getTotalTime() - aimSecondsBeforeEnd);

        Command driveSequence;
        if (adjustableAmpSide == null) {
            driveSequence = buildDriveSequence(path, resetPosition, afterDrive, aimHeading, Robot.isReal() ? 2.5 : 0.25);
        } else {
            driveSequence = buildAdjustableDriveSequence(pathName, resetPosition, afterDrive, aimHeading, Robot.isReal() ? 2.5 : 0.25, adjustableAmpSide);
        }

        return Commands.deadline(
            driveSequence,
            intakeSequence(false)
        );
    }

    public static Command intakeMoveShoot(String pathName, boolean resetPosition, double aimSecondsBeforeEnd, Command afterDrive) {
        return intakeMoveShoot(pathName, resetPosition, aimSecondsBeforeEnd, afterDrive, Robot.isReal() ? 2.5 : 0.25);
    }

    public static Command intakeMoveShoot(String pathName, boolean resetPosition, double aimSecondsBeforeEnd, Command afterDrive, double intakeWaitSeconds) {
        return intakeMoveShoot(pathName, resetPosition, aimSecondsBeforeEnd, afterDrive, intakeWaitSeconds, null);
    }

    public static Command intakeMoveShoot(String pathName, boolean resetPosition, double aimSecondsBeforeEnd, Command afterDrive, double intakeWaitSeconds, Boolean adjustableAmpSide) {
        ChoreoTrajectory path = Choreo.getTrajectory(pathName);
        double aimStart = path.getTotalTime() - aimSecondsBeforeEnd;
        Function<Double, Double> aimHeading = speakerHeading(aimStart);

        Command driveSequence;
        if (adjustableAmpSide == null) {
            driveSequence = buildDriveSequence(path, resetPosition, afterDrive, aimHeading, intakeWaitSeconds);
        } else {
            driveSequence = buildAdjustableDriveSequence(pathName, resetPosition, afterDrive, aimHeading, intakeWaitSeconds, adjustableAmpSide);
        }
        
        return Commands.parallel(
                driveSequence.andThen(
                        new TurnToSpeakerCommand(drivetrain, vision).until(allAligned())//,
                ),
            Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(aimStart),
                    intakeSequence()
                ),
                shootWhenAligned(null)
            )
        );
    }

    public static BooleanSupplier allAligned() {
        if (Robot.isSimulation()) return drivetrain::isAlignedToSpeaker;
        return RobotContainer.instance.shootAllAligned;
    }

    public static Function<Double, Double> speakerHeading(double startTime) {
        return (time) -> {
            if (time < startTime) return null;
            return Math.toRadians(vision.getSpeakerAngle());
        };
    }

    public static Function<Double, Double> speakerHeading(double startTime, double endTime) {
        return (time) -> {
            if (time < startTime || time > endTime) return null;
            return Math.toRadians(vision.getSpeakerAngle());
        };
    }

    private static Command buildDriveSequence(ChoreoTrajectory path, boolean resetPosition, Command afterDrive, Function<Double, Double> heading, double intakeWaitSeconds) {
        return applyIntakeAndAfterDrive(drivetrain.followChoreoPath(path, resetPosition, heading), afterDrive, intakeWaitSeconds);
    }

    public static Command buildAdjustableDriveSequence(String pathName, boolean resetPosition, Command afterDrive, Function<Double, Double> heading, double intakeWaitSeconds, boolean ampSide) {
        ChoreoTrajectory basePath = Choreo.getTrajectory(pathName);
        Double timeSplit = trajectoryReturnTimes.get(pathName);
        if (timeSplit == null) timeSplit = 1.9;
        ChoreoTrajectory notePath = Helpers.cropTrajectoryEnd(basePath, timeSplit);
        ChoreoTrajectory returnPath = Helpers.cropTrajectoryStart(basePath, timeSplit);

        Command driveSequence = Commands.sequence(
            drivetrain.followChoreoPath(notePath, resetPosition, heading),
            Commands.either(
                drivetrain.followChoreoPath(returnPath, resetPosition, heading),
                findCenterlineNote(ampSide),
                intake::isNotePartialDetected
            )
        );

        return applyIntakeAndAfterDrive(driveSequence, afterDrive, intakeWaitSeconds);
    }

    private static Command applyIntakeAndAfterDrive(Command driveSequence, Command afterDrive, double intakeWaitSeconds) {
        Command intakeWait = intakeWaitSeconds > 0 ? waitForIntake(intakeWaitSeconds) : Commands.none();
        Command finishedSequence;
        if (afterDrive == null) {
            finishedSequence = driveSequence.andThen(intakeWait);
        } else {
            finishedSequence = driveSequence.andThen(Commands.parallel(afterDrive, intakeWait));
        }
        return finishedSequence;
    }

    public static Command findCenterlineNote(boolean ampSide) {
        final String pathPrefix = ampSide ? "Amp_" : "Bottom_";
        return Commands.sequence(
            driveDownCenterline(ampSide),
            new DeferredCommand(() -> {

                int closestNote = getClosestNote(drivetrain.getPose());
                RobotContainer.autoNotesScored.add(closestNote);
                
                String pathName = pathPrefix + closestNote;
                try {
                    ChoreoTrajectory path = Choreo.getTrajectory(pathName);
                    Double startTime = trajectoryReturnTimes.get(pathName);
                    if (startTime == null) startTime = 1.9;
                    startTime -= 0.2;
                    return drivetrain.followChoreoPath(Helpers.cropTrajectoryStart(path, startTime), false, null);
                } catch (NullPointerException e) {
                    return drivetrain.runOnce(() -> {
                        drivetrain.drive(new ChassisSpeeds());
                    }).alongWith(Commands.waitSeconds(15));
                }
            }, Set.of(drivetrain))
        );
    }

    public static Command driveDownCenterline(boolean ampSide) {
        Command drive = drivetrain.run(() -> {
            double sign = (ampSide ? 1 : -1) * (Robot.isBlue() ? 1 : -1);
            Pose2d pose = drivetrain.getPose();
        
            double rotationSpeed = drivetrain.calculateHeadingPID(pose.getRotation(), ampSide ? -90 : 90);
            double xSpeed = 2.5 * ((Constants.FIELD_WIDTH_METERS / 2.0) - pose.getX());

            int closestNote = getClosestNote(pose);
            if (!RobotContainer.autoNotesScored.contains(closestNote)) {
                RobotContainer.autoNotesScored.add(closestNote);
            }

            drivetrain.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * (Robot.isBlue() ? 1 : -1), 
              DrivetrainSubsystem.MAX_SPEED * -0.3 * sign, 
              rotationSpeed, 
              drivetrain.getGyroscopeRotation())
            );
        });
        drive = drive.beforeStarting(Commands.runOnce(() -> drivetrain.resetHeadingPID(HeadingTarget.POSE)));

        if (Robot.isSimulation()) return drive.withTimeout(1.5);
        return drive.until(() -> {
            if (intake.isNotePartialDetected()) return true;


            return false;
        });
    }

    private static int getClosestNote(Pose2d pose) {
        int closestNote = -1;
        double closestDistance = 999990;
        for (int i = 0; i < 5; i++) {
            double y = NOTE_Y_BASE - (i * NOTE_Y_DIFF);
            double distance = Math.abs(pose.getY() - y);
            if (distance < closestDistance) {
                closestNote = 5 - i;
                closestDistance = distance;
            }
        }
        return closestNote;
    }
}

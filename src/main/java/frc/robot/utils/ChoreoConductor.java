package frc.robot.utils;

import static frc.robot.utils.AutoHelper.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Generates centerline paths for the robot to follow during autonomous.
 */
public class ChoreoConductor {

    private static double AMP_NOTE_DISTANCE = 4.31;
    private static double SOURCE_NOTE_DISTANCE = 4.55 + .7 + .4;

    private boolean ampSide;
    private boolean toggleVisionOdometry = true;
    private boolean applyDistanceOverrides = false;
    private boolean shootOnTheFly = false;
    private boolean noteBypass = false;
    private boolean fastEndings = true;

    private HashMap<Integer, Boolean> noteBypassOverrides = new HashMap<>();

    public ChoreoConductor(boolean ampSide) {
        this.ampSide = ampSide;
    }

    public ChoreoConductor toggleVisionOdometry(boolean toggle) {
        toggleVisionOdometry = toggle;
        return this;
    }

    public ChoreoConductor applyDistanceOverrides(boolean apply) {
        applyDistanceOverrides = apply;
        return this;
    }

    public ChoreoConductor shootOnTheFly(boolean shootOnTheFlyEnabled) {
        shootOnTheFly = shootOnTheFlyEnabled;
        return this;
    }

    public ChoreoConductor noteBypass(boolean noteBypassEnabled) {
        noteBypass = noteBypassEnabled;
        return this;
    }

    public ChoreoConductor fastEndings(boolean fastEndingsEnabled) {
        fastEndings = fastEndingsEnabled;
        return this;
    }

    public ChoreoConductor noteBypassOverride(int note, boolean bypass) {
        noteBypassOverrides.put(note, bypass);
        return this;
    }

    public Command scoreNotes(int... notes) {
        VisionSubsystem vision = RobotContainer.instance.vision;
        List<Command> commands = new ArrayList<>();
        if (shootOnTheFly) commands.add(vision.shootOnTheFly(true));
        for (int i = 0; i < notes.length; i++) {
            List<Command> noteCommands = new ArrayList<>();
            int note = notes[i];
            String pathName;
            boolean partial = false;
            Function<Double, Double> headingOverride = null;
            double aimSecondsBefore = 0.3;

            if (i == 0 && ampSide) {
                pathName = "Amp_" + note + "_Start";
            } else {
                pathName = (ampSide ? "Amp_" : "Bottom_") + note;
            }
            // Special cases
            if (ampSide && note == 5 && i == 0) {
                headingOverride = speakerHeading(1.67);
            }
            if (ampSide && note == 4 && i != 0) {
                headingOverride = speakerHeading(2.3);
            }
            if (ampSide && note == 3) {
                headingOverride = speakerHeading(2.66);
            }
            if (!ampSide) {
                aimSecondsBefore = 0.3;
            }
            if (!ampSide && note == 2) {
                headingOverride = speakerHeading(3);
            }
            if (!ampSide && note == 3 && i == notes.length - 1 && fastEndings) {
                pathName = "Bottom_3_End";
                headingOverride = speakerHeading(2.5);
                aimSecondsBefore = 0.4;
            }
                
            if (applyDistanceOverrides) noteCommands.add(distanceOverride(vision));

            List<Command> afterDriveCommands = new ArrayList<>();
            afterDriveCommands.add(Commands.runOnce(() -> RobotContainer.autoNotesScored.add(note)));
            if (toggleVisionOdometry) afterDriveCommands.add(vision.allowVisionOdometry(true));
            if (applyDistanceOverrides) afterDriveCommands.add(vision.clearDistanceOverride());

            Command afterDrive = afterDriveCommands.isEmpty() ? null : CommandsUtil.parallel(afterDriveCommands);
            boolean doBypass = noteBypass;
            Boolean bypassOverride = noteBypassOverrides.get(note);
            if (bypassOverride != null) {
                doBypass = bypassOverride;
            }

            if (shootOnTheFly) {
                noteCommands.add(
                    intakeMoveShoot(
                        pathName,
                        false,
                        aimSecondsBefore,
                        afterDrive,
                        0,
                        doBypass ? ampSide : null
                    )
                );
            } else {
                noteCommands.add(
                    intakeMove(
                        pathName, 
                        false,
                        afterDrive,
                        headingOverride,
                        0,
                        doBypass ? ampSide : null
                    )
                );
            }

            if (!partial) {
                if (!shootOnTheFly) noteCommands.add(shootIfNote());
                if (toggleVisionOdometry) noteCommands.add(vision.allowVisionOdometry(false));
            }

            commands.add(
                Commands.either(
                    Commands.none(),
                    CommandsUtil.sequence(noteCommands),
                    () -> RobotContainer.hasScored(note)
                )
            );
        }
        return CommandsUtil.sequence(commands);
    }

    private Command distanceOverride(VisionSubsystem vision) {
        double distance = ampSide ? AMP_NOTE_DISTANCE : SOURCE_NOTE_DISTANCE;
        return vision.distanceOverride(distance);
    }
}
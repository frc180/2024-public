package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.Telemetry;
import frc.robot.subsystems.VisionSubsystem.LimelightPose;
import frc.robot.utils.CommandsUtil;
import frc.robot.utils.Helpers;
import frc.robot.utils.InterpolatorTables;
import frc.robot.utils.PIDTuner;
import monologue.Logged;
import monologue.Annotations.Log;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem, Logged {

    public enum HeadingTarget {
        GYRO,
        POSE
    }

    public static double boundsTolerance = 1.0;

    public static final double MAX_SPEED = 4.6; // Meters per second desired top speed
    public static final double MAX_SPEED_ACCEL = 6; // Meters per second squared max acceleration
    public static final double MAX_ANGULAR_RATE = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static final double MAX_ANGULAR_ACCEL = MAX_ANGULAR_RATE * 8;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.ApplyChassisSpeeds autoClosedLoopRequest = new SwerveRequest.ApplyChassisSpeeds()
                                                                                .withDriveRequestType(DriveRequestType.Velocity);

    private final double kPathFollowDriveP;
    private final double kPathFollowTurnP;

    private ProfiledPIDController xPidController, yPidController, driverRotationPidController;
    private PIDController choreoX, choreoY, choreoRotation;
    private PIDController driverRotationPidAlt;
    private PIDTuner rotationTuner = null;
    private Double m_targetHeading = null;
    private HeadingTarget m_targetHeadingType = HeadingTarget.GYRO;
    private Orchestra orchestra = null;
    private Rotation2d gyroOffset = new Rotation2d();
    @Log
    public boolean alignedToSpeaker = false;
    @Log
    public boolean alignedToSpeakerDebounced = false;
    public Debouncer alignedToSpeakerDebouncer = new Debouncer(0.1);
    private double headingError = 0;
    private double speakerError = 0;
    private Matrix<N3, N1> measureStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.3, 0.3, 0.3);
    
    private final Telemetry logger = new Telemetry(MAX_SPEED);

    // Variables just for simulation
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    public DrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        kPathFollowDriveP = Robot.isReal() ? 5 : 10;
        kPathFollowTurnP = Robot.isReal() ? 3 : 8;
        initialize();
    }

    public void initialize() {
        xPidController = new ProfiledPIDController(2.5, 0., 0,
                                        new TrapezoidProfile.Constraints(MAX_SPEED, 6));
        yPidController = new ProfiledPIDController(2.5, 0., 0,
                                        new TrapezoidProfile.Constraints(MAX_SPEED, 6));

        driverRotationPidController = new ProfiledPIDController(10, 0., 0,
                                        new TrapezoidProfile.Constraints(MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL));
        driverRotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        driverRotationPidAlt = new PIDController(12, 0., 0.);
        driverRotationPidAlt.enableContinuousInput(-Math.PI, Math.PI);

        choreoX = new PIDController(kPathFollowDriveP, 0, 0);
        choreoY = new PIDController(kPathFollowDriveP, 0, 0);
        choreoRotation = new PIDController(kPathFollowTurnP, 0, 0);

        // rotationTuner = new PIDTuner(driverRotationPidController, ()-> m_targetHeading == null ? -1 : m_targetHeading)
        //                   .withName("Drive Rotation")
        //                   .withValue(() -> {
        //                         if (m_targetHeadingType == HeadingTarget.GYRO) {
        //                             return getGyroscopeDegrees();
        //                         } else {
        //                             return getPose().getRotation().getDegrees();
        //                         }
        //                     });
        // rotationTuner.initializeValues(driverRotationPidController);

        orchestra = new Orchestra();
        AudioConfigs audio = new AudioConfigs().withAllowMusicDurDisable(true);
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        for (int i = 0; i < Modules.length; i++) {
            SwerveModule m = Modules[i];
            TalonFXConfigurator driveConfig = m.getDriveMotor().getConfigurator();
            TalonFXConfigurator steerConfig = m.getSteerMotor().getConfigurator();
            driveConfig.refresh(limits);
            limits.SupplyCurrentLimit = 60;
            limits.SupplyCurrentLimitEnable = true;
            driveConfig.apply(limits);
            driveConfig.apply(audio);
            steerConfig.apply(audio);
            orchestra.addInstrument(m.getDriveMotor());
            orchestra.addInstrument(m.getSteerMotor());
        }

        configurePathplanner();

        if (Utils.isSimulation()) {
            startSimThread();
            seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        registerTelemetry(logger::telemeterize);
        
    }

    public void drive(ChassisSpeeds speeds) {
        Helpers.discretizeOverwrite(speeds, Constants.LOOP_TIME);
        setControl(autoRequest.withSpeeds(speeds));
    }

    public void driveClosedLoop(ChassisSpeeds speeds) {
        setControl(autoClosedLoopRequest.withSpeeds(speeds));
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    @Override
    public void periodic() {
        Pose2d pose = getPose();
        double gyroDegrees = getGyroscopeDegrees();
        Rotation2d poseRotation = pose.getRotation();

        VisionSubsystem vision = RobotContainer.instance.vision;
        double headingDegrees = vision.getSpeakerHeadingType() == HeadingTarget.GYRO ? gyroDegrees : poseRotation.getDegrees();
        speakerError = vision.getSpeakerAngle() - MathUtil.inputModulus(headingDegrees, -180, 180);
        log("Speaker Error", speakerError);

        double acceptableSpeakerError = InterpolatorTables.getSpeakerTolerance(vision.getSpeakerDistance());
        log("Speaker Tolerance", acceptableSpeakerError);
        alignedToSpeaker = alignedToSpeakerDebouncer.calculate(Math.abs(speakerError) <= acceptableSpeakerError);
        alignedToSpeakerDebounced = alignedToSpeaker;
    
        log("Aligned to Speaker", alignedToSpeaker);
        log("Gyro Angle", gyroDegrees);
        log("Pose Angle", poseRotation.getDegrees());
        
        if (Robot.isAutonomousMode()) {
            Pose2d choreoTarget = new Pose2d(choreoX.getSetpoint(), choreoY.getSetpoint(), Rotation2d.fromRadians(choreoRotation.getSetpoint()));
            log("Choreo Pose Target", choreoTarget);
            log("Choreo X Error", choreoX.getPositionError());
            log("Choreo Y Error", choreoY.getPositionError());
            log("Choreo Rotation Error", Units.radiansToDegrees(choreoRotation.getPositionError()));
        }

        if (rotationTuner != null) rotationTuner.periodic();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeDegrees());
    }

    public double getGyroscopeDegrees() {
        return (-m_pigeon2.getAngle()) - gyroOffset.getDegrees();
    }

    public void zeroGyroscope() {
        gyroOffset = m_pigeon2.getRotation2d();
        seedFieldRelative();
    }

    public boolean pigeonConnected() {
        return true;
    }

    public void addVisionMeasurement(LimelightPose limelightPose) {
        addVisionMeasurement(limelightPose.pose, limelightPose.timestamp, measureStdDev);
    }

    public void resetPIDs(HeadingTarget type) {
        xPidController.reset(getPose().getX());
        yPidController.reset(getPose().getY());
        resetHeadingPID(type);
    }

    public void resetHeadingPID(HeadingTarget type) {
        resetHeadingPID(type == HeadingTarget.GYRO ? getGyroscopeRotation() : getPose().getRotation());
    }

    public void resetHeadingPID(Rotation2d rotation) {
        driverRotationPidController.reset(rotation.getRadians());
    }

    public double calculateHeadingPID(Rotation2d heading, double targetDegrees) {
        double headingDegrees = heading.getDegrees();
        headingError = targetDegrees - headingDegrees;
        
        return driverRotationPidController.calculate(
            heading.getRadians(), 
            Math.toRadians(targetDegrees)
        );
    }

    public double calculateHeadingPID(double headingDegrees, double targetDegrees) {
        headingError = targetDegrees - headingDegrees;
    
        return driverRotationPidController.calculate(
            Math.toRadians(headingDegrees), 
            Math.toRadians(targetDegrees)
        );
    }

    public double calculateXPID(double currentX, double targetX) {
        log("X Error", (targetX - currentX));
        double result = xPidController.calculate(
            currentX,
            targetX
        );
        log("X PID Result", result);
        return result;
    }

    public ChassisSpeeds calculateChassisSpeeds(Pose2d currentPose, Pose2d targetPose) {
        double xFeedback = xPidController.calculate(currentPose.getX(), targetPose.getX());
        double yFeedback = yPidController.calculate(currentPose.getY(), targetPose.getY());
        double thetaFeedback = calculateHeadingPID(currentPose.getRotation(), targetPose.getRotation().getDegrees());

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, thetaFeedback, currentPose.getRotation());
    }

    public boolean isHeadingAligned() {
        return Math.abs(headingError) <= 4;
    }

    public boolean isAlignedToSpeaker() {
        return alignedToSpeaker;
    }

    public double getSpeakerError() {
        return speakerError;
    }

    public Double getTargetHeading() {
        return m_targetHeading;
    }

    public HeadingTarget getTargetHeadingType() {
        return m_targetHeadingType;
    }
    
    public void setTargetHeading(Double targetHeading) {
        setTargetHeading(targetHeading, HeadingTarget.GYRO);
    }

    public void setTargetHeading(Double targetHeading, HeadingTarget type) {
        m_targetHeading = targetHeading == null ? null : MathUtil.inputModulus(targetHeading, -180, 180);
        m_targetHeadingType = type;
        if (targetHeading == null) {
            headingError = 0;
        } else {
            headingError = 999; // reset heading error to make sure we don't think we're at the new target immediately
        }
    }
    
    public Command targetHeading(Double heading, HeadingTarget type) {
        return Commands.runOnce(() -> setTargetHeading(heading, type));
    }
    
    public Command targetHeadingContinuous(Double heading, HeadingTarget type) {
        return Commands.run(() -> setTargetHeading(heading, type));
    }
    
    public Command targetHeadingContinuous(Supplier<Double> headingSupplier, HeadingTarget type) {
        return Commands.run(() -> setTargetHeading(headingSupplier.get(), type));
    }

    public void setMeasureStdDev(double x, double y, double angle) {
        measureStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), x, y, angle);
    }

    public Command measurementStdDev(double x, double y, double angle) {
        return Commands.runOnce(() -> setMeasureStdDev(x, y, angle));
    }

    public void sing(String song) {
        orchestra.loadMusic(song + ".chrp");
        orchestra.play();
    }

    public Command singCommand(String song) {
        return runEnd(
            ()-> {
                if (!orchestra.isPlaying()) sing(song); 
            },
            orchestra::stop
        ).until(DriverStation::isEnabled).ignoringDisable(true);
    }

    public Command waitForTranslation(Translation2d translation) {
        return waitForTranslation(translation, 0.1);
    }

    public Command waitForTranslation(Translation2d translation, double distanceMeters) {
        final Translation2d flippedTranslation = new Translation2d(Constants.FIELD_WIDTH_METERS - translation.getX(), translation.getY());
        return Commands.waitUntil(() -> {
            Translation2d target = Robot.isRed() ? flippedTranslation : translation;
            
            log("waitForTranslation Target", target);
            return target.getDistance(getPose().getTranslation()) <= distanceMeters;
        });
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     * @param pathName The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return followChoreoPath(pathName, resetPosition, null);
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     * @param pathName The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition, Function<Double, Double> rotationOverride) {
        return followChoreoPath(Choreo.getTrajectory(pathName), resetPosition, rotationOverride);
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     * @param trajectory The Choreo trajectory to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition, Function<Double, Double> rotationOverride) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> {
                seedFieldRelative(Robot.isRed() ? trajectory.getFlippedInitialPose() : trajectory.getInitialPose());
            }));
        }
        commands.add(rotationOverride != null ? choreoRotationCommand(trajectory, rotationOverride) : choreoSwerveCommand(trajectory));
        return CommandsUtil.sequence(commands);
    }

    // This is a helper method that creates a command that makes the robot follow a Choreo path
    private Command choreoSwerveCommand(ChoreoTrajectory trajectory) {
        return Choreo.choreoSwerveCommand(
            trajectory,
            this::getPose,
            choreoX,
            choreoY,
            choreoRotation,
            this::drive,
            Robot::isRed,
            this
        );
    }

    private Command choreoRotationCommand(ChoreoTrajectory trajectory, Function<Double, Double> rotationOverride) {
        return CommandsUtil.choreoCommandWithRotation(
            trajectory,
            this::getPose,
            choreoX,
            choreoY,
            choreoRotation,
            rotationOverride,
            this::drive,
            Robot::isRed,
            this
        );
    }

    /**
     * Returns a command that makes the robot follow a PathPlanner path using the PathPlannerLib library.
     * @param path The PathPlannerPath to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path.
     */
    public Command followPath(PathPlannerPath path, boolean resetPosition) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> {
                seedFieldRelative(path.getStartingDifferentialPose());
            }));
        }

        commands.add(AutoBuilder.followPath(path));
        return CommandsUtil.sequence(commands);
    }

    /**
     * Creates a PathPlanner path that follows the given poses, and finishes with the given end rotation.
     * @param endRotation The rotation (i.e. heading) the robot should be facing at the end of the path
     * @param poses The poses that comprise the path.
     * @return A PathPlannerPath object that can be followed using {@link #followPath(PathPlannerPath, boolean) followPath}.
     */
    public PathPlannerPath createPath(Rotation2d endRotation, Pose2d... poses) {
        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

        // Create the path using the bezier points created above
        return new PathPlannerPath(
                bezierPoints,
                new PathConstraints(MAX_SPEED * 0.9, MAX_SPEED_ACCEL, MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, endRotation) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void configurePathplanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(3 * 0.25, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
                        MAX_SPEED, // Max module speed, in m/s
                        Constants.DRIVETRAIN_TRACKWIDTH_METERS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                Robot::isRed, // Boolean supplier that controls when the path will be mirrored for the red alliance
                this // Reference to this subsystem to set requirements
        );
    }
}

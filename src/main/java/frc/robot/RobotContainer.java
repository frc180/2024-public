package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.AmpPose;
import frc.robot.commands.AmpRaceAuto;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FourNoteOTFAuto;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FanSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.Middle3Auto;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.RunPivot;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SourceV2Auto;
import frc.robot.commands.SourceV2Auto2;
import frc.robot.commands.WinchCommand_Base;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.LedSubsystem.LED_PATTERN;
import frc.robot.utils.AutoHelper;
import frc.robot.utils.InterpolatorTables;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

  public DrivetrainSubsystem drivetrain = TunerConstants.DriveTrain;
  public VisionSubsystem vision = null;
  public ShooterSubsystem shooter = null;
  public ShooterPivotSubsystem shooterPivot = null;
  public IntakeSubsystem intake = null;
  public IntakePivotSubsystem intakePivot = null;
  public WinchSubsystem winch = null;
  public AmpSubsystem amp = null;
  public LedSubsystem led = null;
  public FanSubsystem fan = null;

  public PowerDistribution powerDistribution = null;

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandGenericHID testController = new CommandGenericHID(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  public final SendableChooser<DriverStation.Alliance> allianceChooser = new SendableChooser<DriverStation.Alliance>();
  public static RobotContainer instance = null;

  public Trigger shootAllAligned;
  public Trigger shootMechanismsAligned;
  private Double scoreHeldTime = null;

  public static List<Integer> autoNotesScored = new ArrayList<>();

  public RobotContainer() {
    vision = new VisionSubsystem();
    shooter = new ShooterSubsystem();
    shooterPivot = new ShooterPivotSubsystem();
    intake = new IntakeSubsystem();
    intakePivot = new IntakePivotSubsystem();
    winch = new WinchSubsystem();
    amp = new AmpSubsystem();
    led = new LedSubsystem();
    fan = new FanSubsystem();
    
    powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    shootAllAligned = new Trigger(() -> {
      return drivetrain.isAlignedToSpeaker() && shooter.isAtTargetSpeed() && shooterPivot.isAligned() && intakePivot.isAligned();
    });

    shootMechanismsAligned = new Trigger(() -> {
      return shooter.isAtTargetSpeed() && shooterPivot.isAligned() && intakePivot.isAligned();
    });

    InterpolatorTables.initiateTables();
    AutoHelper.loadSubsystems(this);
    drivetrain.zeroGyroscope();
    configureBindings();
    Monologue.setupMonologue(this, "Robot", false, true);
    RobotContainer.instance = this;


    autoChooser.setDefaultOption("Do Nothing", Commands.print("Do Nothing Auto!"));

    autoChooser.addOption("Amp RACE: 2W -> 5, 4, 3, 2", new AmpRaceAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, false, 5, 4, 3, 2));
    autoChooser.addOption("Amp RACE: 2W -> 4, 5, 3, 2", new AmpRaceAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, false, 4, 5, 3, 2));
    autoChooser.addOption("Amp RACE: 2W -> 4, 3, 2", new AmpRaceAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, false, 4, 3, 2));
    autoChooser.addOption("Amp RACE: 2W -> 5, 4 -> 2W", new AmpRaceAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, true, 5, 4));
    autoChooser.addOption("Amp RACE: 2W -> 4, 5 -> 2W", new AmpRaceAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, true, 4, 5));

    autoChooser.addOption("Middle FAST: 4W -> 5, 4, 3, 2", new FourNoteOTFAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, 5, 4, 3, 2));
    autoChooser.addOption("Middle FAST: 4W -> 4, 3, 2", new FourNoteOTFAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, 4, 3, 2));
    autoChooser.addOption("Middle FAST: 3W -> 3", new Middle3Auto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, true));
    autoChooser.addOption("Middle FAST: 3W", new Middle3Auto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, false));

    autoChooser.addOption("Source FAST: 1W -> 1, 2, 3", new SourceV2Auto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, true, 2, 3));
    autoChooser.addOption("Source FAST: 1W -> 2, 3", new SourceV2Auto2(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, true, 3));
    autoChooser.addOption("Source FAST Integrity: 1W -> 1, 2, 3", new SourceV2Auto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, false, 2, 3));

    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putNumberArray("Custom Note Order", new double[] { 5, 4, 3 });

    allianceChooser.addOption("None", null);
    allianceChooser.addOption("Blue", Alliance.Blue);
    allianceChooser.addOption("Red", Alliance.Red);
    SmartDashboard.putData("Alliance", allianceChooser);

    SmartDashboard.putData("Flashlight On", setPDHChannel(true));
    SmartDashboard.putData("Flashlight Off", setPDHChannel(false));

    SmartDashboard.putNumber("BlueFeed", -45);
    SmartDashboard.putNumber("RedFeed", 25);

  }

  private void configureBindings() {
    driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope));

    final Trigger driverLeftTrigger = driverController.leftTrigger();
    final Trigger driverY = driverController.y();
    final Trigger scoreHeldTimeout = new Trigger(() -> {
      if (scoreHeldTime == null) return false;
      return Timer.getFPGATimestamp() - scoreHeldTime >= 2;
    });

    DoubleSupplier rotationSupplier = () -> {
      return -modifyAxis(driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_RATE;
    };

    DoubleSupplier xAxisSupplier = () -> {
      double speed = -modifyAxis(driverController.getLeftX()) * DrivetrainSubsystem.MAX_SPEED;
      if (driverLeftTrigger.getAsBoolean()) {
        speed *= 0.5;
      }
      return speed;
    };

    DoubleSupplier yAxisSupplier = () -> {
      double speed = -modifyAxis(driverController.getLeftY()) * DrivetrainSubsystem.MAX_SPEED;
      if (driverLeftTrigger.getAsBoolean()) {
        speed *= 0.5;
      }
      return speed;
    };

    drivetrain.setDefaultCommand(new DefaultDriveCommand(
      drivetrain,
      yAxisSupplier,
      xAxisSupplier,
      rotationSupplier
    ));

    if (amp != null)  {
      RobotModeTriggers.teleop().or(RobotModeTriggers.autonomous()).onTrue(
        Commands.parallel(
          new AmpPose(amp, AmpSubsystem.STOW).withRunOnce(true),
          winch.servoLatch()
        )
      );
    }

    if (shooterPivot != null) {
      // Always be aiming the shooter pivot
      shooterPivot.setDefaultCommand(new RunPivot(shooterPivot));
    }

    if (shooter != null) {
      // Run the shooter if we have a note and are on our half of the field
      shooter.setDefaultCommand(shooter.run(() -> {
        double speakerDistance = vision.getSpeakerDistance();
        boolean notClimbing = !winch.isClimbing && !driverY.getAsBoolean();
        if (intake.isNoteDetected() && speakerDistance <= 9 && notClimbing) {
          shooter.runShooter(Constants.SHOOTER_IDLE);
        } else {
          shooter.stopShooter();
        }
      }));

      // Speaker ready
      driverController.leftTrigger().whileTrue(
        Commands.parallel(
          // new TuneShooter(shooter, shooterPivot), // tuning mode
          new RunShooter(shooter, Constants.SHOOTER_IDLE), // Override in case the auto-rev is not working
          Commands.run(shooter::setShooterThresholdDistanceBased),
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED),
          Commands.run(() -> {
            double angle = vision.getSpeakerAngle();
            HeadingTarget headingType = vision.getSpeakerHeadingType();
            if (vision.getSpeakerDistance() > 8.3) {
              angle = Robot.isBlue() ? 0 : 180;
              headingType = HeadingTarget.POSE;
            }
            drivetrain.setTargetHeading(angle, headingType);
          })
        )
      ).onFalse(drivetrain.targetHeading(null, HeadingTarget.POSE));

      if (amp != null) {
        Command ampLoadWithSensor = Commands.sequence(
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.AMP_FEED).withRunOnce(true),
          new AmpPose(amp, AmpSubsystem.STOW).until(()-> amp.isAligned() && intakePivot.isAligned()),
          Commands.deadline(
            Commands.waitUntil(() -> amp.isNoteDetected()).andThen(Commands.waitSeconds(0.2)),
            Commands.parallel(
              intake.setIntakeSpeedCommand(-0.3),
              amp.setRollerSpeed(-0.30)
            )
          )      
        );
      
        // Amp ready with amp arm
        driverController.rightTrigger().whileTrue(
          Commands.parallel(
            new RunShooter(shooter, 0),
            drivetrain.targetHeadingContinuous(() -> {
              return Robot.isBlue() ? 90.0 : -90;
            }, HeadingTarget.GYRO),
            Commands.sequence(
              Commands.either(ampLoadWithSensor, Commands.none(), intake::isNoteDetected),
              Commands.parallel(
                intake.setIntakeSpeedCommand(-0.8),
                new AmpPose(amp, AmpSubsystem.UP)
              ).until(amp::isAligned)
            )
          )
        ).onFalse(drivetrain.targetHeading(null, HeadingTarget.POSE));

        // Automatically retract amp arm when we're away from where we scored
        new Trigger(amp::awayFromScorePose).and(driverController.rightTrigger().negate()).onTrue(
          Commands.sequence(
            amp.clearScorePose(),
            new AmpPose(amp, AmpSubsystem.STOW).withRunOnce(true),
            new RumbleCommand(0.5).withTimeout(0.5)
          )
        );
      }

      // Feeder shot ready
      driverController.x().whileTrue(
        Commands.parallel(
          new ReadyShooter(shooter, shooterPivot, 3000, ShooterPivotSubsystem.FEED_ANGLE),
          Commands.run(() -> shooter.setShooterThreshold(null)),
          Commands.run(() -> shooter.setFeederRPM(true)),
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.FEED_SHOT),
          drivetrain.targetHeadingContinuous(() -> {
              
              return Robot.isBlue() ? -45.0 : 25;
          }, HeadingTarget.GYRO)
        )
      ).onFalse(Commands.runOnce(() -> shooter.setFeederRPM(false)));

      // Climb ready (extend winch)
      driverController.y().whileTrue(
        Commands.parallel(
          new WinchCommand_Base(winch, WinchSubsystem.UP, false, false)
                              .alongWith(Commands.runOnce(() -> winch.setClimbing(false))),
          drivetrain.targetHeadingContinuous(vision::getStageAngle, HeadingTarget.GYRO),
          led.ledPattern(LED_PATTERN.RAINBOW)
        )                  
      ).onFalse(
        Commands.parallel(
          drivetrain.targetHeading(null, HeadingTarget.GYRO),
          Commands.either(
            Commands.none(),
            Commands.sequence(
              winch.runWinchToZero(-0.2, false),
              winch.servoLatch()
            ),
            winch::isClimbing
          )
        )
      );

      // Trap shot ready
      driverController.a().whileTrue(
        Commands.parallel(
          new ReadyShooter(shooter, shooterPivot, 2200, ShooterPivotSubsystem.TRAP_ANGLE),
          Commands.run(() -> shooter.setShooterThreshold(null)),
          Commands.run(() -> shooter.setFeederRPM(true)),
          vision.megatag2Enabled(false),
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.TRAP_FEED),
          new DriveToPose(drivetrain, vision::getCurrentTrapPose),
          fan.setSpeed(1)
        )
      ).onFalse(
        Commands.parallel(
          drivetrain.targetHeading(null, HeadingTarget.GYRO),
          Commands.runOnce(() -> shooter.setFeederRPM(false))
        )
      );


      driverController.b()
                      .onTrue(Commands.runOnce(() -> {
                        scoreHeldTime = Timer.getFPGATimestamp();
                      }))
                      .onFalse(Commands.runOnce(() -> {
                        scoreHeldTime = null;
                      }));

      // Speaker score
      driverController.b()
                      .and(driverController.leftTrigger().or(scoreHeldTimeout))
                      .and(shootAllAligned)
                      .onTrue(intake.feedShooterCommand());

      // Amp score
      if (amp != null) {
        driverController.b()
                        .and(driverController.rightTrigger())                  
                        .onTrue(
                            Commands.sequence(
                              new AmpPose(amp, AmpSubsystem.UP).until(amp::isAligned),
                              amp.setRollerSpeed(1)
                                  .withTimeout(0.5)
                                  .deadlineWith(new RumbleCommand(0.25))
                                  .andThen(amp.saveScorePose())
                            )
                        );
      }

      // Climb "score"
      driverController.b()
                      .and(driverController.y())
                      .whileTrue(
                        Commands.parallel(
                          Commands.sequence(
                            winch.runWinchToZero(-0.6, true),
                            new RumbleCommand(0.5).withTimeout(0.5)
                          ),
                          Commands.runOnce(() -> winch.setClimbing(true))
                        )
                      );
      
      // Feeder shot score
      driverController.b()
                      .and(driverController.x())
                      .and(shootMechanismsAligned)
                      .onTrue(intake.feedShooterCommand());
      
      // Trap shot score
      driverController.b()
                      .and(driverController.a())
                      .and(shootMechanismsAligned)
                      .onTrue(intake.feedShooterCommand());

      
    }

    if (intake != null) {
      // Intake from front (amp side) of robot
      driverController.leftBumper()
        .whileTrue(Commands.parallel(
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.FRONT),
          new IntakeCommand(intake, true, 1),
          new RunShooter(shooter, Constants.SHOOTER_IDLE_LOW)
        ))
        .onFalse(
            Commands.either(
              new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED),
              Commands.none(),
              () -> !intake.isNoteHalfDetected()
            )
        );
      
      // Intake from back (shooter side) of robot
      driverController.rightBumper()
        .whileTrue(
          Commands.sequence(
            new IntakePivotCommand(intakePivot, IntakePivotSubsystem.BACK).until(intakePivot::isAligned),
            new IntakeCommand(intake, false, 1)
          ).alongWith(new RunShooter(shooter, Constants.SHOOTER_IDLE_LOW))
        )
        .onFalse(
            Commands.either(
              new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED),
              Commands.none(),
              () -> !intake.isNoteHalfDetected()
            )
        );

      // Automatically retract intake if we have a note
      driverController.leftBumper().or(driverController.rightBumper()).and(intake.noteDetected())
        .onTrue(
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED)
            .alongWith(new RumbleCommand(1).withTimeout(0.5))
        );

      // Outtake from front (amp side) of robot
      driverController.leftBumper().and(driverController.b())
        .whileTrue(Commands.sequence(
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.FRONT).until(intakePivot::isAligned),
          Commands.parallel(
            intake.setIntakeSpeedCommand(-1),
            intake.setSweeperSpeedCommandNoRequirements(-0.8)
          )
        ))
        .onFalse(
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED)
        );
 
      // Outtake from back (shooter side) of robot
      driverController.rightBumper().and(driverController.b())
        .whileTrue(Commands.sequence(
          Commands.parallel(
            intake.setBackIntakeSpeedCommandNoRequirements(-1),
            new IntakePivotCommand(intakePivot, IntakePivotSubsystem.FRONT)
          )
        ))
        .onFalse(
          new IntakePivotCommand(intakePivot, IntakePivotSubsystem.SHOOTER_FEED)
        );
    }

    led.setDefaultCommand(led.run(() -> {
      LED_PATTERN pattern;
      if (!vision.isLimelightConnected()) {
        pattern = LED_PATTERN.RAINBOW;
      } else if (vision.canSeeSpeaker() && intake.isNoteDetected()){
        pattern = LED_PATTERN.HAS_TARGET;
      } else if (intake.isNoteDetected()){
        pattern = LED_PATTERN.HAS_NOTE;
      } else if (intake.frontMidBreak() || intake.backMidBreak()){
        pattern = LED_PATTERN.INTAKING_NOTE;
      } else {
        pattern = LED_PATTERN.NO_NOTE;
      }
      led.setLedPattern(pattern);
    }));

  }

  public void periodic() {    
    boolean aligned = drivetrain.isAlignedToSpeaker() && shooter.isAtTargetSpeed() && shooterPivot.isAligned() && intakePivot.isAligned();

    log("Controller Connected?", driverController.getHID().isConnected());
    log("Tag Height Aim?", Robot.TAG_HEIGHT_AIM);
    log("Shoot All Aligned?", aligned);
    log("Browned Out?", RobotController.isBrownedOut());
    log("Battery Voltage", RobotController.getBatteryVoltage());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command setPDHChannel(boolean enabled) {
    return Commands.runOnce(() -> powerDistribution.setSwitchableChannel(enabled)).ignoringDisable(true);
  }

  public int[] getCustomNoteOrder() {
    double[] notes = SmartDashboard.getNumberArray("Custom Note Order", new double[0]);
    int[] notesInt = new int[notes.length];
    for (int i = 0; i < notes.length; i++) {
      notesInt[i] = (int) notes[i];
    }
    return notesInt;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }

    if (value > 0.0) {
      return (value - deadband) / (1.0 - deadband);
    } else {
      return (value + deadband) / (1.0 - deadband);
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.07);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static boolean hasScored(int note) {
    return autoNotesScored.contains(note);
  }
}

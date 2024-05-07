package frc.robot;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeSubsystem;

public class Robot extends TimedRobot {
  public static final boolean DEFAULT_TAG_HEIGHT_AIM = false;
  public static boolean TAG_HEIGHT_AIM = DEFAULT_TAG_HEIGHT_AIM;
  public static double autoStartTime = -1;
  private static boolean isAutonomous = false;

  private static boolean blue = true;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    // Query and cache the alliance
    addPeriodic(() -> {
      Alliance a = m_robotContainer.allianceChooser.getSelected();
      if (a != null) {
        blue = a == Alliance.Blue;
      } else {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        blue = alliance.isEmpty() || alliance.get() == Alliance.Blue;
      }
      m_robotContainer.log("Blue Alliance?", blue);
    }, 0.2);
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoStartTime = Timer.getFPGATimestamp();
    Robot.TAG_HEIGHT_AIM = DEFAULT_TAG_HEIGHT_AIM;
    Robot.isAutonomous = true;
    RobotContainer.autoNotesScored.clear();
    IntakeSubsystem.shotNumber = 1;
    m_robotContainer.drivetrain.zeroGyroscope();
    m_robotContainer.drivetrain.setMeasureStdDev(0.3, 0.3, 0.3);
    m_robotContainer.vision.setDistanceOverride(null);
    m_robotContainer.vision.setAllowVisionOdometry(false);
    m_robotContainer.vision.setDirectTagAiming(true);
    m_robotContainer.vision.setShootOnTheFly(false);
    m_robotContainer.vision.setMegatag2Enabled(false);
    m_robotContainer.vision.setAutoTagFilter();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Robot.TAG_HEIGHT_AIM = DEFAULT_TAG_HEIGHT_AIM;
    Robot.isAutonomous = false;
    m_robotContainer.drivetrain.setMeasureStdDev(0.1, 0.1, 0.1);
    m_robotContainer.vision.setDistanceOverride(null);
    m_robotContainer.vision.setAllowVisionOdometry(true);
    m_robotContainer.vision.setDirectTagAiming(false); 
    m_robotContainer.vision.setShootOnTheFly(true);
    m_robotContainer.vision.setMegatag2Enabled(false);
    m_robotContainer.vision.setTeleopTagFilter();
  
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    IntakeSubsystem.shotNumber = 1;
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  public static boolean isBlue() {
    return blue;
  }

  public static boolean isRed() {
    return !blue;
  }

  public static boolean isAutonomousMode() {
    return isAutonomous;
  }
}

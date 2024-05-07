package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.utils.Helpers;
import frc.robot.utils.InterpolatorTables;
import frc.robot.utils.PIDTuner;
import monologue.Logged;

public class ShooterSubsystem extends SubsystemBase implements Logged {

  private CANSparkFlex m_shootMotorA;
  private CANSparkFlex m_shootMotorB;
  private SparkPIDController m_shootPID;
  private RelativeEncoder m_shootEncoder;
  private double shooterPIDTarget = -1;
  private Double shooterPIDThreshold = null;
  private boolean atTargetSpeedBool = false;
  private Trigger atTargetSpeed;
  private boolean feederRPM = false;

  private PIDTuner shooterTuner = null;
  
  public ShooterSubsystem() {
    m_shootMotorA = new CANSparkFlex(20, MotorType.kBrushless);
    m_shootMotorB = new CANSparkFlex(21, MotorType.kBrushless);

    m_shootMotorA.restoreFactoryDefaults();
    Helpers.setSparkInverted(m_shootMotorA, true);
    m_shootMotorA.setIdleMode(IdleMode.kCoast);

    m_shootMotorB.restoreFactoryDefaults();
    m_shootMotorB.setInverted(false);
    m_shootMotorB.setIdleMode(IdleMode.kCoast);

    m_shootPID = m_shootMotorA.getPIDController();
    m_shootPID.setP(0.001);
    m_shootPID.setI(0);
    m_shootPID.setD(0);
    m_shootPID.setFF(0.000205);
    m_shootPID.setOutputRange(0, 1);
    
    m_shootEncoder = m_shootMotorA.getEncoder();

    // shooterTuner = new PIDTuner(m_shootPID, ()-> shooterPIDTarget)
    //                   .withName("Shooter")
    //                   .withValue(m_shootEncoder::getVelocity)
    //                   .withOutput(m_shootMotorA::get);
    // shooterTuner.initializeValues(m_shootPID);

    atTargetSpeed = new Trigger(this::isAtTargetSpeed);
  
    SmartDashboard.putNumber("Shooter Tuning Speed", 3500);
    SmartDashboard.putNumber("Amp Speed", 0.18);

    m_shootMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    m_shootMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    
    m_shootMotorB.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_shootMotorB.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    
  }

  @Override
  public void periodic() {
    double velocity = m_shootEncoder.getVelocity();

    double thresholdTarget = shooterPIDThreshold != null ? shooterPIDThreshold : shooterPIDTarget;
    if (feederRPM) {
      atTargetSpeedBool = shooterPIDTarget > 0 && Math.abs(thresholdTarget - velocity) <= 125;
    } else {
      atTargetSpeedBool = shooterPIDTarget > 0 && 
                          (Math.abs(thresholdTarget - velocity) <= 25 || thresholdTarget < velocity);
    }

    log("Shooter RPM", velocity);
    log("Shooter RPM Threshold", thresholdTarget);
    log("Feeder RPM Mode", feederRPM);
    log("At Target Speed?", atTargetSpeedBool);
    log("Motors/ShooterA Temperature (C)", m_shootMotorA.getMotorTemperature());
    log("Motors/ShooterA Output", m_shootMotorA.getAppliedOutput());
    log("Motors/ShooterB Temperature (C)", m_shootMotorB.getMotorTemperature());
    log("Motors/ShooterB Output", m_shootMotorB.getAppliedOutput());

    
    if (shooterTuner != null) shooterTuner.periodic();
  }

  public void runShooterDistanceBased() {
    runShooter(InterpolatorTables.shooterRPMTable().get(RobotContainer.instance.vision.getSpeakerTableKey()));
  }

  /**
   * Run the shooter at the given RPM
   * @param rpm The RPM to run the shooter at
   */
  public void runShooter(double rpm) {
    if (rpm == 0) {
      stopShooter();
      shooterPIDTarget = rpm;
      log("Shooter RPM Target", shooterPIDTarget);
      return; 
    }

    m_shootPID.setReference(rpm, ControlType.kVelocity);
    // Because the Vortexes are on the same shaft and there's latency in the follower CAN frames,
    // both motors running can cause higher oscillation at lower power. So, if we are
    // running at a lower speed, we only want to run one motor.
    if (rpm >= 2000) {
      m_shootMotorB.follow(m_shootMotorA, true);
    } else {
      m_shootMotorB.follow(ExternalFollower.kFollowerDisabled, 0);
      m_shootMotorB.setVoltage(0);
    }
    shooterPIDTarget = rpm;
    log("Shooter RPM Target", shooterPIDTarget);
  }

  public void runShooterOpenLoop(double speed) {
    m_shootMotorA.setVoltage(speed * 12);
    shooterPIDTarget = -1;
  }

  public Command shooterOpenLoop(double speed) {
    return Commands.runEnd(
      ()-> runShooterOpenLoop(speed),
      ()-> stopShooter()
    );
  }

  public void setShooterThresholdDistanceBased() {
    setShooterThreshold(InterpolatorTables.shooterRPMTable().get(RobotContainer.instance.vision.getSpeakerTableKey()));
  }

  public void setShooterThreshold(Double threshold) {
    shooterPIDThreshold = threshold;
  }

  public void stopShooter(){
    m_shootMotorA.stopMotor();
    m_shootMotorB.follow(ExternalFollower.kFollowerDisabled, 0);
    m_shootMotorB.stopMotor();
    shooterPIDTarget = -1;
  }

  public void setFeederRPM(boolean isFeederRPM) {
    feederRPM = isFeederRPM;
  }

  public boolean isAtTargetSpeed() {
    return atTargetSpeedBool;
  }

  public Trigger atTargetSpeed() {
    return atTargetSpeed;
  }

  public Command setShootSpeedACommand(double speed) {
    return runEnd(
      ()-> {
        m_shootMotorA.set(speed);
      },
      () -> {
        m_shootMotorA.set(0);
      }
    );
  }

  public Command setShootSpeedBCommand(double speed) {
    return runEnd(
      ()-> {
        m_shootMotorB.set(speed);
      },
      () -> {
        m_shootMotorB.set(0);
      }
    );
  }
}

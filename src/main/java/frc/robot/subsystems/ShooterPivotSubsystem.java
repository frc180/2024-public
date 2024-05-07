package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.Helpers;
import frc.robot.utils.InterpolatorTables;
import frc.robot.utils.PIDTuner;
import monologue.Logged;

public class ShooterPivotSubsystem extends SubsystemBase implements Logged {

  public static final double FEED_ANGLE = 235; // was 227.5, 230, low shot 200;
  public static final double TRAP_ANGLE = 247;

  private final double pivotMin = 198.453;
  private final double pivotMax = 257.5;

  private CANSparkMax m_pivotMotor;
  private SparkPIDController m_pivotPID;
  private SparkAbsoluteEncoder m_pivotEncoder;
  private double pivotPIDTarget = -1;
  private double position = 0;

  private PIDTuner pivotTuner = null;
  
  public ShooterPivotSubsystem() {
    m_pivotMotor = new CANSparkMax(22, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    Helpers.setSparkInverted(m_pivotMotor, true);
    m_pivotMotor.setIdleMode(IdleMode.kBrake);

    m_pivotEncoder = m_pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_pivotEncoder.setPositionConversionFactor(360);
    m_pivotEncoder.setInverted(true);

    m_pivotPID = m_pivotMotor.getPIDController();
    m_pivotPID.setP(0.11);
    m_pivotPID.setI(0);
    m_pivotPID.setD(0);
    m_pivotPID.setFF(0);
    m_pivotPID.setFeedbackDevice(m_pivotEncoder);

    // pivotTuner = new PIDTuner(m_pivotPID, ()-> pivotPIDTarget)
    //                 .withName("Shooter Pivot")
    //                 .withValue(m_pivotEncoder::getPosition)
    //                 .withOutput(m_pivotMotor::get);
    // pivotTuner.initializeValues(m_pivotPID);
  
    SmartDashboard.putNumber("Shooter Tuning Angle", 255);

    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor position
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); // duty cycle position
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // duty cycle velocity
  }

  double error = 0;
  boolean isAligned = false;

  @Override
  public void periodic() {
    position = m_pivotEncoder.getPosition();
    error = pivotPIDTarget - position;
    isAligned = Math.abs(error) <= 0.3;

    log("Shooter Pivot Aligned", isAligned);
    log("Shooter Pivot Position", position);
    log("Shooter Pivot Error", error);
    log("Motors/Pivot Temperature (C)", m_pivotMotor.getMotorTemperature());
    log("Motors/Pivot Current (A)", m_pivotMotor.getOutputCurrent());

    if (pivotTuner != null) pivotTuner.periodic();
  }

  /**
   * Set the shooter position (pivot) to the angle that corresponds to the current distance from the target
   */
  public void setPositionDistanceBased() {
    setPosition(InterpolatorTables.pivotAngleTable().get(RobotContainer.instance.vision.getSpeakerTableKey()));
  }

  public double getCurrentPostion() {
    return position;
  }

  /**
   * Set the shooter position (pivot) to the given angle
   * @param angle The angle to set the pivot to
   */
  public void setPosition(double angle) {
    angle = boundPivot(angle);
    m_pivotPID.setReference(angle, ControlType.kPosition);
    pivotPIDTarget = angle;
    log("Shooter Pivot Target", pivotPIDTarget);
  }

  public boolean isAligned() {
    return isAligned;
  }

  public double boundPivot(double target) {
    if(target < pivotMin){
      return pivotMin;
    }
    if(target > pivotMax){
      return pivotMax;
    }
    return target;
  }

  public void stopPivot(){
    m_pivotMotor.stopMotor();
  }

  public Command setPivotSpeedCommand(double speed) {
    return runEnd(
      ()-> {
        m_pivotMotor.set(speed);
      },
      () -> {
        m_pivotMotor.set(0);
      }
    );
  }
}

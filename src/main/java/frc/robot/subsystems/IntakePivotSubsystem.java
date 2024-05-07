package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.Helpers;
import frc.robot.utils.InterpolatorTables;
import frc.robot.utils.PIDTuner;
import monologue.Logged;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class IntakePivotSubsystem extends SubsystemBase implements Logged {

    public static final double PIVOT_ZERO = 60.15;

    private static final double pivotMaxPosition = 317.87;
    private static final double pivotMinPosition = 223.013;

    public static final double FRONT = pivotMaxPosition - 2.5;
    public static final double STOW = (pivotMaxPosition + pivotMinPosition) / 2;
    public static final double BACK = pivotMinPosition + 2.5;
    public static final double SHOOTER_FEED = 285;
    public static final double AMP_FEED = 243;
    public static final double FEED_SHOT = 298;
    public static final double TRAP_FEED = 298.01;

    private CANSparkMax m_intakePivotMotor;
    private SparkPIDController m_pivotPID;
    private SparkAbsoluteEncoder m_pivotEncoder;

    private final int intakePivotMotorID = 25;
    private Double m_setpoint = SHOOTER_FEED;
    private PIDTuner pivotTuner = null;
    private boolean aligned = false;
    
    public IntakePivotSubsystem() {
        m_intakePivotMotor = new CANSparkMax(intakePivotMotorID, MotorType.kBrushless);
        m_intakePivotMotor.restoreFactoryDefaults();
        m_intakePivotMotor.setIdleMode(IdleMode.kBrake);
        Helpers.setSparkInverted(m_intakePivotMotor, true);

        m_pivotEncoder = m_intakePivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_pivotEncoder.setPositionConversionFactor(360); // Change units to degrees
        m_pivotEncoder.setInverted(false);
        m_pivotEncoder.setZeroOffset(PIVOT_ZERO);

        m_pivotPID = m_intakePivotMotor.getPIDController();
        m_pivotPID.setP(0.045);
        m_pivotPID.setOutputRange(-1, 1);
        m_pivotPID.setFeedbackDevice(m_pivotEncoder);
        m_pivotPID.setPositionPIDWrappingEnabled(false);

        // pivotTuner = new PIDTuner(m_pivotPID, ()-> m_setpoint)
        //     .withName("Intake Pivot")
        //     .withValue(m_pivotEncoder::getPosition)
        //     .withOutput(m_intakePivotMotor::get);
        // pivotTuner.initializeValues(m_pivotPID);
        
        SmartDashboard.putNumber("Intake Feed Angle", 290);

        m_intakePivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        m_intakePivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        m_intakePivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        m_intakePivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor, default 50ms
        m_intakePivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        m_intakePivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // duty cycle encoder position, default 200ms
        m_intakePivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

        initialize();
    }

    public void initialize() {
        stop();
    }

    public void zeroEncoders() {
    }

    public void stop() {
        m_intakePivotMotor.stopMotor();
    }

    public void getEncoderPosition() {
        m_pivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        double position = m_pivotEncoder.getPosition();
        if (m_setpoint != null) {
            double currentSetpoint = m_setpoint;

            if (currentSetpoint == SHOOTER_FEED) {
                currentSetpoint = InterpolatorTables.intakeAngleTable().get(RobotContainer.instance.vision.getSpeakerTableKey());
            }

            m_pivotPID.setReference(currentSetpoint, ControlType.kPosition);
            aligned = Math.abs(currentSetpoint - position) <= 1;
            log("Intake Pivot Target", currentSetpoint);
        } else {
            log("Intake Pivot Target", -1);
            stop();
            aligned = true;
        }

        log("Intake Pivot Aligned", aligned);
        log("Intake Pivot Position", position);
        log("Intake Pivot Temperature (C)", m_intakePivotMotor.getMotorTemperature());
        if (pivotTuner != null) pivotTuner.periodic();
    }

    public boolean isAligned() {
        return aligned;
    }

    public void setPose(Double intakeDegrees) {
        double oldSetpoint = m_setpoint;
        if(intakeDegrees != null) {
            m_setpoint = verifyRange(intakeDegrees, pivotMinPosition, pivotMaxPosition);
            if (m_setpoint != oldSetpoint) {
                double currentSetpoint = m_setpoint;
                if (currentSetpoint == SHOOTER_FEED) {
                    currentSetpoint = InterpolatorTables.intakeAngleTable().get(RobotContainer.instance.vision.getSpeakerTableKey());
                }
                aligned = Math.abs(currentSetpoint - m_pivotEncoder.getPosition()) <= 1;
            }
        }
    }

    public void setSpeed(double speed) {
        m_intakePivotMotor.set(speed);
    }

    public Command setSpeedCommand(double speed) {
        return runEnd(
            ()-> {
                setSpeed(speed);
            },
            () -> {
                stop();
            }
        );
    }

    private static double verifyRange(double value, double min, double max) {
        if (value < min) value = min;
        if (value > max) value  = max;

        return value;
    }

}

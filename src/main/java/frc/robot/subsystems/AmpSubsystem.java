package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.Helpers;
import frc.robot.utils.PIDTuner;
import monologue.Logged;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class AmpSubsystem extends SubsystemBase implements Logged {

    private static final double BOTTOM_OFFSET = 10;
    private static final double ZERO_OFFSET = 356.87 - BOTTOM_OFFSET;

    public static final double UP = 142;
    public static final double SCORE = 105;
    public static final double STOW = 18;
    public static final double TRAP = 125.5;

    private CANSparkMax m_ampArmMotor, m_ampRollerMotor;
    private SparkPIDController m_motorPID;
    public SparkAbsoluteEncoder m_ampEncoder;
    private DigitalInput noteSensor;
    private PIDTuner armTuner = null;

    private double m_setpoint = -1;
    private double pivotMinPosition = 0;
    private double pivotMaxPosition = 160;

    private double poseNumber;
    private boolean aligned = false;
    private boolean noteDetected = false;
    private boolean slowMode = false;
    private Pose2d lastScorePose = null;

    public AmpSubsystem() {
        m_ampArmMotor = new CANSparkMax(29, MotorType.kBrushless);
        m_ampRollerMotor = new CANSparkMax(30, MotorType.kBrushless);

        m_ampArmMotor.restoreFactoryDefaults();
        Helpers.setSparkInverted(m_ampArmMotor, true);
        m_ampArmMotor.setIdleMode(IdleMode.kBrake);

        m_ampRollerMotor.restoreFactoryDefaults();
        Helpers.setSparkInverted(m_ampRollerMotor, true);
        m_ampRollerMotor.setIdleMode(IdleMode.kBrake);
        m_ampRollerMotor.setSmartCurrentLimit(40);
        
        m_ampEncoder = m_ampArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_ampEncoder.setInverted(true);
        m_ampEncoder.setPositionConversionFactor(360);
        m_ampEncoder.setZeroOffset(ZERO_OFFSET);

        noteSensor = new DigitalInput(3);
        
        m_motorPID = m_ampArmMotor.getPIDController();
        m_motorPID.setP(0.015, 0);
        m_motorPID.setP(0.009, 1);
        m_motorPID.setFeedbackDevice(m_ampEncoder);

        // armTuner = new PIDTuner(m_motorPID, ()-> m_setpoint)
        //             .withName("Amp Arm")
        //             .withValue(m_ampEncoder::getPosition)
        //             .withOutput(m_ampArmMotor::get);
        // armTuner.initializeValues(m_motorPID);

        stop();
    }

    public void stop() {
        m_ampArmMotor.set(0);
        m_ampRollerMotor.set(0);
        m_setpoint = -1;
    }

    public void zeroEncoders() {
        m_ampEncoder.setZeroOffset(0);
    }

    public void setAmpPose(Double pivotDegrees, boolean slowMode) {
        if(pivotDegrees != null) {
            m_setpoint = verifyRange(pivotDegrees, pivotMinPosition, pivotMaxPosition);
        }
        this.slowMode = slowMode;
        this.poseNumber = pivotDegrees;
    }

    public boolean awayFromScorePose() {
        if (lastScorePose == null) return false;
        
        double distance = RobotContainer.instance.drivetrain.getPose().getTranslation().getDistance(lastScorePose.getTranslation());
        return Math.abs(distance) >= 0.5;
    }

    public Command saveScorePose() {
        return Commands.runOnce(() -> {
            lastScorePose = RobotContainer.instance.drivetrain.getPose();
        });
    }

    public Command clearScorePose() {
        return Commands.runOnce(() -> {
            lastScorePose = null;
        });
    }

    public boolean isAligned() {
        return aligned;
    }

    public double getPoseNumber() {
        return poseNumber;
    }

    private static double verifyRange(double value, double min, double max) {
        
        return value;
    }

    @Override
    public void periodic() {
        if (m_setpoint != -1) {
            int pidSlot;
            double arbFF;
            if (slowMode) {
                pidSlot = 1;
                arbFF = 0;
            } else {
                pidSlot = 0;
                arbFF = 0.53;
            }        
            m_motorPID.setReference(m_setpoint, ControlType.kPosition, pidSlot, arbFF);
        }
        double position = m_ampEncoder.getPosition();
        double error = m_setpoint != -1 ? (m_setpoint - position) : 0;
        aligned = Math.abs(error) <= (slowMode ? 3 : 4.5);
        noteDetected = noteSensor.get();
        log("Note Detected?", noteDetected);
        log("Setpoint", m_setpoint);
        log("Encoder Position", position);
        log("Encoder Error", error);
        log("Aligned?", aligned);

        if (armTuner != null) armTuner.periodic();
    }

    public boolean isNoteDetected() {
        return noteDetected;
    }

    public Command setArmSpeed(double speed) {
        return runEnd(() -> {
            m_ampArmMotor.set(speed);
        },
        () -> {
            m_ampArmMotor.set(0);
        });
    }

    public Command setRollerSpeed(double speed) {
        return runEnd(() -> {
            m_ampRollerMotor.set(speed);
        },
        () -> {
            m_ampRollerMotor.set(0);
        });
    }
}


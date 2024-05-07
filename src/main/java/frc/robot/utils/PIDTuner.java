package frc.robot.utils;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * PIDTuner is a class for tuning PID values in real time using NetworkTables.
 */
public class PIDTuner {

  public double p, i, d, ff;

  private Consumer<Double> pSetter;
  private Consumer<Double> iSetter;
  private Consumer<Double> dSetter;
  private Consumer<Double> ffSetter;
  private Supplier<Double> targetGetter;
  private Supplier<Double> valueGetter = null;
  private Supplier<Double> outputGetter = null;
  private String name = "";
  private NetworkTable nt = null;

  public PIDTuner(PIDController pid) {
    this(pid::setP, pid::setI, pid::setD, null, pid::getSetpoint);
  }

  public PIDTuner(ProfiledPIDController pid, Supplier<Double> targetGetter) {
    this(pid::setP, pid::setI, pid::setD, null, targetGetter);
  }

  public PIDTuner(SparkPIDController pid, Supplier<Double> targetGetter) {
    this(pid::setP, pid::setI, pid::setD, pid::setFF, targetGetter);
  }

  public PIDTuner(Slot0Configs config, TalonFX talonFX, Supplier<Double> targetGetter) {
    this(
      (p) -> {
        config.kP = p;
        talonFX.getConfigurator().apply(config);
      },
      (i) -> {
        config.kI = i;
        talonFX.getConfigurator().apply(config);
      },
      (d) -> {
        config.kD = d;
        talonFX.getConfigurator().apply(config);
      },
      (ff) -> {
        config.kV = ff;
        talonFX.getConfigurator().apply(config);
      },
      targetGetter
    );
  }

  public PIDTuner(Consumer<Double> pSetter, 
                  Consumer<Double> iSetter,
                  Consumer<Double> dSetter,
                  Consumer<Double> ffSetter,
                  Supplier<Double> targetGetter) {
    this.pSetter = pSetter;
    this.iSetter = iSetter;
    this.dSetter = dSetter;
    this.ffSetter = ffSetter;
    this.targetGetter = targetGetter;
    nt = NetworkTableInstance.getDefault().getTable("PID Tuners").getSubTable("Unnamed Tuner");
  }

  /**
   * Sets the supplier of the sensor value, which is displayed in NetworkTables for comparing to the target.
   * @param valueGetter
   * @return This PIDTuner.
   */
  public PIDTuner withValue(Supplier<Double> valueGetter) {
    this.valueGetter = valueGetter;
    return this;
  }

  /**
   * Sets the supplier of the motor output, which is displayed in NetworkTables.
   * @param outputGetter
   * @return This PIDTuner.
   */
  public PIDTuner withOutput(Supplier<Double> outputGetter) {
    this.outputGetter = outputGetter;
    return this;
  }

  /**
   * Sets the name of the PIDTuner in NetworkTables.
   * @param name
   * @return This PIDTuner.
   */
  public PIDTuner withName(String name) {
    this.name = name;
    nt = NetworkTableInstance.getDefault().getTable("PID Tuners").getSubTable(this.name);
    return this;
  }

  /**
   * Initializes the PID values to the ones stored in the PIDController.
   * @param pid
   */
  public void initializeValues(PIDController pid) {
    initializeValues(pid.getP(), pid.getI(), pid.getD(), 0);
  }

  /**
   * Initializes the PID values to the ones stored in the ProfiledPIDController.
   * @param pid
   */
  public void initializeValues(ProfiledPIDController pid) {
    initializeValues(pid.getP(), pid.getI(), pid.getD(), 0);
  }

  /**
   * Initializes the PID values to the ones stored in the SparkPIDController.
   * @param pid
   */
  public void initializeValues(SparkPIDController pid) {
    initializeValues(pid.getP(), pid.getI(), pid.getD(), pid.getFF());
  }

  /**
   * Initializes the PID values to the ones stored in the Slot0Configs.
   * @param config
   */
  public void initializeValues(Slot0Configs config) {
    initializeValues(config.kP, config.kI, config.kD, config.kV);
  }

  /**
   * Initializes the PID values to the given starting values.
   * @param p
   * @param i
   * @param d
   * @param ff
   */
  public void initializeValues(double p, double i, double d, double ff) {
    setP(p);
    setI(i);
    setD(d);
    setFF(ff);
  }

  public void periodic() {
    nt.getEntry("PID Value").setNumber(valueGetter == null ? -1 : valueGetter.get());
    nt.getEntry("PID Target").setNumber(targetGetter == null ? -1 : targetGetter.get());
    nt.getEntry("PID Output").setNumber(outputGetter == null ? -1 : outputGetter.get());
    if (valueGetter != null && targetGetter != null) {
      nt.getEntry("PID Error").setNumber(targetGetter.get() - valueGetter.get());
    }

    double nP = nt.getEntry("P Value").getDouble(p);
    double nI = nt.getEntry("I Value").getDouble(i);
    double nD = nt.getEntry("D Value").getDouble(d);
    double nFF = nt.getEntry("FF Value").getDouble(ff);
    if(nP != p) setP(nP);
    if(nI != i) setI(nI); 
    if(nD != d) setD(nD);
    if (nFF != ff) setFF(nFF); 
  }

  public void setP(double p) {
    this.p = p;
    pSetter.accept(p);
    nt.getEntry("P Value").setNumber(p);
  }

  public void setI(double i) {
    this.i = i;
    iSetter.accept(i);
    nt.getEntry("I Value").setNumber(i);
  }

  public void setD(double d) {
    this.d = d;
    dSetter.accept(d);
    nt.getEntry("D Value").setNumber(d);
  }

  public void setFF(double ff) {
    this.ff = ff;
    if (ffSetter != null) ffSetter.accept(ff);
    nt.getEntry("FF Value").setNumber(ff);
  }
}

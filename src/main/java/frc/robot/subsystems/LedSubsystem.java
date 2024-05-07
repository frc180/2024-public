package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  public enum LED_PATTERN {
    NO_NOTE(0),
    INTAKING_NOTE(1),
    HAS_NOTE(2),
    HAS_TARGET(3),
    RAINBOW(4),
    OFF(20);

    public final int index;
    public final double value;

    private LED_PATTERN(int index) {
      this.index = index;
      this.value = index * 0.05;
    }
  }

  private Servo leds;
  private LED_PATTERN currentPattern;

  public LedSubsystem() {
    leds = new Servo(9);
    setLedPattern(LED_PATTERN.OFF);
  }

  @Override
  public void periodic() {
    leds.set(currentPattern.value);
  }

  public void setLedPattern(LED_PATTERN pattern) {
    currentPattern = pattern;
  }

  public Command ledPattern(LED_PATTERN pattern) {
    return run(() -> setLedPattern(pattern));
  }
}
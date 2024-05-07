package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class FanSubsystem extends SubsystemBase implements Logged {

    private final CANSparkMax fanMotor;

    public FanSubsystem() {
        fanMotor = new CANSparkMax(31, MotorType.kBrushed);
        fanMotor.restoreFactoryDefaults();
        fanMotor.setIdleMode(IdleMode.kCoast);
    }

    public Command setSpeed(double speed) {
        return runEnd(
            () -> {
                fanMotor.set(speed);
            },
            () -> {
                fanMotor.set(0);
            }
        );
    }
}

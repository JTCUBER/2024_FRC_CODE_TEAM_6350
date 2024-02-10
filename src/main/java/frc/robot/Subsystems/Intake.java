package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Intake Subsystem is a simple roller system with a single motor for intakeing and reversing
 */


public class Intake extends SubsystemBase{

    private final CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless);

    /** Creat a new roller intake subsystem */

    public Intake() {
        super();
        m_motor.setSmartCurrentLimit(30);
        m_motor.setInverted(false);
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void log() {

    }

    public void open() {
        m_motor.set(1);
    }

    public void close(double speed) {
        if (speed > 0) {                  // protect against coding from reversing by accident
            m_motor.set(0);
        }
        else {
            this.stop();
        }
    }

    /** Stops the intake motor from moving. */

    public void stop() {
        m_motor.set(0);
    }

    /** Call log method every loop */
    @Override
    public void periodic() {
            log();
    }
    
}

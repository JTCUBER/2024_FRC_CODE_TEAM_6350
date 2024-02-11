package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

/**
 * The Arm Subsystem is a single pivit point with 2 motors controlling it
 */

public class Arm extends SubsystemBase {

    private final CANSparkMax m_ArmLeft = new CANSparkMax(SuperStructure.kArmLeftCANId, MotorType.kBrushless);
    private final CANSparkMax m_ArmRight = new CANSparkMax(SuperStructure.kArmRightCANId, MotorType.kBrushless);

    /** Create a new pivit arm subsystem */

    public Arm() {
        super();
        m_ArmLeft.setSmartCurrentLimit(40);
        m_ArmRight.setSmartCurrentLimit(40);

        m_ArmLeft.setIdleMode(IdleMode.kBrake);
        m_ArmRight.setIdleMode(IdleMode.kBrake);

        m_ArmLeft.setInverted(false);
        m_ArmRight.setInverted(true);

    }

    public void log() {

    }

    public void up() {

        m_ArmLeft.set(0);
        m_ArmRight.set(0);

    }

    public void down() {

        m_ArmLeft.set(0);
        m_ArmRight.set(0);

    }

    public void stop() {

        m_ArmLeft.set(0);
        m_ArmRight.set(0);

    }

    @Override
    public void periodic() {
        log();
    }
}

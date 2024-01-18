package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants.ModuleContants;

public class MAXSwerveModule {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final SparkPIDController m_turningPIDController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        m_drivingEncoder.setPositionConversionFactor(ModuleContants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleContants.kDrivingEncoderVelocityFactor);

        m_turningEncoder.setPositionConversionFactor(ModuleContants.kTurningEncoderPostionFactor);
        m_turningEncoder.setVelocityConversionFactor(ModuleContants.kTurningEncoderVelocityFactor);

        m_turningEncoder.setInverted(ModuleContants.kTurningEncoderInverted);

        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(ModuleContants.kTurningEncoderPostionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleContants.kTurningEncoderPostionPIDMaxInput);

        m_drivingPIDController.setP(ModuleContants.kDrivingP);
        m_drivingPIDController.setI(ModuleContants.kDrivingI);
        m_drivingPIDController.setD(ModuleContants.kDrivingD);
        m_drivingPIDController.setFF(ModuleContants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleContants.kDrivingMinOutput, ModuleContants.kDrivingMaxOutput);

        m_turningPIDController.setP(ModuleContants.kTurningP);
        m_turningPIDController.setI(ModuleContants.kTurningI);
        m_turningPIDController.setD(ModuleContants.kTurningD);
        m_turningPIDController.setFF(ModuleContants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleContants.kTurningMinOutput, ModuleContants.kTurningMaxOutput);

        m_drivingSparkMax.setIdleMode(ModuleContants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleContants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleContants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleContants.kTurningMotorCurrentLimit);

        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));

    }
    

    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));

    }


    public void setDesiredStates(SwerveModuleState desiredState) {

        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        SwerveModuleState optimiziedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(m_turningEncoder.getPosition()));

        m_drivingPIDController.setReference(optimiziedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimiziedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}

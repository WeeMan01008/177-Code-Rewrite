package frc.robot.subsystems.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.hoodConstants;
import frc.robot.Constants.intakeConstants;

public class HoodState extends SubsystemBase {
    private TalonFX hood;

    public HoodState() {
        configureHood();
    }

    private final MotionMagicExpoVoltage hoodPositionRequest = new MotionMagicExpoVoltage(0);

    public enum HoodStates {
        Idle,
        PositionShoot,
        PositionPass
    }

    private HoodStates CurrentState = HoodStates.Idle;

    public void configureHood() {
        var hoodConfigure = new TalonFXConfiguration();
        hood = new TalonFX(hoodConstants.HOOD_MOTOR_ID, new CANBus("rio"));
        hoodConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
        hoodConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfigure.Slot0.kS = 0.25;
        hoodConfigure.Slot0.kV = 0.2;
        hoodConfigure.Slot0.kP = 12;
        hoodConfigure.Slot0.kI = 0;
        hoodConfigure.Slot0.kD = 0;

        hoodConfigure.MotionMagic.MotionMagicCruiseVelocity = 0;
        hoodConfigure.MotionMagic.MotionMagicAcceleration = 700;
        hoodConfigure.MotionMagic.MotionMagicExpo_kA = 0.01;
        hoodConfigure.MotionMagic.MotionMagicExpo_kV = 0.4;

        hood.getConfigurator().apply(hoodConfigure);
    }

    public void setHoodStop() {
        hood.stopMotor();
    }

    public void setHoodPosition(double angle) {
        hood.setControl(hoodPositionRequest.withPosition(angle));
    }

    public StatusSignal<Angle> getHoodPosition() {
        return hood.getPosition();
    }

    public void setCurrentState(HoodStates state) {
        CurrentState = state;
    }

    @Override
    public void periodic() {
        switch (CurrentState) {
            case Idle:
                this.setHoodStop();

                break;

            case PositionShoot:
                this.setHoodPosition(hoodConstants.HOOD_TEST_POSITION);

                break;

            case PositionPass:
                this.setHoodPosition(hoodConstants.HOOD_TEST_POSITION);

                break;
        }
    }
}

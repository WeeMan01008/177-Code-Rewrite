package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.hoodConstants;

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

    private HoodStates CurrentState = HoodStates.PositionShoot;

    public void configureHood() {
        var hoodConfigure = new TalonFXConfiguration();
        hood = new TalonFX(hoodConstants.HOOD_MOTOR_ID, new CANBus("rio"));
        hoodConfigure.CurrentLimits.StatorCurrentLimit = hoodConstants.statorCurrentLimit;
        hoodConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfigure.CurrentLimits.SupplyCurrentLimit = hoodConstants.supplyCurrentLimit;
        hoodConfigure.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfigure.Slot0.kS = 0.25;
        hoodConfigure.Slot0.kV = 0.2;
        hoodConfigure.Slot0.kP = 12;
        hoodConfigure.Slot0.kI = 0;
        hoodConfigure.Slot0.kD = 0;

        hoodConfigure.MotionMagic.MotionMagicCruiseVelocity = 30;
        hoodConfigure.MotionMagic.MotionMagicAcceleration = 700;
        hoodConfigure.MotionMagic.MotionMagicJerk = 0;
        hoodConfigure.MotionMagic.MotionMagicExpo_kA = 0.01;
        hoodConfigure.MotionMagic.MotionMagicExpo_kV = 0.4;

        hood.getConfigurator().apply(hoodConfigure);

    }

    public void setHoodStop() {
        setHoodPosition(0);
        // hood.stopMotor();
    }

    public void setHoodPosition(double angle) {
        hood.setControl(hoodPositionRequest.withPosition(angle));
    }

    public void resetHoodZeroPosition() {
        hood.setPosition(0);
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
                Logger.recordOutput("Hood_Set_Position", hoodConstants.HOOD_TEST_POSITION);

                break;

            case PositionPass:
                this.setHoodPosition(hoodConstants.HOOD_TEST_POSITION);
                Logger.recordOutput("Hood_Set_Position", hoodConstants.HOOD_TEST_POSITION);

                break;
        }
    }
}

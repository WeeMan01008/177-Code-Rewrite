package frc.robot.subsystems.hotdog;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.hotdogConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeState.IntakeStates;

public class HotdogState extends SubsystemBase {
    private TalonFX hotdog;

    public HotdogState() {
        configureHotdog();
    }

    private final VelocityVoltage HotdogPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    private double velocity;

    public enum HotdogStates {
        Idle,
        // forward is toward carwash and reverse is toward intake
        Forward,
        Reverse,
    }

    private HotdogStates CurrentState = HotdogStates.Idle;

    public void configureHotdog() {
        hotdog = new TalonFX(hotdogConstants.HOTDOG_MOTOR_ID, new CANBus("rio"));
        var hotdogConfigure = new TalonFXConfiguration();
        hotdogConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
        hotdogConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
        hotdogConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hotdogConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hotdogConfigure.Slot0.kS = 0.359;
        hotdogConfigure.Slot0.kV = 0.114;
        hotdogConfigure.Slot0.kP = 1.758;
        hotdogConfigure.Slot0.kI = 0;
        hotdogConfigure.Slot0.kD = -0.002;
        hotdog.getConfigurator().apply(hotdogConfigure);
    }

    public void setHotdogVelocity(double velocity) {
        hotdog.setControl(HotdogPIDRequest.withVelocity(velocity));
    }

    public void setHotdogStop() {
        hotdog.stopMotor();
    }

    public void setHotdogReverseVelocity(double velocity) {
        hotdog.setControl(HotdogPIDRequest.withVelocity(-velocity));
    }

    public StatusSignal<AngularVelocity> getHotdogVelocity() {
        return hotdog.getVelocity();
    }

    public void setCurrentState(HotdogStates state) {
        CurrentState = state;
    }

    @Override
    public void periodic() {
        switch (CurrentState) {
            case Idle:
                this.setHotdogStop();

                break;

            case Forward:
                this.setHotdogVelocity(hotdogConstants.INTAKE_VELOCITY);

                break;

            case Reverse:
                this.setHotdogReverseVelocity(hotdogConstants.OUTTAKE_VELOCITY);

                break;
        }
    }
}

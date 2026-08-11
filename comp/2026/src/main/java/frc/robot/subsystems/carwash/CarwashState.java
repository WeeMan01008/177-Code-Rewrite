package frc.robot.subsystems.carwash;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.carwashConstants;
import frc.robot.Constants.intakeConstants;

public class CarwashState extends SubsystemBase {
    private TalonFX carwash;

    public CarwashState() {
        configureCarwash();
    }

    private final VelocityVoltage carwashPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

    public enum CarwashStates {
        Idle,
        // forward is into shooter and reverse is into hopper
        Foward,
        Reverse,
        Outtake
    }

    private CarwashStates CurrentState = CarwashStates.Idle;

    public void configureCarwash() {
        carwash = new TalonFX(carwashConstants.CARWASH_WASHER_MOTOR_ID, new CANBus("rio"));
        var carwashConfigure = new TalonFXConfiguration();
        carwashConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
        carwashConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
        carwashConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        carwashConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        carwashConfigure.Slot0.kS = 0.340;
        carwashConfigure.Slot0.kV = 0.123;
        carwashConfigure.Slot0.kP = 1.632;
        carwashConfigure.Slot0.kI = 0;
        carwashConfigure.Slot0.kD = -0.002;
        carwash.getConfigurator().apply(carwashConfigure);
    }

    public void setCarwashForwardVelocity(double velocity) {
        carwash.setControl(carwashPIDRequest.withVelocity(velocity));
    }

    public void setCarwashStop() {
        carwash.stopMotor();
    }

    public void setCarwashReverseVelocity(double velocity) {
        carwash.setControl(carwashPIDRequest.withVelocity(-velocity));
    }

    public StatusSignal<AngularVelocity> getCarwashVelocity() {
        return carwash.getVelocity();
    }

    public void setCurrentState(CarwashStates state) {
        CurrentState = state;
    }

    @Override
    public void periodic() {
        switch (CurrentState) {
            case Idle:
                this.setCarwashStop();

                break;

            case Foward:
                this.setCarwashForwardVelocity(carwashConstants.CARWASH_SPIN_VELOCITY);

                break;

            case Reverse:
                this.setCarwashReverseVelocity(carwashConstants.CARWASH_SPIN_REVERSE_VELOCITY);

                break;

            case Outtake:
                this.setCarwashReverseVelocity(carwashConstants.CARWASH_OUTTAKE_VELOCITY);
        }
    }
}

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

public class HotdogState extends SubsystemBase{
    public TalonFX hotdog;

    final VelocityVoltage HotdogPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    private double velocity;

public enum HotdogStates {
    idle, 
    //forward is toward carwash and reverse is toward intake
    forward,
    reverse,
}

public void configureIntakeDeploy(){
    hotdog = new TalonFX(hotdogConstants.HOTDOG_MOTOR_ID, new CANBus("rio"));
    var hotdogConfigure = new TalonFXConfiguration();
    hotdogConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
    hotdogConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
    hotdogConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hotdogConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hotdogConfigure.Slot0.kS = 0;
    hotdogConfigure.Slot0.kV = 0;
    hotdogConfigure.Slot0.kP = 0;
    hotdogConfigure.Slot0.kI = 0;
    hotdogConfigure.Slot0.kD = 0;
    hotdog.getConfigurator().apply(hotdogConfigure);
}

public void setHotdogVelocity(double velocity) {
    hotdog.setControl(HotdogPIDRequest.withVelocity(velocity));
}

public void setHotdogStop(){
    hotdog.stopMotor();
}

public void setHotdogReverseVelocity(double velocity) {
    hotdog.setControl(HotdogPIDRequest.withVelocity(-velocity));
}

public StatusSignal<AngularVelocity> getHotdogVelocity(){
    return hotdog.getVelocity();
}

public HotdogStates CurrentState = HotdogStates.idle;

@Override
public void periodic(){
    switch (CurrentState) {
        case idle:
            this.setHotdogStop();
            
        break;

        case forward:
            this.setHotdogVelocity(velocity);

        break;

        case reverse:
            this.setHotdogReverseVelocity(velocity);

        break;
    }
}
}

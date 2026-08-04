package frc.robot.subsystems.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.hoodConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.hotdog.HotdogState.HotdogStates;

//TODO: fix tabbing/spacing
public class HoodState extends SubsystemBase{
    private TalonFX hood;

    public HoodState() {
        configureHood();
    }

    private final PositionVoltage HoodPositionRequest = new PositionVoltage(null);
    private double velocity;

    public enum HoodStates {
        Idle, 
        PositionShoot,
        PositionPass
}

private HoodStates CurrentState = HoodStates.Idle;

public void configureHood(){
    hood = new TalonFX(hoodConstants.HOOD_MOTOR_ID, new CANBus("rio"));
    var hoodConfigure = new TalonFXConfiguration();
    hoodConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
    hoodConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfigure.Slot0.kS = 0;
    hoodConfigure.Slot0.kV = 0;
    hoodConfigure.Slot0.kP = 1;
    hoodConfigure.Slot0.kI = 0;
    hoodConfigure.Slot0.kD = 0;
    hood.getConfigurator().apply(hoodConfigure);
}

public void setHoodStop(){
    hood.stopMotor();
}

public void setHoodPosition(double angle) {
    hood.setControl(HoodPositionRequest.withPosition(angle).withFeedForward(0.5));
}

public StatusSignal<Angle> getHoodPosition(){
    return hood.getPosition();
}

public void setCurrentState(HoodStates state){
    CurrentState = state;
}

@Override
public void periodic(){
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

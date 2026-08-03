package frc.robot.subsystems.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    //TOOD: why public?
    private TalonFX hood;

    public HoodState() {
        configureHood();
    }

    // TODO: why no explicit access modifier
    private final VelocityVoltage HoodPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    private double velocity;

    public enum HoodStates {
        Idle, 
        Up,
        Down,
        PositionShoot,
        PositionPass
}

// TODO:  shouldn't expose public fields
private HoodStates CurrentState = HoodStates.Idle;

//TODO:  what executes this?
public void configureHood(){
    hood = new TalonFX(hoodConstants.HOOD_MOTOR_ID, new CANBus("rio"));
    var hoodConfigure = new TalonFXConfiguration();
    hoodConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
    hoodConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfigure.Slot0.kS = 0;
    hoodConfigure.Slot0.kV = 0;
    hoodConfigure.Slot0.kP = 0;
    hoodConfigure.Slot0.kI = 0;
    hoodConfigure.Slot0.kD = 0;
    hood.getConfigurator().apply(hoodConfigure);
}

public void setHoodUpVelocity(double velocity) {
    hood.setControl(HoodPIDRequest.withVelocity(velocity));
}

public void setHoodStop(){
    hood.stopMotor();
}

public void setHoodDownVelocity(double velocity) {
    hood.setControl(HoodPIDRequest.withVelocity(-velocity));
}

public void setHoodPosition(double angle) {
    hood.setPosition(angle);
}

public StatusSignal<AngularVelocity> getHoodVelocity(){
    return hood.getVelocity();
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

        case Up:
            //TODO:  nothing is setting velocity
            this.setHoodUpVelocity(velocity);

        break;

        case Down:
            this.setHoodDownVelocity(velocity);

        break;

        case PositionShoot:
            this.setHoodPosition(0);

        break;

        case PositionPass:
            this.setHoodPosition(0);
        
        break;
    }
}
}

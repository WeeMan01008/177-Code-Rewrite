package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeState extends SubsystemBase{
    public TalonFX intakeDeploy;
    public TalonFX rightIntakeRoller;
    public TalonFX leftIntakeRoller;

    final VelocityVoltage intakeRollerPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    final VelocityVoltage intakeDeployPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    private double velocity;
    private double angle;

public enum IntakeStates {
    idle, 
    intakeDeployed,
    IntakeStored,
    intake,
    outtake
}

public void configureIntakeDeploy(){
    intakeDeploy = new TalonFX(intakeConstants.INTAKE_DEPLOY_MOTOR_ID, new CANBus("rio"));
    var intakeDeployConfigure = new TalonFXConfiguration();
    intakeDeployConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
    intakeDeployConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeDeployConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeDeployConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeDeployConfigure.Slot0.kS = 0;
    intakeDeployConfigure.Slot0.kV = 0;
    intakeDeployConfigure.Slot0.kP = 0;
    intakeDeployConfigure.Slot0.kI = 0;
    intakeDeployConfigure.Slot0.kD = 0;
    intakeDeploy.getConfigurator().apply(intakeDeployConfigure);
}

public void configureIntakeRollers(){
    rightIntakeRoller = new TalonFX(intakeConstants.INTAKE_ROLLERLEFT_MOTOR_ID, new CANBus("rio"));
    var intakeRollersConfigure = new TalonFXConfiguration();
    intakeRollersConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.STATOR_CURRENT_LIMIT;
    intakeRollersConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeRollersConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeRollersConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeRollersConfigure.Slot0.kS = 0;
    intakeRollersConfigure.Slot0.kV = 0;
    intakeRollersConfigure.Slot0.kP = 0;
    intakeRollersConfigure.Slot0.kI = 0;
    intakeRollersConfigure.Slot0.kD = 0;
    rightIntakeRoller.getConfigurator().apply(intakeRollersConfigure);

    leftIntakeRoller.setControl(new Follower(intakeConstants.INTAKE_ROLLERRIGHT_MOTOR_ID, MotorAlignmentValue.Opposed));
}

public void setIntakeDeployedAngle(double angle) {
    intakeDeploy.setPosition(angle);
}

public void setIntakeDeployedStop(){
    intakeDeploy.stopMotor();
}

public void setIntakeStoredAngle(double angle) {
    intakeDeploy.setPosition(angle);
}

public void setLeftIntakeRollerVelocity(double velocity) {
    leftIntakeRoller.setControl(intakeRollerPIDRequest.withVelocity(velocity));
}

public void setLeftIntakeRollerOuttakeVelocity(double velocity) {
    leftIntakeRoller.setControl(intakeRollerPIDRequest.withVelocity(-velocity));
}

public void setLeftIntakeRollerStop(){
    leftIntakeRoller.stopMotor();
}

public StatusSignal<Angle> getIntakePosition(){
    return intakeDeploy.getPosition();
}

public StatusSignal<AngularVelocity> getRightIntakeVelocity(){
    return rightIntakeRoller.getVelocity();
}

public StatusSignal<AngularVelocity> getLeftIntakeVelocity(){
    return leftIntakeRoller.getVelocity();
}

public IntakeStates CurrentState = IntakeStates.idle;

@Override
public void periodic(){
    switch (CurrentState) {
        case idle:
            this.setIntakeDeployedStop();
            this.setLeftIntakeRollerStop();
            
        break;

        case intakeDeployed:
            this.setIntakeDeployedAngle(angle);

        break;

        case IntakeStored:
            this.setIntakeStoredAngle(angle);

        break;

        case intake:
            this.setLeftIntakeRollerVelocity(velocity);

        break;

        case outtake:
            this.setLeftIntakeRollerVelocity(velocity);
    }
}
}
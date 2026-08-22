package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeState extends SubsystemBase {
    private TalonFX intakeDeploy;
    private TalonFX rightIntakeRoller;
    private TalonFX leftIntakeRoller;

    public IntakeState() {
        configureIntakeDeploy();
        configureIntakeRollers();
    }

    private final VelocityVoltage intakeRollerPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    private final PositionVoltage intakeDeployPositionRequest = new PositionVoltage(0);
    private double velocity;
    private double angle;

    public enum IntakeStates {
        Idle,
        IntakeDeployed,
        IntakeStored,
        Intake,
        Outtake
    }

    private IntakeStates CurrentState = IntakeStates.Idle;

    public void configureIntakeDeploy() {
        intakeDeploy = new TalonFX(intakeConstants.INTAKE_DEPLOY_MOTOR_ID, new CANBus("rio"));
        var intakeDeployConfigure = new TalonFXConfiguration();
        intakeDeployConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.INTAKE_DEPLOY_STATOR_LIMIT;
        intakeDeployConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeDeployConfigure.CurrentLimits.SupplyCurrentLimit = intakeConstants.INTAKE_DEPLOY_SUPPLY_LIMIT;
        intakeDeployConfigure.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeDeployConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeDeployConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeDeployConfigure.Slot0.kS = 0;
        intakeDeployConfigure.Slot0.kV = 0;
        intakeDeployConfigure.Slot0.kP = 1;
        intakeDeployConfigure.Slot0.kI = 0;
        intakeDeployConfigure.Slot0.kD = 0;
        intakeDeploy.getConfigurator().apply(intakeDeployConfigure);
    }

    public void configureIntakeRollers() {
        rightIntakeRoller = new TalonFX(intakeConstants.INTAKE_ROLLERRIGHT_MOTOR_ID, new CANBus("rio"));
        leftIntakeRoller = new TalonFX(intakeConstants.INTAKE_ROLLERLEFT_MOTOR_ID, new CANBus("rio"));
        var intakeRollersConfigure = new TalonFXConfiguration();
        intakeRollersConfigure.CurrentLimits.StatorCurrentLimit = intakeConstants.INTAKE_STATOR_LIMIT;
        intakeRollersConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeRollersConfigure.CurrentLimits.SupplyCurrentLimit = intakeConstants.INTAKE_SUPPLY_LIMIT;
        intakeRollersConfigure.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeRollersConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeRollersConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeRollersConfigure.Slot0.kS = 0.241;
        intakeRollersConfigure.Slot0.kV = 0.126;
        intakeRollersConfigure.Slot0.kP = 1.589;
        intakeRollersConfigure.Slot0.kI = 0;
        intakeRollersConfigure.Slot0.kD = -0.002;
        rightIntakeRoller.getConfigurator().apply(intakeRollersConfigure);

        leftIntakeRoller
                .setControl(new Follower(intakeConstants.INTAKE_ROLLERRIGHT_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void setIntakeDeployedAngle(double angle) {
        intakeDeploy.setControl(intakeDeployPositionRequest.withPosition(angle).withFeedForward(1));
    }

    public void setIntakeStoredAngle(double angle) {
        intakeDeploy.setControl(intakeDeployPositionRequest.withPosition(angle).withFeedForward(-1.6));
    }

    public void setIntakeDeployedStop() {
        intakeDeploy.stopMotor();
    }

    public void setRightIntakeRollerVelocity(double velocity) {
        rightIntakeRoller.setControl(intakeRollerPIDRequest.withVelocity(velocity));
    }

    public void setRightIntakeRollerOuttakeVelocity(double velocity) {
        rightIntakeRoller.setControl(intakeRollerPIDRequest.withVelocity(-velocity));
    }

    public void setRightIntakeRollerStop() {
        rightIntakeRoller.stopMotor();
    }

    public void resetIntakeZeroPosition() {
        intakeDeploy.setPosition(0);
    }

    public StatusSignal<Angle> getIntakePosition() {
        return intakeDeploy.getPosition();
    }

    public StatusSignal<AngularVelocity> getRightIntakeVelocity() {
        return rightIntakeRoller.getVelocity();
    }

    public StatusSignal<AngularVelocity> getLeftIntakeVelocity() {
        return leftIntakeRoller.getVelocity();
    }

    public void setCurrentState(IntakeStates state) {
        CurrentState = state;
    }

    @Override
    public void periodic() {
        switch (CurrentState) {
            case Idle:
                this.setIntakeDeployedStop();
                this.setRightIntakeRollerStop();

                break;

            case IntakeDeployed:
                this.setIntakeDeployedAngle(intakeConstants.INTAKE_DEPLOYED_POSITION);

                break;

            case IntakeStored:
                this.setIntakeStoredAngle(intakeConstants.INTAKE_STORED_POSITION);

                break;

            case Intake:
                this.setRightIntakeRollerVelocity(intakeConstants.INTAKE_VELOCITY);

                break;

            case Outtake:
                this.setRightIntakeRollerOuttakeVelocity(intakeConstants.OUTTAKE_VELOCITY);
        }
    }
}
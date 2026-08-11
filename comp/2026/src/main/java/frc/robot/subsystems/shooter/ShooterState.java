package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class ShooterState extends SubsystemBase {
    // motors listed as viewed from back of the robot
    private TalonFX topRightShooter;
    private TalonFX topLeftShooter;
    private TalonFX bottomRightShooter;
    private TalonFX bottomLeftShooter;

    public ShooterState() {
        configureTopRightShooter();
    }

    private final VelocityVoltage shooterMotorPIDRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    private double velocity;

    public enum ShooterStates {
        Idle,
        Shoot,
        Reverse,
        Pass
    }

    private ShooterStates CurrentState = ShooterStates.Idle;

    public void configureTopRightShooter() {
        topRightShooter = new TalonFX(shooterConstants.TOPRIGHT_SHOOTER_MOTOR_ID, new CANBus("rio"));
        bottomRightShooter = new TalonFX(shooterConstants.BOTTOMRIGHT_SHOOTER_MOTOR_ID, new CANBus("rio"));
        topLeftShooter = new TalonFX(shooterConstants.TOPLEFT_SHOOTER_MOTOR_ID, new CANBus("rio"));
        bottomLeftShooter = new TalonFX(shooterConstants.BOTTOMLEFT_SHOOTER_MOTOR_ID, new CANBus("rio"));
        var shooterConfigure = new TalonFXConfiguration();
        shooterConfigure.CurrentLimits.StatorCurrentLimit = shooterConstants.STATOR_CURRENT_LIMIT;
        shooterConfigure.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfigure.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfigure.Slot0.kS = 0.273;
        shooterConfigure.Slot0.kV = 0.120;
        shooterConfigure.Slot0.kP = 1.664;
        shooterConfigure.Slot0.kI = 0;
        shooterConfigure.Slot0.kD = -0.002;
        topRightShooter.getConfigurator().apply(shooterConfigure);

        bottomRightShooter
                .setControl(new Follower(shooterConstants.TOPRIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Aligned));
        topLeftShooter
                .setControl(new Follower(shooterConstants.TOPRIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
        bottomLeftShooter
                .setControl(new Follower(shooterConstants.TOPRIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void setTopRightShooterVelocity(double velocity) {
        topRightShooter.setControl(shooterMotorPIDRequest.withVelocity(velocity));
    }

    public void setTopRightShooterUnjamVelocity(double velocity) {
        topRightShooter.setControl(shooterMotorPIDRequest.withVelocity(-velocity));
    }

    public void stopStopRightShooter() {
        topRightShooter.stopMotor();
    }

    public StatusSignal<AngularVelocity> getTopRightShooterVelocity() {
        return topLeftShooter.getVelocity();
    }

    public StatusSignal<AngularVelocity> getTopLeftShooterVelocity() {
        return topRightShooter.getVelocity();
    }

    public StatusSignal<AngularVelocity> getBottomRightShooterVelocity() {
        return bottomRightShooter.getVelocity();
    }

    public StatusSignal<AngularVelocity> getBottomLeftShooterVelocity() {
        return bottomLeftShooter.getVelocity();
    }

    public void setCurrentState(ShooterStates state) {
        CurrentState = state;
    }

    @Override
    public void periodic() {
        switch (CurrentState) {
            case Idle:
                topRightShooter.stopMotor();

                break;

            case Shoot:
                this.setTopRightShooterVelocity(shooterConstants.SHOOTING_TEST_SPEEDS);

                break;

            case Pass:
                this.setTopRightShooterVelocity(velocity);

                break;

            case Reverse:
                this.setTopRightShooterUnjamVelocity(velocity);
        }
    }
}
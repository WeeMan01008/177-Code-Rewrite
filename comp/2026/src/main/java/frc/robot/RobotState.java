package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.carwash.CarwashState;
import frc.robot.subsystems.carwash.CarwashState.CarwashStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodState;
import frc.robot.subsystems.hood.HoodState.HoodStates;
import frc.robot.subsystems.hotdog.HotdogState;
import frc.robot.subsystems.hotdog.HotdogState.HotdogStates;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeState.IntakeStates;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterState.ShooterStates;

import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import org.bobcatrobotics.Util.Interpolators.*;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;

public class RobotState extends SubsystemBase {
    private double distanceToHub = 0.0;
    private double distanceToPass = 0.0;
    public Translation2d passLocation = new Translation2d();
    // private double passingVelocity = shooterConstants.SHOOTER_SPEEDS[0];
    // private double shooterVelocity = shooterConstants.PASSING_SPEEDS[0];

    private IntakeState intakeState = new IntakeState();
    private ShooterState shooterState = new ShooterState();
    private HotdogState hotdogState = new HotdogState();
    private HoodState hoodState = new HoodState();
    private CarwashState carwashState = new CarwashState();
    private Drive drive;
    private Alliance alliance = Alliance.Red;

    public Translation2d hubLocation = HubUtil.getMyHubCoordinates(alliance).toPose2d().getTranslation();

    public RobotState(Drive drive) {
        this.drive = drive;
        passLocation = HubUtil.getMyPassingCoordinates(alliance, drive).toPose2d().getTranslation();
    }

    // public DoubleOutputInterpolator shootingInterpolator = new
    // DoubleOutputInterpolator(shooterConstants.SHOOTING_DISTANCES,
    // shooterConstants.SHOOTER_SPEEDS, hoodConstants.HOOD_POSITION_SHOOTING,
    // false);
    // public DoubleOutputInterpolator passingInterpolator = new
    // DoubleOutputInterpolator(shooterConstants.PASSING_DISTANCES,
    // shooterConstants.PASSING_SPEEDS, hoodConstants.HOOD_POSITION_PASSING, false);

    public enum RobotStates {
        Idle,
        Shoot,
        Pass,
        SpinUp,
        Intake,
        Outtake,
        Unjam,
        IntakeStored,
        IntakeDeployed
    }

    private RobotStates CurrentState = RobotStates.Idle;

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public double getShooterVelocity() {
        return shooterState.getTopRightShooterVelocity().getValueAsDouble();
    }

    public void setCurrentState(RobotStates state) {
        CurrentState = state;
        Logger.recordOutput("State", CurrentState);
    }

    public void setDistanceToHub(double distance) {
        distanceToHub = distance;
        // shooterVelocity = shootingInterpolator.getAsList(distanceToHub).get(0);
    }

    public double getDistanceToHub() {
        return distanceToHub;
    }

    public void setPassingLocation(Translation2d loc) {
        passLocation = loc;
    }

    public Command setStateIdle() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.Idle),
                shooterState, intakeState, hotdogState, carwashState);
    }

    public Command setStateShoot() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.Shoot),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    public Command setStatePass() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.Pass),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    public Command setStateSpinUp() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.SpinUp),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    public Command setStateIntake() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.Intake),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    public Command setStateOuttake() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.Outtake),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    public Command setStateUnjam() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.Unjam),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    public Command setStateIntakeStored() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.IntakeStored),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    public Command setStateIntakeDeploy() {
        return Commands.runOnce(() -> setCurrentState(RobotStates.IntakeDeployed),
                shooterState, intakeState, hotdogState, hoodState, carwashState);
    }

    @Override
    public void periodic() {
        switch (CurrentState) {
            case Idle:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Idle);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Idle);

                break;

            case Shoot:
                // shooterState.setCurrentState(ShooterStates.Shoot);
                // intakeState.setCurrentState(IntakeStates.Intake);
                // hotdogState.setCurrentState(HotdogStates.Forward);
                hoodState.setCurrentState(HoodStates.PositionShoot);
                // carwashState.setCurrentState(CarwashStates.Foward);

                break;

            case Pass:
                shooterState.setCurrentState(ShooterStates.Pass);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Forward);
                hoodState.setCurrentState(HoodStates.PositionPass);
                carwashState.setCurrentState(CarwashStates.Foward);

                break;

            case SpinUp:
                // shooterState.setCurrentState(ShooterStates.Shoot);
                // intakeState.setCurrentState(IntakeStates.Intake);
                // hotdogState.setCurrentState(HotdogStates.Reverse);
                hoodState.setCurrentState(HoodStates.PositionShoot);
                // carwashState.setCurrentState(CarwashStates.Reverse);

                break;

            case Intake:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Intake);
                hotdogState.setCurrentState(HotdogStates.Forward);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Reverse);

                break;

            case Outtake:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Outtake);
                hotdogState.setCurrentState(HotdogStates.Reverse);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Outtake);

                break;

            case Unjam:
                shooterState.setCurrentState(ShooterStates.Reverse);
                intakeState.setCurrentState(IntakeStates.Intake);
                hotdogState.setCurrentState(HotdogStates.Reverse);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Reverse);

                break;

            case IntakeStored:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.IntakeStored);
                hotdogState.setCurrentState(HotdogStates.Idle);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Idle);

                break;

            case IntakeDeployed:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.IntakeDeployed);
                hotdogState.setCurrentState(HotdogStates.Idle);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Idle);

                break;
        }

        logStateInformation();
    }

    public void logStateInformation() {
        if (DriverStation.getAlliance().isPresent()) {
            setAlliance(DriverStation.getAlliance().get());
            hubLocation = HubUtil.getMyHubCoordinates(DriverStation.getAlliance().get()).toPose2d()
                    .getTranslation();
        }

        setPassingLocation(HubUtil
                .getMyPassingCoordinates(alliance,
                        drive)
                .toPose2d().getTranslation());
        Logger.recordOutput("Hood_Position", hoodState.getHoodPosition().getValueAsDouble());
        Logger.recordOutput("Hood_State", hoodState.CurrentState);
    }

}

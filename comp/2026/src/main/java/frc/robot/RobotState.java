package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.carwash.CarwashState;
import frc.robot.subsystems.carwash.CarwashState.CarwashStates;
import frc.robot.subsystems.hood.HoodState;
import frc.robot.subsystems.hood.HoodState.HoodStates;
import frc.robot.subsystems.hotdog.HotdogState;
import frc.robot.subsystems.hotdog.HotdogState.HotdogStates;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeState.IntakeStates;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterState.ShooterStates;

public class RobotState extends SubsystemBase {
    private IntakeState intakeState = new IntakeState();
    private ShooterState shooterState = new ShooterState();
    private HotdogState hotdogState = new HotdogState();
    private HoodState hoodState = new HoodState();
    private CarwashState carwashState = new CarwashState();

    public enum RobotStates {
        Idle,
        Shoot,
        Pass,
        SpinUp,
        Intake,
        Outtake,
        Unjam,
        IntakeStored,
        IntakeDeployed,
        AlignToPass,
        AlignToHub
    }

    private RobotStates CurrentState = RobotStates.Idle;

    public void setCurrentState(RobotStates state){
        CurrentState = state;
}

    @Override
    public void periodic(){
        switch (CurrentState){
            case Idle:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Idle);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Idle);

            break;

            case Shoot:
                shooterState.setCurrentState(ShooterStates.Shoot);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Forward);
                hoodState.setCurrentState(HoodStates.PositionShoot);
                carwashState.setCurrentState(CarwashStates.Foward);

            break;

            case Pass:
                shooterState.setCurrentState(ShooterStates.Pass);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Forward);
                hoodState.setCurrentState(HoodStates.PositionPass);
                carwashState.setCurrentState(CarwashStates.Foward);

            break;

            case SpinUp:
                shooterState.setCurrentState(ShooterStates.Shoot);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Reverse);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Reverse);

            break;

            case Intake:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Intake);
                hotdogState.setCurrentState(HotdogStates.Forward);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Idle);

            break;

            case Outtake:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Outtake);
                hotdogState.setCurrentState(HotdogStates.Reverse);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Reverse);

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

            case AlignToHub:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Idle);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Idle);

            break;

            case AlignToPass:
                shooterState.setCurrentState(ShooterStates.Idle);
                intakeState.setCurrentState(IntakeStates.Idle);
                hotdogState.setCurrentState(HotdogStates.Idle);
                hoodState.setCurrentState(HoodStates.Idle);
                carwashState.setCurrentState(CarwashStates.Idle);

            break;
        }
    }
    
}

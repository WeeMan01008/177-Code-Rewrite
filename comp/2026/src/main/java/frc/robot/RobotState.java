package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
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

import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
//import org.bobcatrobotics.Util.DoubleOutPutInterpolator;

public class RobotState extends SubsystemBase {
    private static RobotState instance;
    private double distanceToHub = 0.0;
    private double distanceToPass = 0.0;
    private double passingVelocity = shooterConstants.SHOOTER_SPEEDS[0];
    private double shooterVelocity = shooterConstants.PASSING_SPEEDS[0];

    private IntakeState intakeState = new IntakeState();
    private ShooterState shooterState = new ShooterState();
    private HotdogState hotdogState = new HotdogState();
    private HoodState hoodState = new HoodState();
    private CarwashState carwashState = new CarwashState();
    private Alliance alliance = Alliance.Red;

    public Translation2d hubLocation = HubUtil.getMyHubCoordinates(alliance).toPose2d().getTranslation();

    //public DoubleOutPutInterpolator shootingInterpolator = new DoubleOutPutInterpolator();
    //public DoubleOutPutInterpolator passingInterpolator = new DoubleOutPutInterpolator();

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

    public static RobotState getInstance() {
      if (instance == null)
      {
        instance = new RobotState();
      }
      return instance;
    }

    public void setAlliance(Alliance alliance) {
      this.alliance = alliance;
}

    public void setCurrentState(RobotStates state){
        CurrentState = state;
}

    public void setDistanceToHub(double distance) { 
        distanceToHub = distance; 
        //shooterVelocity = shootingInterpolator.getAsList(distanceToHub).get(0);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.cameraConstants;

import frc.robot.Constants.hoodConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.RobotState.RobotStates;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.vision.VisionConstants;

import org.bobcatrobotics.Commands.ActionFactory;
//import frc.robot.util.AllianceFlipUtil;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final RobotState robotState;
    public final Vision vision;
    public final Drive drive;
    private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        switch (Constants.currentMode) {

            case REAL:
                drive = new Drive(new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                vision = new Vision(drive::addVisionMeasurement,
                        new VisionIOLimelight(cameraConstants[0].name, drive::getRotation),
                        // new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[2].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[3].name, drive::getRotation));
                break;

            case SIM:
                drive = new Drive(new GyroIO() {
                }, new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                vision = new Vision(drive::addVisionMeasurement,
                        new VisionIOLimelight(cameraConstants[0].name, drive::getRotation),
                        // new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[2].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[3].name, drive::getRotation));
                break;

            default:
                drive = new Drive(new GyroIO() {
                }, new ModuleIO() {
                }, new ModuleIO() {
                }, new ModuleIO() {
                },
                        new ModuleIO() {
                        });

                vision = new Vision(drive::addVisionMeasurement,
                        new VisionIOLimelight(cameraConstants[0].name, drive::getRotation),
                        // new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[2].name, drive::getRotation),
                        new VisionIOLimelight(cameraConstants[3].name, drive::getRotation));
                break;
        }

        robotState = new RobotState(drive);

        registerNammedCommands();

        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Do Nothing", new PathPlannerAuto("Do Nothing"));
        autoChooser.addOption("Mid Sweep", new PathPlannerAuto("Single Mid Sweep"));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the trigger bindings
        configureBindings();
    }

    private void registerNammedCommands() {
        NamedCommands.registerCommand("Idle", robotState.setStateIdle());
        NamedCommands.registerCommand("Shoot", robotState.setStateShoot());
        NamedCommands.registerCommand("Pass", robotState.setStatePass());
        NamedCommands.registerCommand("SpinUp", robotState.setStateSpinUp());
        NamedCommands.registerCommand("Intake", robotState.setStateIntake());
        NamedCommands.registerCommand("Outtake", robotState.setStateOuttake());
        NamedCommands.registerCommand("Unjam", robotState.setStateUnjam());
        NamedCommands.registerCommand("Intake Stored", robotState.setStateIntakeStored());
        NamedCommands.registerCommand("Intake Deployed", robotState.setStateIntakeDeploy());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.

        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        driverController.rightBumper().whileTrue(DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d(
                        HubUtil.hubPosition(robotState.getAlliance()).getX()
                                - drive.getPose().getX(),
                        HubUtil.hubPosition(robotState.getAlliance()).getY()
                                - drive.getPose().getY())));

        driverController.rightTrigger().whileTrue(DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d(robotState.passLocation.getX() - drive.getPose().getX(),
                        robotState.passLocation.getY() - drive.getPose().getY())));

        driverController.x()
                .onTrue(new ActionFactory().singleAction("X-Command", () -> drive.stopWithX(), drive));

        driverController.b()
                .onTrue(new ActionFactory().singleAction("ZeroGyroCommand",
                        () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kZero))),
                        drive).ignoringDisable(true));

        driverController.a()
                .whileTrue(robotState.setStateSpinUp()
                        .until(() -> robotState
                                .getShooterVelocity() >= shooterConstants.SHOOTING_TEST_SPEEDS
                                        - 3)
                        .andThen(robotState.setStateShoot()))
                .onFalse(robotState.setStateIdle());

        driverController.y().whileTrue(robotState.setStateShoot())
                .onFalse(robotState.setStateIdle());

        driverController.leftBumper()
                .whileTrue(robotState.setStateIntake())
                .onFalse(robotState.setStateIdle());

        driverController.leftTrigger()
                .whileTrue(robotState.setStateOuttake())
                .onFalse(robotState.setStateIdle());

        driverController.povDown()
                .whileTrue(robotState.setStateIntakeDeploy())
                .onFalse(robotState.setStateIdle());

        driverController.povUp()
                .whileTrue(robotState.setStateIntakeStored())
                .onFalse(robotState.setStateIdle());

        // driverController.b().onTrue(Commands.runOnce(()->
        // intake.setCurrentState(IntakeStates.IntakeDeployed)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public Pose2d getPose2D() {
        return drive.getPose();
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.containers;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;
import static frc.robot.utility.ExtendedMath.withHardDeadzone;
import static frc.robot.utility.ExtendedMath.withContinuousDeadzone;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.PreciserVisionPreciseShootingOI;
import frc.robot.autonomous.Autonomous_ForceIndexBallsCommand;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.autonomous.Autonomous_Megindex;
import frc.robot.autonomous.Autonomous_SingleSensorIndexBallsCommand;
import frc.robot.autonomous.BouncePathCommand;
import frc.robot.autonomous.ExtendedTrajectoryUtilities;
import frc.robot.autonomous.VisionDistanceCalculator;
import frc.robot.autonomous.gsc.MegalacticSearchCommand;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.components.hardware.CameraVisionComponent;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.factory.HardwareIntakeFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.HardwareArmFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.factory.HardwareClimberFactory;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.factory.HardwareIndexerFactory;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.factory.HardwareShooterFactory;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.TrackLoadingCommand;
import frc.robot.subsystems.swerve.kinematic.command.KinematicSwerve_RampedDriveCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.command.v2.OdometricSwerve_FollowDottedTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.vision.CameraVisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utility.OutputRamper;

/**
 * Add your docs here.
 */
public class CleanBoiRobotContainer implements RobotContainer {


    private SendableChooser<Command> autonomousChooser;

    private Joystick driveStick = new Joystick(0), controlStick = new Joystick(1);
    private JoystickButton 
    intakeInButton = new JoystickButton(controlStick, 5),
    intakeOutButton = new JoystickButton(controlStick, 3),
    indexerInButton  = new JoystickButton(controlStick,6),
    indexerOutButton = new JoystickButton(controlStick, 4),
    mainIntakeButton = new JoystickButton(controlStick, 1),
    shootButton = new JoystickButton(controlStick,2),
    climberUpButton = new JoystickButton(controlStick, 9),
    climberDownButton = new JoystickButton(controlStick, 11),
    resetGyroButton = new JoystickButton(driveStick, 5),
    backupIndexerButton = new JoystickButton(controlStick, 8),
    alignToLoadButton = new JoystickButton(driveStick, 3);
    
    private Intake intake = new HardwareIntakeFactory().makeIntake();
    private Arm arm = new HardwareArmFactory().makeArm();
    private OdometricSwerve swerve = new EntropySwerveFactory().makeSwerve();
    private Indexer indexer = new HardwareIndexerFactory().makeIndexer();
    ShuffleboardTab driverTab;

    public CleanBoiRobotContainer() {

        driverTab = Shuffleboard.getTab("Driver Controls");


        configureBasicOverrides();

        // alternateConfigureMainintakeButton();
        configureMainIntakeButton();

        resetGyroButton.whenPressed(() -> swerve.resetPose());
        swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));

        configureSwerve();

        configureAutonomous();

        backupIndexerButton.whileHeld(new Autonomous_Megindex(indexer, intake, 1, 0));
    }


    private void configureAutonomous() {
        autonomousChooser = new SendableChooser<>();

        autonomousChooser.addOption("Megalactic Search", configureMegalacticSearchCommand());

        autonomousChooser.addOption(
            "Dotted Barrel Racing", 
            ExtendedTrajectoryUtilities.addDottedTrajectoryWithShuffleboard(swerve, "Dotted Barrel Racing", "BarrelRacing"));
        SmartDashboard.putData("Selected Auto", autonomousChooser);
        SmartDashboard.putData("Bounce Path", new BouncePathCommand(swerve));
        SmartDashboard.putData("lower arm", new InstantCommand(() -> {arm.setAngle(Math.PI/2);}));
    
    }

 
 

    private void configureSwerve() {
        var xSettings = driverTab.getLayout("X Settings",BuiltInLayouts.kList);
        var ySettings = driverTab.getLayout("Y Settings",BuiltInLayouts.kList);
        var zSettings = driverTab.getLayout("Z Settings",BuiltInLayouts.kList);

        var xSensitivityEntry = xSettings.addPersistent("Sensitivity", 4).getEntry();
        var xDeadzoneEntry = xSettings.addPersistent("Deadzone", 0.2).getEntry();

        var ySensitivityEntry = ySettings.addPersistent("Sensitivity", 4).getEntry();
        var yDeadzoneEntry = ySettings.addPersistent("Deadzone", 0.2).getEntry();

        var zSensitivityEntry = zSettings.addPersistent("Sensitivity", 4).getEntry();
        var zDeadzoneEntry = zSettings.addPersistent("Deadzone", 0.3).getEntry();

        var continuousDrivingEntry = driverTab.addPersistent("Use Continuous Driving", false).getEntry();


        swerve.setDefaultCommand(new RunCommand(() -> {
            double forwardSpeed, leftwardSpeed, counterClockwardSpeed;
            if(continuousDrivingEntry.getBoolean(false)){
                forwardSpeed = withContinuousDeadzone(-driveStick.getY(), yDeadzoneEntry.getDouble(0.2)) * ySensitivityEntry.getDouble(4);
                leftwardSpeed = withContinuousDeadzone(-driveStick.getX(), xDeadzoneEntry.getDouble(0.2)) * xSensitivityEntry.getDouble(4);
                counterClockwardSpeed = withContinuousDeadzone(-driveStick.getZ(), zDeadzoneEntry.getDouble(0.3)) * zSensitivityEntry.getDouble(4);
            }else{
                forwardSpeed = withHardDeadzone(-driveStick.getY(), yDeadzoneEntry.getDouble(0.2)) * ySensitivityEntry.getDouble(4);
                leftwardSpeed = withHardDeadzone(-driveStick.getX(), xDeadzoneEntry.getDouble(0.2)) * xSensitivityEntry.getDouble(4);
                counterClockwardSpeed = withHardDeadzone(-driveStick.getZ(), zDeadzoneEntry.getDouble(0.3)) * zSensitivityEntry.getDouble(4);
            }
            swerve.moveFieldCentric(forwardSpeed, leftwardSpeed, counterClockwardSpeed);
        },swerve));

        
        
   
        driverTab.add("Swerve Pose", new OdometricSwerveDashboardUtility(swerve));
    }
 
    private void configureMainIntakeButton() {
        mainIntakeButton
        .whileHeld(
            new ConditionalCommand(
                new Autonomous_Megindex(indexer, intake, 1, 0.9)
                .alongWith(
                    new FunctionalCommand(
                        () -> arm.setAngle(Math.PI / 2), 
                        () -> {}, 
                        interrupted -> arm.setAngle(0), 
                        () -> false, 
                        arm)), 
                    new FunctionalCommand(
                        () -> {
                            indexer.setSpeed(1);
                            intake.setSpeed(-1);
                            arm.setAngle(Math.PI / 2);
                        }, 
                        () -> {}, 
                        interrupted -> {
                            indexer.setSpeed(0);
                            intake.setSpeed(0);
                            arm.setAngle(0);
                        }, 
                        () -> false, 
                        indexer, arm, intake), 
                () -> false));
    }
    private void alternateConfigureMainintakeButton(){
        mainIntakeButton
        .whileHeld(new Autonomous_SingleSensorIndexBallsCommand(arm, intake, indexer, 0.0, 0.0, 1.0));
        //.whileHeld(new Autonomous_FastIndexBallsCommand(0.001, intake, arm, indexer, 0.0, 0.0, 1.0));
    }

    private void configureBasicOverrides() {
        intakeInButton
        .whenPressed(() -> {
            intake.setSpeed(1);
            arm.setAngle(Math.PI);
        }, intake, arm)
        .whenReleased(() -> {
            intake.setSpeed(0);
            arm.setAngle(0);
        }, intake, arm);

        intakeOutButton
        .whenPressed(() -> {
            intake.setSpeed(-1);
            arm.setAngle(Math.PI);
        }, intake, arm)
        .whenReleased(() -> {
            intake.setSpeed(0);
            arm.setAngle(0);
        }, intake, arm);

        indexerInButton
        .whenPressed(() -> indexer.setSpeed(1), indexer)
        .whenReleased(() -> indexer.setSpeed(0),indexer);

        indexerOutButton
        .whenPressed(() -> indexer.setSpeed(-1), indexer)
        .whenReleased(() -> indexer.setSpeed(0), indexer);
    }

    @Override
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }


    private Command configureMegalacticSearchCommand(){
        var mgsc = new MegalacticSearchCommand(swerve, arm, intake, indexer);
        mgsc.initializeShuffleboardTab("Megalactic Search Command");
        SmartDashboard.putData("Record Points", new InstantCommand(() -> {
            var points = mgsc.getPoints();
            var string = "";
            for(var point : points){
                string += point.toString();
            }
            SmartDashboard.putString("Detected Points",string);
        }));
        return mgsc;
    }
}

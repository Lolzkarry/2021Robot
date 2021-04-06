// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.gsc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Autonomous_Megindex;
import frc.robot.autonomous.ExtendedTrajectoryUtilities;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.v2.OdometricSwerve_FollowDottedTrajectoryCommand;

import static frc.robot.subsystems.swerve.odometric.command.v2.OdometricSwerve_FollowTrajectoryCommand.createBasicController;
import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MegalacticSearchCommand extends SequentialCommandGroup {
  PowerCellFinder finder = new PowerCellFinder();
  ConfigurationIdentifier identifier = new ConfigurationIdentifier();
  ArrayList<Rect> finderResult;
  HashMap<GalacticSearchConfiguration, Trajectory> configToTrajectory;
  boolean error = false;
  OdometricSwerve swerve;

  OdometricSwerve_FollowDottedTrajectoryCommand followCommand;

  //trajectory constraints
  TrajectoryConstraint[] ARedConstraints;
  TrajectoryConstraint[] ABlueConstraints;
  TrajectoryConstraint[] BRedConstraints;
  TrajectoryConstraint[] BBlueConstraints;
  TrajectoryConstraint[][] allConstraints;


  /** Creates a new MegalacticSearchCommand. */
  public MegalacticSearchCommand(OdometricSwerve swerve, Arm arm, Intake intake, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    configToTrajectory = new HashMap<>();
    configToTrajectory.put(GalacticSearchConfiguration.ABlue, tryGetDeployedTrajectory("GSearchABlue"));
    configToTrajectory.put(GalacticSearchConfiguration.ARed, tryGetDeployedTrajectory("GSearchARed"));
    configToTrajectory.put(GalacticSearchConfiguration.BBlue, tryGetDeployedTrajectory("GSearchBBlue"));
    configToTrajectory.put(GalacticSearchConfiguration.BRed, tryGetDeployedTrajectory("GSearchBRed"));


    //trajectory constraints
    double maxBallVelocity = 1;
    double constraintDiameter = 1;

    ARedConstraints = new TrajectoryConstraint[] {
            createSquareConstraint(7.5, 7.5, constraintDiameter/2, maxBallVelocity),//C3
            createSquareConstraint(12.5, 5.0, constraintDiameter/2, maxBallVelocity),//D5
            createSquareConstraint(15.0, 12.5, constraintDiameter/2, maxBallVelocity)//A6
    };
    ABlueConstraints = new TrajectoryConstraint[] {
      createSquareConstraint(15, 2.5, constraintDiameter/2, maxBallVelocity),//E6
      createSquareConstraint(17.5, 10.0, constraintDiameter/2, maxBallVelocity),//B7
      createSquareConstraint(22.5, 7.5, constraintDiameter/2, maxBallVelocity)//C9
    };
    BRedConstraints = new TrajectoryConstraint[] {
      createSquareConstraint(7.5, 10.0, constraintDiameter/2, maxBallVelocity),//B3
      createSquareConstraint(12.5, 5.0, constraintDiameter/2, maxBallVelocity),//D5
      createSquareConstraint(17.5, 10.0, constraintDiameter/2, maxBallVelocity)//B7
    };
    BBlueConstraints = new TrajectoryConstraint[] {
      createSquareConstraint(15.0, 5.0, constraintDiameter/2, maxBallVelocity),//D6
      createSquareConstraint(20.0, 10.0, constraintDiameter/2, maxBallVelocity),//B8
      createSquareConstraint(25.0, 5.0, constraintDiameter/2, maxBallVelocity)//D10
    };
    allConstraints = new TrajectoryConstraint[][]{ARedConstraints,ABlueConstraints,BRedConstraints,BBlueConstraints};

    this.swerve = swerve;
    followCommand = new OdometricSwerve_FollowDottedTrajectoryCommand(swerve, new Trajectory(), createBasicController(1, 1, 4, 4, 3));
    followCommand.getController().setTolerance(new Pose2d(0.1, 0.1, new Rotation2d(0.5)));


      Command intakeCommand = new Autonomous_Megindex(indexer, intake, 1, 0.9);
      Command setArm = new InstantCommand(() ->  arm.setAngle(Math.PI));
      Command resetArm = new InstantCommand(() -> arm.setAngle(0));
      var resetPosition = new InstantCommand(() -> swerve.resetPose(followCommand.getTrajectory().getInitialPose().getTranslation()), swerve);
      Command run = new ConditionalCommand (new InstantCommand(), //if not error
        setArm.alongWith(intakeCommand).alongWith(resetPosition.andThen(followCommand)).andThen(resetArm), //if error
              () -> error);




    addCommands(
      new InstantCommand(() -> {finderResult = finder.findPowerCells(); }), //finds power cells
      new InstantCommand(() -> {followCommand.setTrajectory(getTrajectory(finderResult)); }), //sets the trajectory
        run); //runs the code
  }

  public Trajectory getTrajectory(ArrayList<Rect> finderResults){
    if(finderResult.size() != 3){
      DriverStation.reportError("Robot is not finding 3 cells", false);
      error = true;
      return null;
    }
    else{
      error = false;
      var points = getFinderResult().stream().map((Rect rect) -> new Point(rect.x + rect.width/2, rect.y + rect.height/2)).toArray(Point[]::new);
      var configuration = identifier.identifyConfiguration(points);
      var trajectory = configToTrajectory.get(configuration);
      return trajectory;
    }
  }

  public ArrayList<Rect> getFinderResult() {
      return finderResult;
  }
  public void initializeShuffleboardTab(String tabName){
    var tab = Shuffleboard.getTab(tabName);
    for(var value : GalacticSearchConfiguration.values()){
      addTrajectory(tab, value);
    }
    tab.add("Run Megalactic Search Command", this);
  }
  private void addTrajectory(ShuffleboardTab tab, String layoutName, GalacticSearchConfiguration configuration, Trajectory trajectory){
    var trajectoryLayout = tab.getLayout(layoutName, BuiltInLayouts.kList);
    var maxVelocityEntry = trajectoryLayout.add("Max Velocity Meters", 2.4).getEntry();
    var maxAccelerationEntry = trajectoryLayout.add("Max Acceleration Meters", 0.5).getEntry();
    var maxCentripetalAccelerationEntry = trajectoryLayout.add("Max Centripetal Acceleration Meters", 0.5).getEntry();
    trajectoryLayout.add("Regenerate Trajectory", new InstantCommand(() -> {

        var config =  new TrajectoryConfig(
            maxVelocityEntry.getDouble(2.4), 
            maxAccelerationEntry.getDouble(0.5));
        config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAccelerationEntry.getDouble(0.5)));

        for (TrajectoryConstraint constraint : allConstraints[configuration.ordinal()]){
          config.addConstraint(constraint);
        }

        var newTrajectory = ExtendedTrajectoryUtilities.regenerateTrajectory(trajectory, config);
        configToTrajectory.put(configuration, newTrajectory);
    }, swerve));
  }
  private void addTrajectory(ShuffleboardTab tab, String layoutName, GalacticSearchConfiguration configuration){
    addTrajectory(tab, layoutName, configuration, configToTrajectory.get(configuration));
  }
  private void addTrajectory(ShuffleboardTab tab, GalacticSearchConfiguration configuration){
    addTrajectory(tab, configuration.toString() + " Trajectory Regeneration", configuration);
  }
  public Point[] getPoints(){
    return finder.findPowerCells().stream().map(rect -> new Point(rect.x + rect.width/2, rect.y + rect.height/2)).toArray(Point[]::new);
  }
  static Translation2d convertFieldToTrajectory(double xFeet, double yFeet){
    return new Translation2d(Units.feetToMeters(xFeet), -Units.feetToMeters(15-yFeet)+15);
  }
  static RectangularRegionConstraint createSquareConstraint(Translation2d center, double apothem, TrajectoryConstraint constraint){
    var offset  = new Translation2d(apothem,apothem);
    return new RectangularRegionConstraint(center.minus(offset), center.plus(offset), constraint);
  }
  static RectangularRegionConstraint createSquareConstraint(Translation2d center, double apothem, double velocity){
    return createSquareConstraint(center, apothem, new MaxVelocityConstraint(velocity));
  }
  static RectangularRegionConstraint createSquareConstraint(double xFeet, double yFeet, double apothem, double velocity){
    return createSquareConstraint(convertFieldToTrajectory(xFeet, yFeet), apothem, velocity);
  }
}

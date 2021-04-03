/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.v2.OdometricSwerve_FollowStatesCommand;
import static frc.robot.subsystems.swerve.odometric.command.v2.OdometricSwerve_FollowTrajectoryCommand.createBasicController;

/**
 * Add your docs here.
 */
public class ExtendedTrajectoryUtilities {
    public static Trajectory getDeployedTrajectory(String trajectoryName) throws IOException {

        var trajectoryJSON = "paths/output/"+trajectoryName+".wpilib.json";
        //Stolen pretty much from the example code
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        return trajectory;
    }
    public static Trajectory tryGetDeployedTrajectory(String trajectoryName){
        try{
            return getDeployedTrajectory(trajectoryName);
        }catch(IOException ex){
            DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
            return new Trajectory(null);
        }
    }
    public static Trajectory regenerateTrajectory(Trajectory trajectory, TrajectoryConfig config){
        return TrajectoryGenerator.generateTrajectory(Arrays.asList(trajectory.getStates().stream().map(s -> s.poseMeters).toArray(Pose2d[]::new)), config);
    }
    public static void addTrajectoryWithShuffleboard(OdometricSwerve swerve, String tabName, String trajectoryName){
        
        var tab = Shuffleboard.getTab(tabName);
        var timeAdvanceEntry = tab.addPersistent("Time Advance", 0.02).getEntry();
        var distanceMultiplerEntry = tab.addPersistent("Distance Multiplier", 3).getEntry();

        var trajectory = tryGetDeployedTrajectory(trajectoryName);
        var followCommand = new OdometricSwerve_FollowStatesCommand(swerve, trajectory, createBasicController(1, 1, 1, 4, 1), distance -> {
            return Math.min(timeAdvanceEntry.getDouble(0.02), 1/(distance * distanceMultiplerEntry.getDouble(3)));
        });

        

        var trajectoryLayout = tab.getLayout("Trajectory Regeneration", BuiltInLayouts.kList);
        var maxVelocityEntry = trajectoryLayout.add("Max Velocity Meters", 2.4).getEntry();
        var maxAccelerationEntry = trajectoryLayout.add("Max Acceleration Meters", 0.5).getEntry();
        var maxCentripetalAccelerationEntry = trajectoryLayout.add("Max Centripetal Acceleration Meters", 0.5).getEntry();
        trajectoryLayout.add("Regenerate Trajectory", new InstantCommand(() -> {

            var config =  new TrajectoryConfig(
                maxVelocityEntry.getDouble(2.4), 
                maxAccelerationEntry.getDouble(0.5));
            config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAccelerationEntry.getDouble(0.5)));
            followCommand.setTrajectory(
                ExtendedTrajectoryUtilities.regenerateTrajectory(
                    trajectory, 
                   config));
        }, swerve));

        var controllerLayout = tab.getLayout("Controller Regeneration", BuiltInLayouts.kList);
        var kPxEntry = controllerLayout.add("kPx", 1).getEntry();
        var kPyEntry = controllerLayout.add("kPy", 1).getEntry();
        var kPwEntry = controllerLayout.add("kPw", 1).getEntry();
        var maxRotSpeedEntry = controllerLayout.add("Max Rotational Speed", 4).getEntry();
        var maxRotAccelerationEntry = controllerLayout.add("Max Rotational Acceleration", 1).getEntry();
        controllerLayout.add("Regenerate Controller", new InstantCommand(() -> {
            followCommand.setController(createBasicController(
                kPxEntry.getDouble(1), 
                kPyEntry.getDouble(1), 
                kPwEntry.getDouble(1), 
                maxRotSpeedEntry.getDouble(4), 
                maxRotAccelerationEntry.getDouble(1)));
        }, swerve));

        tab.add("Follow Command Diagnostics", followCommand);
        tab.add("Run Follow Command", new InstantCommand(() -> swerve.resetPose(trajectory.getInitialPose().getTranslation()), swerve).andThen(followCommand));


        
    }
}

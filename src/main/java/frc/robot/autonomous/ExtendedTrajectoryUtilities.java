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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

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
}

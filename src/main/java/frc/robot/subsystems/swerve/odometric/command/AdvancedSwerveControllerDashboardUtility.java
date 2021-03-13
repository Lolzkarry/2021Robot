// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/** Add your docs here. */
public class AdvancedSwerveControllerDashboardUtility implements Sendable {
    private AdvancedSwerveController controller;
    public AdvancedSwerveControllerDashboardUtility(AdvancedSwerveController controller){
        this.controller = controller;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current State X", () -> a().getX(), null);
        builder.addDoubleProperty("Current State Y", () -> a().getY(), null);
    }
    private Pose2d a(){
        if(controller.currentState != null && controller.currentState.poseMeters != null){
            return controller.currentState.poseMeters;
        }else{
            return new Pose2d();
        }
    }
}

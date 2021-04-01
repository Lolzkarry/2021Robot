// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.odometric.command.v2;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.utility.ExtendedMath;

public class OdometricSwerve_FollowStatesCommand extends OdometricSwerve_FollowTrajectoryCommand {

  protected double internalTime = 0.0;
  protected DoubleFunction<Double> timeTransformer;
  public OdometricSwerve_FollowStatesCommand(OdometricSwerve swerve, Trajectory trajectory,
      HolonomicDriveController controller, DoubleFunction<Double> timeTransformer) {
    super(swerve, trajectory, controller);
    this.timeTransformer = timeTransformer;
  }
  @Override
  public void initialize() {
    internalTime = 0.0;
    currentTranslation = trajectory.getInitialPose().getTranslation();
  }
  @Override
  public void execute() {
    if(internalTime > 0.0){
      var distance = ExtendedMath.distance(swerve.getCurrentPose().getTranslation(), currentTranslation);
      internalTime += (timeTransformer != null)? timeTransformer.apply(distance) : 1/distance;
    }
    applyState(trajectory.sample(internalTime));
  }
  @Override
  public boolean isFinished() {
    return internalTime >= trajectory.getTotalTimeSeconds() && controller.atReference();
  }
}

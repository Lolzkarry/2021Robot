// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.odometric.command.v2;

import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;

public class OdometricSwerve_FollowTrajectoryCommand extends CommandBase {

  private OdometricSwerve swerve;
  private Trajectory trajectory;
  private Timer timer;
  private HolonomicDriveController controller;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var state = controller.calculate(swerve.getCurrentPose(), trajectory.sample(timer.get()), new Rotation2d());
    swerve.moveFieldCentric(state.vxMetersPerSecond, state.vyMetersPerSecond, state.omegaRadiansPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds() && controller.atReference();
  }

  public OdometricSwerve_FollowTrajectoryCommand(OdometricSwerve swerve, Trajectory trajectory,
      HolonomicDriveController controller) {
    this.swerve = swerve;
    this.trajectory = trajectory;
    this.controller = controller;
    addRequirements(swerve);
  }

  public Trajectory getTrajectory() {
    return trajectory;
  }

  public void setTrajectory(Trajectory trajectory) {
    this.trajectory = trajectory;
  }

  public HolonomicDriveController getController() {
    return controller;
  }

  public void setController(HolonomicDriveController controller) {
    this.controller = controller;
  }
  public static HolonomicDriveController createController(double kPx, double kPy, double kPw, double maxRotationalSpeed, double maxRotationlAcceleration){
    return new HolonomicDriveController(new PIDController(kPx, 0, 0), new PIDController(kPy, 0, 0), new ProfiledPIDController(kPw, 0, 0, new Constraints(maxRotationalSpeed, maxRotationlAcceleration) ));
  }
}

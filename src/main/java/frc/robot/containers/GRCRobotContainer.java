package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.intake.IntakeOI;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.factory.HardwareIntakeFactory;
import static frc.robot.ControlConstants.xSensitivity;
import static frc.robot.ControlConstants.ySensitivity;
import static frc.robot.ControlConstants.zSensitivity;
import static frc.robot.ControlConstants.xDeadzone;
import static frc.robot.ControlConstants.yDeadzone;
import static frc.robot.ControlConstants.zDeadzone;
import static frc.robot.utility.ExtendedMath.withHardDeadzone;

public class GRCRobotContainer implements RobotContainer, SwerveOI, IntakeOI {
    public OdometricSwerve swerve = EntropySwerveFactory.makeSwerve();
    private ShuffleboardTab driverTab;
    private double intakeSpeed;
    private Joystick
            driveStick = new Joystick(0),
            controlStick = new Joystick(1);


    private JoystickButton
            resetGyroButton = new JoystickButton(driveStick, 5);

    private Intake intake = new HardwareIntakeFactory().makeIntake();



    public GRCRobotContainer() {
        driverTab = Shuffleboard.getTab("Driver Controls");

        resetGyroButton.whenPressed(() -> swerve.resetPose());
        swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));

        configureSwerve();

    }

    //Configuration
    public void configureSwerve(){
        swerve.setDefaultCommand(createHardDeadzoneSwerveCommand());
    }
    private CommandBase createHardDeadzoneSwerveCommand(){
        return new RunCommand(() -> {
            var forwardSpeed = withHardDeadzone(-driveStick.getY(), yDeadzone) * ySensitivity;
            var leftwardSpeed = withHardDeadzone(-driveStick.getX(), xDeadzone) * xSensitivity;
            var counterClockwardSpeed = withHardDeadzone(-driveStick.getZ(), zDeadzone) * zSensitivity;
            swerve.moveFieldCentric(forwardSpeed, leftwardSpeed, counterClockwardSpeed);
        }, swerve);
    }

    //Methods from OIs
    public double getX(){
        return driveStick.getX();
    }
    public double getY(){
        return driveStick.getY();
    }
    public double getZ(){
        return driveStick.getZ();
    }
    public double getIntakeSpeed() {return intakeSpeed;}
}

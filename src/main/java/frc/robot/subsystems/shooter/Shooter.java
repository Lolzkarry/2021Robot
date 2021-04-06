package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.SmartMotorComponent;

public class Shooter extends SubsystemBase {
    private SmartMotorComponent topMotor;
    private SmartMotorComponent bottomMotor;
    private double desiredTopSpeed;
    private double desiredBottomSpeed;
    public Shooter(SmartMotorComponent topMotor, SmartMotorComponent bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }

    public void run(double topSpeed, double bottomSpeed){
        desiredTopSpeed = topSpeed;
        desiredBottomSpeed = bottomSpeed;
        this.topMotor.setAngularVelocity(rotPerMinToRadPerSec(topSpeed));
        this.bottomMotor.setAngularVelocity(rotPerMinToRadPerSec(bottomSpeed));
    }
    public boolean atSpeeds(double threshold){
        return  topAtSpeed(threshold) && bottomAtSpeed(threshold);
    }
    public double radPerSecToRotPerMin(double radPerSec){
        return radPerSec / Math.PI / 2.0 * 60;
    }
    public double rotPerMinToRadPerSec(double rotPerMin){
        return rotPerMin / 60 * 2 * Math.PI;
    }
    public boolean topAtSpeed(double threshold){
        var res = Math.abs(desiredTopSpeed - radPerSecToRotPerMin(topMotor.getAngularVelocity()));
        
        return  res < threshold;
    }
    public boolean bottomAtSpeed(double threshold){
        return Math.abs(desiredBottomSpeed - radPerSecToRotPerMin(bottomMotor.getAngularVelocity())) < threshold;
    }
    public void disableMotors(){
        topMotor.setOutput(0);
        bottomMotor.setOutput(0);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Top Motor", () -> desiredTopSpeed - radPerSecToRotPerMin(topMotor.getAngularVelocity()), null);
        builder.addDoubleProperty("Bottom Motor", () -> desiredBottomSpeed - radPerSecToRotPerMin(bottomMotor.getAngularVelocity()), null);
    }
}


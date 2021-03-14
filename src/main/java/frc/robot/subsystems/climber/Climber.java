package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.SmartMotorComponent;
public class Climber extends SubsystemBase{
    private SmartMotorComponent motor;

    public Climber(SmartMotorComponent motor) {
        this.motor = motor;
    }

    public void setSpeed(double speed) {
        motor.setOutput(speed);
    }

}
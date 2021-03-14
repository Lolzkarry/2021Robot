package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.AngleSetterComponent;

public class Arm extends SubsystemBase {
    AngleSetterComponent motor;
    public final double ARM_ROTS_PER_MOTOR_ROTS = 0.2; //(Math.PI/2)/(3600.0/4096.0);
    public Arm(AngleSetterComponent motor) {
        this.motor = motor;
    }

    public void setAngle(double angle){
        motor.setAngle(angle / ARM_ROTS_PER_MOTOR_ROTS);
    }

}


package frc.robot.subsystems.intake.factory;

import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeMap;

public class HardwareIntakeFactory implements IntakeFactory {
    /**
     *
     */
    ;

    public Intake makeIntake(){
        return new Intake(new SparkMaxComponent(IntakeMap.INTAKE_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless));
    }
}

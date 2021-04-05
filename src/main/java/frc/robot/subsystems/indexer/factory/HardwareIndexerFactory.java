package frc.robot.subsystems.indexer.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.indexer.AnalogPotentiometerBallSensor;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;

public class HardwareIndexerFactory implements IndexerFactory {
    /**
     *
     */
    private static final int BALL_SENSOR_THRESHOLD = 40;
    private static final int CHANNEL = 1;
    private static final double FULL_RANGE = 10, OFFSET = 0, MAIN_THRESHOLD = 10;
    /**
     *
     */
    private static final int INDEXER_MOTOR_PORT = 12;
    public Indexer makeIndexer(){
        return new Indexer(
            new TalonSRXComponent(INDEXER_MOTOR_PORT), 
            new AnalogPotentiometerBallSensor(CHANNEL, FULL_RANGE, OFFSET, MAIN_THRESHOLD), 
            new NetworkTableBallSensor("Sensor1", BALL_SENSOR_THRESHOLD+15), 
            new NetworkTableBallSensor("Sensor2", BALL_SENSOR_THRESHOLD), 
            new NetworkTableBallSensor("Sensor3", BALL_SENSOR_THRESHOLD), 
            new NetworkTableBallSensor("Sensor4", BALL_SENSOR_THRESHOLD), 
            new NetworkTableBallSensor("Sensor5", BALL_SENSOR_THRESHOLD)
        );
    }
}

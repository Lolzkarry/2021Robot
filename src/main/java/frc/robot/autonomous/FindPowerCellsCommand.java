package frc.robot.autonomous;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.vision.RandomGripPipeline;

public class FindPowerCellsCommand extends InstantCommand{
  int width = 320, height = 240;
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    CvSink cvSink = CameraServer.getInstance().getVideo(camera);
    CvSource outputStream1 = CameraServer.getInstance().putVideo("Pipeline Output", width, height), outputStream2 = CameraServer.getInstance().putVideo("Captured Frame", width, height);
    
  Mat source = new Mat();

  RandomGripPipeline pipeline = new RandomGripPipeline();

  public FindPowerCellsCommand(){
    camera.setResolution(width, height);
  }

  @Override
  public void execute() {
    if(cvSink.grabFrame(source, 10) != 0){
        pipeline.process(source);
        outputStream2.putFrame(source);
    }else{
        System.err.println(cvSink.getError());
    }
    if(!pipeline.findContoursOutput().isEmpty()){
        outputStream1.putFrame(pipeline.hsvThresholdOutput());

        SmartDashboard.putStringArray("Contour Outputs", pipeline.getRects().stream().map(r -> r.toString()).toArray(String[]::new));

    }
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem implements Runnable {
  Thread cameraThread;
  
  UsbCamera camera;
  CvSink cvSink;
  CvSource outputStream;

  public Camera() {
    // Run camera code in separate thread to avoid
    // interfering with other code.
    cameraThread = new Thread(this);
    
    // Automatically find camera (on any USB port)
    camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(640, 480);
    
    cvSink = CameraServer.getInstance().getVideo();
    outputStream = CameraServer.getInstance().putVideo("Vision", 640, 480);

    // Trigger run() method inside cameraThread
    cameraThread.start();
  }

  public void run() {
    Mat source = new Mat();
    Mat processingImg = new Mat();

    Scalar hsvLow = new Scalar(50, 50, 40);
    Scalar hsvHigh = new Scalar(90, 150, 245);

    Size blurSize = new Size(10, 10);

    ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    ArrayList<MatOfPoint> bigContours = new ArrayList<MatOfPoint>();
    Mat hierarchy = new Mat();

    Scalar green = new Scalar(0, 255, 0);
    Scalar blue = new Scalar(255, 0, 0);
    Rect bbox;

    int i;

    while (!Thread.interrupted()) {
      // Get frame from camera
      cvSink.grabFrame(source);
      if (source.empty()) {
        System.out.println("Cannot get camera frame");
        continue;
      }

      // TODO: Move image processing to separate method (return contours?)
      // source.copyTo(processingImg);

      // Convert BGR to HSV (easier to process hue than rgb)
      Imgproc.cvtColor(source, processingImg, Imgproc.COLOR_BGR2HSV);

      // Turn green (within range) white, all else black
      Core.inRange(
        processingImg,
        hsvLow,
        hsvHigh,
        processingImg
      );

      // Blur black + white image (remove outlier pixels)
      Imgproc.blur(processingImg, processingImg, blurSize);

      // TODO: Sharpen image by replacing dark gray with black, light gray with white
      /*
      Core.inRange(
        processingImg,
        new Scalar(0, 0, 100), // Cutoff point: mid-range gray
        new Scalar(0, 0, 255), // Pure white
        processingImg
      );
      */

      // Find countours (white objects) in image
      contours.clear();
      Imgproc.findContours(processingImg, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

      // Only include large contours
      bigContours.clear();
      for(i = 0; i < contours.size(); i++) {
        if(Imgproc.contourArea(contours.get(i)) > 500) {
          bigContours.add(contours.get(i));
        }
      }

      // Draw contours to vision output
      Imgproc.drawContours(source, bigContours, -1, green, 2);
      for(i = 0; i < bigContours.size(); i++) {
        bbox = Imgproc.boundingRect(bigContours.get(i));
        Imgproc.rectangle(source, bbox.tl(), bbox.br(), blue, 2);
      }

      // Send vision feed to dashboard
      outputStream.putFrame(source);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

package frc.robot.cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Webcams {

    private ShuffleboardTab tab;

    private UsbCamera leftCamera;
    private UsbCamera rightCamera;

    public Webcams() {
        tab = Shuffleboard.getTab("Webcams");

        leftCamera = CameraServer.startAutomaticCapture(0);
        rightCamera = CameraServer.startAutomaticCapture(1);

        leftCamera.setResolution(320, 240);
        rightCamera.setResolution(320, 240);

        leftCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);
        rightCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);

        resetCameras();
    }

    public void setLeft() {
        rightCamera.setExposureManual(0);
        rightCamera.setFPS(Constants.WebCams.offFPS);
        leftCamera.setExposureAuto();
        leftCamera.setFPS(Constants.WebCams.fastFPS);
    }

    public void setRight() {
        leftCamera.setExposureManual(0);
        leftCamera.setFPS(Constants.WebCams.offFPS);
        rightCamera.setExposureAuto();
        rightCamera.setFPS(Constants.WebCams.fastFPS);
    }

    public void resetCameras() {
        rightCamera.setExposureAuto();
        rightCamera.setFPS(Constants.WebCams.slowFPS);
        leftCamera.setExposureAuto();
        leftCamera.setFPS(Constants.WebCams.slowFPS);
    }
}
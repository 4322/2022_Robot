package frc.robot.cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import frc.robot.Constants;

public class Webcams {

    private UsbCamera leftCamera;
    private UsbCamera rightCamera;

    public Webcams() {
        leftCamera = CameraServer.startAutomaticCapture(0);
        rightCamera = CameraServer.startAutomaticCapture(1);

        leftCamera.setResolution(320, 240);
        rightCamera.setResolution(320, 240);

        leftCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);
        rightCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);

        resetCameras();
    }

    public void setLeft() {
        rightCamera.setExposureManual(5);
        leftCamera.setExposureAuto();
    }

    public void setRight() {
        leftCamera.setExposureManual(5);
        rightCamera.setExposureAuto();
    }

    public void resetCameras() {
        rightCamera.setExposureAuto();
        rightCamera.setFPS(15);
        leftCamera.setExposureAuto();
        leftCamera.setFPS(15);
    }
}
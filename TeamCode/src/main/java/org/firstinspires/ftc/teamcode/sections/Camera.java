package org.firstinspires.ftc.teamcode.sections;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    OpenCvWebcam cam;
    String color;
    public Camera.CameraDetectPipeline pipeline;
    public static class Params {
        public boolean displayColorsOnly = true;

        public int objTargetX = 150;
        public int objTargetY = 110;
    }
    Params PARAMS = new Params();
    public Camera(HardwareMap hardwareMap,String team){
        color = team;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new Camera.CameraDetectPipeline();

        // Open async and start streaming inside opened callback
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


                cam.setPipeline(pipeline);

            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
    class CameraDetectPipeline extends OpenCvPipeline
    {

        RotatedRect target;
        // switch use HSV values
        // Divide H by 2
        Scalar blueL = new Scalar(200/2, .6*255, .1*255);
        Scalar blueU = new Scalar(260/2, .85*255, 1.00*255);

        //red wraps around the hue spectrum so we need 2 sets of bounds

        Scalar redL1 = new Scalar(345/2, .6*255, .1*255);
        Scalar redU1 = new Scalar(359/2, 1.*255, 1.00*255);
        Scalar redL2 = new Scalar(0, .6*255, .1*255);
        Scalar redU2 = new Scalar(7.5, 1.*255, 1.00*255);

        Scalar yellowL = new Scalar(15, 150, 150);
        Scalar yellowU = new Scalar(35, 255, 255);
        Boolean trackYellow = true;

        //inches
        double width = 1.5;
        double length = 3.5;
        double wlratio = 1.5/3.5;

        @Override
        public Mat processFrame(Mat input)
        {
            Scalar lowerBound;
            Scalar upperBound;

            List<MatOfPoint> contours = new ArrayList<>();
            List<MatOfPoint> contoursYellow = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.cvtColor(input, hierarchy, Imgproc.COLOR_RGB2HSV);

//
//        // Create a mask for blue color
            Mat blueMask = new Mat();
            Core.inRange(hierarchy, blueL, blueU, blueMask);

            Mat redMask = new Mat();
            Mat redMask2 = new Mat();
            Core.inRange(hierarchy, redL1, redU1, redMask);
            Core.inRange(hierarchy, redL2, redU2, redMask2);
            Core.bitwise_or(redMask,redMask2,redMask);

            Mat yellowMask = new Mat();
            Core.inRange(hierarchy, yellowL, yellowU, yellowMask);
            Mat selectedColorMask = new Mat();
            if(color.equals("red")){
                selectedColorMask = redMask;
            }else selectedColorMask = blueMask;
            Mat output = new Mat();
            Mat objOnly = new Mat();
            if(PARAMS.displayColorsOnly) {
                if(trackYellow){
                    Mat dualMask = new Mat();
                    Core.bitwise_or(yellowMask, selectedColorMask, dualMask);
                    Core.bitwise_and(input, input, output, dualMask);
                }else{
                    Core.bitwise_and(input, selectedColorMask, output);
                }
            }else{
                output = input;
            }

//      Insert Code for Rectangle detections here
            List<RotatedRect> rectList = new ArrayList<>();
            rectList = getRect(selectedColorMask, hierarchy,rectList);
            if(trackYellow){
                rectList = getRect(selectedColorMask, hierarchy,rectList);
                output = addRect(output, rectList);
            }else {
                output = addRect(output, rectList);
            }
            target = getClosest(rectList);
            return output;
        }
        private List<RotatedRect> getRect(Mat mask, Mat hierarchy, List<RotatedRect> minRect){
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > 100) { // Adjust this threshold as needed
                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    double rat = Math.min(rect.size.height, rect.size.width)/Math.max(rect.size.height, rect.size.width);
                    if(Math.abs(rat-wlratio)/wlratio<0.175){
                        minRect.add(rect);
                    }

                }
            }
            return minRect;
        }

        private Mat addRect(Mat input, List<RotatedRect> minRect){
            // Draw rectangles and get data
            for (int i = 0; i < minRect.size(); i++) {
                Point[] rectPoints = new Point[4];
                minRect.get(i).points(rectPoints);
                for (int j = 0; j < 4; j++) {
                    Imgproc.line(input, rectPoints[j], rectPoints[(j+1) % 4], new Scalar(0, 255, 0), 4);
                }
            }
            return input;
        }
        public RotatedRect getClosest(List<RotatedRect> list){
            RotatedRect lowest;
            RotatedRect temp;
            double lowestLength;
            double tempLength;

            if(list.size()>0){
                lowest = list.get(0);
                lowestLength = Math.sqrt(Math.pow(lowest.center.x,2)+Math.pow(lowest.center.y,2));
                for(int i = 1; i < list.size();i++){
                    temp = list.get(i);
                    tempLength = Math.sqrt(Math.pow(temp.center.x,2)+Math.pow(temp.center.y,2));
                    if(tempLength<lowestLength){
                        lowestLength = tempLength;
                        lowest = temp;
                    }
                }
                return lowest;
            }else return null;
        }
        public double getRotation() {
            try {
                if (!target.equals(null)) {
                    Point[] rectPoints = new Point[4];
                    target.points(rectPoints);
                    Point[] topPoints = new Point[2];
                    topPoints[0] = rectPoints[0];
                    topPoints[1] = rectPoints[1];
                    double distance = Math.sqrt(Math.pow(topPoints[0].x - topPoints[1].x, 2) + Math.pow(topPoints[0].y - topPoints[1].y, 2));
                    for (int i = 1; i < 4; i++) {
                        if (distance > Math.sqrt(Math.pow(topPoints[0].x - rectPoints[i].x, 2) + Math.pow(topPoints[0].y - rectPoints[i].y, 2))) {
                            topPoints[1] = rectPoints[i];
                            distance = Math.sqrt(Math.pow(topPoints[0].x - rectPoints[i].x, 2) + Math.pow(topPoints[0].y - rectPoints[i].y, 2));
                        }
                    }
                    double avgX = (topPoints[0].x + topPoints[1].x) / 2;
                    double avgY = (topPoints[0].y + topPoints[1].y) / 2;

                    double angle = 180*(Math.atan((target.center.x - avgX)/(target.center.y - avgY))/Math.PI);
                    return angle;
                }
            }catch(Exception e){
                return 0;
            }
            return 0;
        }


        public double getX() {
            try{
                return target.center.x;
            }catch(Exception e){
                return -1;
            }

        }

        public double getY() {
            try{
                return target.center.y;
            }catch(Exception e) {
                return -1;
            }
        }
    }

    public double angleCor(double ang){
        if(Math.abs(ang)>Math.abs(ang+Math.PI*2)){
            ang = ang+Math.PI*2;
        }else if(Math.abs(ang)>Math.abs(ang-Math.PI*2)){
            ang = ang-Math.PI*2;
        }
        return ang;
    }

    public double getObjX(){
        return pipeline.getX();
    }
    public double getObjY(){
        return pipeline.getY();
    }
    public double getObjRot(){
        return pipeline.getRotation();
    }
    public double getTargetX(){return PARAMS.objTargetX;}
    public double getTargetY(){return PARAMS.objTargetY;}
    public void pauseCamera(){
        cam.pauseViewport();
    }
    public void resumeCamera(){
        cam.resumeViewport();
    }
    public void setTeamRed(){
        color = "red";
    }
    public void setTeamBlue(){
        color = "blue";
    }
    public void yellowTrackingOn(){
        pipeline.trackYellow = true;
    }
    public void yellowTrackingOff(){
        pipeline.trackYellow = false;
    }

}

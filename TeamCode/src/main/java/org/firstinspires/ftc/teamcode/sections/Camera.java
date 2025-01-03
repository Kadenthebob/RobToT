package org.firstinspires.ftc.teamcode.sections;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    public OpenCvWebcam cam;
    public Camera.CameraDetectPipeline pipeline;
    @Config
    public static class CamParams {
        public static boolean DISPLAY_OBJ_ONLY = false;
        public static boolean TRACK_YELLOW = true;
        public static boolean TRACK_TARGET_COLOR = true;

        public static int OBJ_TARGET_X = 226;
        public static int OBJ_TARGET_Y = 337;
        public static int OBJ_MIN_SIZE = 1500;

        public static int CAM_HEIGHT = 360;
        public static int CAM_WIDTH = 640;
        public static boolean redTeam = true;

        public static double SURROUNDED_DISTANCE = 3;//in inches

        public static double FOCUS_LENGTH = 0;

    }
    @Config
    public static class WarpParam {
        // Source points (as fractions of image width/height)
        public static double SRC_TL_X = 0.3;   // Moved closer to edge
        public static double SRC_TL_Y = 0.0;
        public static double SRC_TR_X = 0.7;   // Moved closer to edge
        public static double SRC_TR_Y = 0.0;
        public static double SRC_BL_X = 0.0;
        public static double SRC_BL_Y = 1.0;
        public static double SRC_BR_X = 1.0;
        public static double SRC_BR_Y = 1.0;

        // Wider destination points
        public static double DST_TL_X = 0.3;   // Wider output
        public static double DST_TL_Y = 0.0;
        public static double DST_TR_X = 0.7;   // Wider output
        public static double DST_TR_Y = 0.0;
        public static double DST_BL_X = 0.3;
        public static double DST_BL_Y = 1.0;
        public static double DST_BR_X = 0.7;
        public static double DST_BR_Y = 1.0;  // Bottom-right Y
    }
    @Config
    public static class CamYellowDetect {
        public static int L_H = 15;
        public static int L_S = 25;
        public static int L_V = 230;

        public static int U_H = 35;
        public static int U_S = 255;
        public static int U_V = 255;

        public static Mat getMask(Mat hierarchy){
            Mat yellowMask = new Mat();
            Core.inRange(hierarchy, new Scalar(L_H, L_S, L_V), new Scalar(U_H, U_S, U_V), yellowMask);
            return yellowMask;
        }
    }

    @Config
    public static class CamBlueDetect {
        public static int L_H = 100;
        public static int L_S = 100;
        public static int L_V = 150;

        public static int U_H = 130;
        public static int U_S = 255;
        public static int U_V = 255;

        public static Mat getMask(Mat hierarchy){
            Mat blueMask = new Mat();
            Core.inRange(hierarchy, new Scalar(L_H, L_S, L_V), new Scalar(U_H, U_S, U_V), blueMask);
            return blueMask;
        }
    }

    @Config
    public static class CamRedDetect {
        public static int FL_H = 172;
        public static int FU_H = 179;

        public static int SL_H = 0;
        public static int SU_H = 8;

        public static int L_S = 100;
        public static int L_V = 210;


        public static int U_S = 255;
        public static int U_V = 255;

        public static Mat getMask(Mat hierarchy){
            Mat redMask = new Mat();
            Mat redMask2 = new Mat();
            Core.inRange(hierarchy, new Scalar(FL_H, L_S, L_V), new Scalar(FU_H, U_S, U_V), redMask);
            Core.inRange(hierarchy, new Scalar(SL_H, L_S, L_V), new Scalar(SU_H, U_S, U_V), redMask2);
            Core.bitwise_or(redMask,redMask2,redMask);
            redMask2.release();
            return redMask;
        }
    }

    WarpParam WarpParams = new WarpParam();
    CamParams PARAMS = new CamParams();
    CamYellowDetect yellowDetect = new CamYellowDetect();
    CamRedDetect redDetect = new CamRedDetect();
    CamBlueDetect blueDetect = new CamBlueDetect();


    public CameraDetectPipeline getPipeline(){
        return pipeline;
    }
    public Camera(HardwareMap hardwareMap,Boolean redTeam){



        CamParams.redTeam = redTeam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new Camera.CameraDetectPipeline();

        // Open async and start streaming inside opened callback
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(PARAMS.CAM_WIDTH, PARAMS.CAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
                cam.getFocusControl().setMode(FocusControl.Mode.Fixed);
                cam.getFocusControl().setFocusLength(CamParams.FOCUS_LENGTH);

                cam.setPipeline(pipeline);

            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
    class CameraDetectPipeline extends OpenCvPipeline
    {
        Mat output = new Mat();
        Mat TotalMask = new Mat();
        Mat hierarchy = new Mat();
        Mat selectedColorMask;
        Mat unselectedColorMask;
        Mat yellowMask;
        public RotatedRect target;
        public RotatedRect selected;
        //inches
        double wlratio = 1.5/3.5;// width/length of object

        public boolean isTargetSurrounded = false;

        @Override
        public Mat processFrame(Mat input)
        {
            output = new Mat();
            if(!PARAMS.TRACK_YELLOW&&!PARAMS.TRACK_TARGET_COLOR){
                return input;
            }
            input = warpPerspective(input);

            Imgproc.cvtColor(input, hierarchy, Imgproc.COLOR_RGB2HSV);

            yellowMask = yellowDetect.getMask(hierarchy);
            if(CamParams.redTeam){
                selectedColorMask = redDetect.getMask(hierarchy);
                unselectedColorMask = blueDetect.getMask(hierarchy);
            }else{
                selectedColorMask = blueDetect.getMask(hierarchy);
                unselectedColorMask = redDetect.getMask(hierarchy);
            }


            if(PARAMS.DISPLAY_OBJ_ONLY) {

                if(PARAMS.TRACK_TARGET_COLOR&&PARAMS.TRACK_YELLOW){
                    Core.bitwise_or(yellowMask,selectedColorMask,TotalMask);
                }else if(PARAMS.TRACK_YELLOW){
                    TotalMask = yellowMask;
                }
                else if(PARAMS.TRACK_TARGET_COLOR){
                    TotalMask = selectedColorMask;
                }
                Core.bitwise_and(input, input, output, TotalMask);
            }else{
                output = input;
            }

//      Insert Code for Rectangle detections here
            List<RotatedRect> rectList = new ArrayList<>();
            List<RotatedRect> rectListUnselec = new ArrayList<>();
            if(PARAMS.TRACK_TARGET_COLOR){
                rectList = getRect(selectedColorMask, hierarchy,rectList);
            }
            if(PARAMS.TRACK_YELLOW) {
                rectList = getRect(yellowMask, hierarchy, rectList);
            }

            rectListUnselec = getRect(unselectedColorMask,hierarchy,rectListUnselec);

            output = addRect(output, rectList, new Scalar(0, 255, 0));
            target = getClosest(rectList);
            output = addRect(output,rectListUnselec, new Scalar(255,0,255));
            rectListUnselec.addAll(rectList);
            isTargetSurrounded = setTargetSurrounded(rectListUnselec);
            try {
                Imgproc.drawMarker(output, target.center, new Scalar(255, 0, 0));
            } catch(Exception e){

            }
            return output;
        }
        private List<RotatedRect> getRect(Mat mask, Mat hierarchy, List<RotatedRect> minRect){
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > CamParams.OBJ_MIN_SIZE) { // Adjust this threshold as needed
                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                    double rat = Math.min(rect.size.height, rect.size.width)/Math.max(rect.size.height, rect.size.width);
                    if(Math.abs(rat-wlratio)/wlratio<.4){
                        minRect.add(rect);
                    }


                }
            }
            return minRect;
        }

        private Mat addRect(Mat input, List<RotatedRect> minRect, Scalar color){

            // Draw rectangles and get data
            for (int i = 0; i < minRect.size(); i++) {
                Point[] rectPoints = new Point[4];
                minRect.get(i).points(rectPoints);
                Imgproc.drawMarker(input,new Point(minRect.get(i).center.x,minRect.get(i).center.y),new Scalar(0,255,0));
                for (int j = 0; j < 4; j++) {
                    Imgproc.line(input, rectPoints[j], rectPoints[(j+1) % 4], color, 4);
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
                lowestLength = Math.sqrt(Math.pow(CamParams.OBJ_TARGET_X-lowest.center.x,2)+Math.pow(CamParams.OBJ_TARGET_Y-lowest.center.y,2));
                for(int i = 1; i < list.size();i++){
                    temp = list.get(i);
                    tempLength = Math.sqrt(Math.pow(CamParams.OBJ_TARGET_X-temp.center.x,2)+Math.pow(CamParams.OBJ_TARGET_Y-temp.center.y,2));
                    if(tempLength<lowestLength){
                        lowestLength = tempLength;
                        lowest = temp;
                    }
                }
                return lowest;
            }else return null;
        }

        public boolean setTargetSurrounded(List<RotatedRect> list){
            RotatedRect lowest;
            RotatedRect temp;
            double lowestLength;
            double tempLength;
            try {
                if (list.size() > 1) {
                    for (RotatedRect rect : list) {
                        if (!rect.center.equals(target.center)) {
                            temp = rect;
                            tempLength = Math.sqrt(Math.pow(target.center.x - temp.center.x, 2) + Math.pow(target.center.y - temp.center.y, 2));

                            if (tempLength * getCam2Inch() < CamParams.SURROUNDED_DISTANCE) {
                                return true;
                            }
                        }
                    }
                    return false;
                } else return false;
            } catch (Exception e){
                return isTargetSurrounded;
            }
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

        public double calcRotation(RotatedRect rect) {
            try {
                if (!rect.equals(null)) {
                    Point[] rectPoints = new Point[4];
                    rect.points(rectPoints);
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


        Mat warpPerspective(Mat input) {
            int width = input.width();
            int height = input.height();

            // Create source points
            Point[] srcPoints = new Point[4];
            srcPoints[0] = new Point(width * WarpParams.SRC_TL_X, height * WarpParams.SRC_TL_Y);   // Top-left
            srcPoints[1] = new Point(width * WarpParams.SRC_TR_X, height * WarpParams.SRC_TR_Y);   // Top-right
            srcPoints[2] = new Point(width * WarpParams.SRC_BL_X, height * WarpParams.SRC_BL_Y);   // Bottom-left
            srcPoints[3] = new Point(width * WarpParams.SRC_BR_X, height * WarpParams.SRC_BR_Y);   // Bottom-right

            // Create destination points
            Point[] dstPoints = new Point[4];
            dstPoints[0] = new Point(width * WarpParams.DST_TL_X, height * WarpParams.DST_TL_Y);
            dstPoints[1] = new Point(width * WarpParams.DST_TR_X, height * WarpParams.DST_TR_Y);
            dstPoints[2] = new Point(width * WarpParams.DST_BL_X, height * WarpParams.DST_BL_Y);
            dstPoints[3] = new Point(width * WarpParams.DST_BR_X, height * WarpParams.DST_BR_Y);

            // Convert points to the format needed by OpenCV
            MatOfPoint2f src = new MatOfPoint2f(srcPoints);
            MatOfPoint2f dst = new MatOfPoint2f(dstPoints);
            // Get the transformation matrix
            Mat perspectiveTransform = Imgproc.getPerspectiveTransform(src, dst);

            // Create output matrix and apply the transformation
            Imgproc.warpPerspective(input, input, perspectiveTransform, new Size(width, height));

            return input;
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

        public double getWidth() {
            try{
                return target.size.width;
            }catch(Exception e) {
                return -1;
            }
        }
        public double getHeight() {
            try{
                return target.size.height;
            }catch(Exception e) {
                return -1;
            }
        }

        public RotatedRect getTarget() {
            return target;
        }
    }
    public static class selected{
        public static RotatedRect rect;
        public static boolean surrounded = false;
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
    public RotatedRect getTargetObj(){
        return pipeline.getTarget();
    }
    public double getTargetX(){return PARAMS.OBJ_TARGET_X;}
    public double getTargetY(){return PARAMS.OBJ_TARGET_Y;}
    public void pauseCamera(){
        cam.pauseViewport();
    }
    public void resumeCamera(){
        cam.resumeViewport();
    }
    public void setTeamRed(){
        CamParams.redTeam = true;
    }
    public void setTeamBlue(){
        CamParams.redTeam = false;
    }
    public void yellowTrackingOn(){
        PARAMS.TRACK_YELLOW = true;
    }
    public void yellowTrackingOff(){
        PARAMS.TRACK_YELLOW = false;
    }
    public double getWidth(){
        return pipeline.getWidth();
    }
    public double getHeight(){
        return pipeline.getHeight();
    }
    public void resetTarget(){
        pipeline.target = null;
    }
    public boolean getSurrounded(){
        return pipeline.isTargetSurrounded;
    }
    public double getCam2Inch(){ return 1.5 / Math.min(getWidth(), getHeight());}

    public void setSelected(){
        selected.rect = getTargetObj();
        selected.surrounded = getSurrounded();
    }
    public RotatedRect getSelectedRect(){
        return selected.rect;
    }
    public boolean getSelectedSurrounded(){
        return selected.surrounded;
    }
}

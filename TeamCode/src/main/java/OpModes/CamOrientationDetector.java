package OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.opencv.core.Point;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;

@Autonomous(name = "CamOrientation", group = "Auto Testing")
public class CamOrientationDetector extends ColorLocator {

    private double lastValidAngle = 90; // Default straight ahead

    // Define source points (in your image, identify these manually or using detection)
    private Point[] sourcePoints = {
            new Point(210, 90),   // Top-left of the object
            new Point(210, 36), // Bottom-left
            new Point(315, 36), // Bottom-right
            new Point(315, 90)  // Top-right
    };

    // Define destination points (top-down perspective)
    private Point[] destinationPoints = {
            new Point(100, 150),   // Adjust based on your desired top-down view
            new Point(100, 100),
            new Point(250, 100),
            new Point(250, 150)
    };

    /**
     * Perform perspective correction (homography)
     * @param inputImage The original image
     * @return The corrected (top-down view) image
     */
    public Mat applyHomography(Mat inputImage) {
        // Convert points to MatOfPoint2f (required by findHomography)
        MatOfPoint2f srcMat = new MatOfPoint2f(sourcePoints);
        MatOfPoint2f dstMat = new MatOfPoint2f(destinationPoints);

        // Calculate the homography matrix
        Mat homographyMatrix = Calib3d.findHomography(srcMat, dstMat);

        // Apply the homography matrix to get the warped image
        Mat warpedImage = new Mat();
        Imgproc.warpPerspective(inputImage, warpedImage, homographyMatrix, inputImage.size());
        return warpedImage;
    }

    /**
     * Gets orientation from OpenCV RotatedRect
     * @param box The RotatedRect from vision processing
     * @return angle in degrees normalized to 0-180
     */
    public double getOrientationFromCorners(Point[] corners) {
        if (corners == null || corners.length != 4) {
            return lastValidAngle;
        }

        double check = distance(corners[0], corners[3]);
        double check1 = distance(corners[2], corners[3]);
        if (check > check1) {
            Point temp = corners[0];
            corners[0] = corners[3];
            corners[3] = temp;
        }

        // Calculate lengths of the edges
        double edge1Length = distance(corners[0], corners[1]);
        double edge2Length = distance(corners[1], corners[2]);

        // Use the longer edge for orientation
        double angle;
        if (edge1Length > edge2Length) {
            angle = calculateAngle(corners[0], corners[1]);
        } else {
            angle = calculateAngle(corners[1], corners[2]);
        }

        angle = (angle + 90) % 180;
        lastValidAngle = angle;
        return angle;
    }

    private double distance(Point p1, Point p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double calculateAngle(Point p1, Point p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double angle = Math.toDegrees(Math.atan2(dy, dx));
        return (angle + 360) % 180;
    }

    // Example of applying homography to an image during processing
    public void processImage(Mat inputImage) {
        // Apply the homography correction to the input image
        Mat correctedImage = applyHomography(inputImage);

        // Now you can pass correctedImage to your color locator or other processing
        // For example, detecting contours in the corrected image:
        detectColorBlob(correctedImage);
    }

    /**
     * Example method for color blob detection (your existing method could go here)
     */
    private void detectColorBlob(Mat inputImage) {
        // Process the image (detect blobs, etc.)
        // Example: Using the corrected image to locate color blobs or objects.
    }
}

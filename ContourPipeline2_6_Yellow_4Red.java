package org.firstinspires.ftc.teamcode;

//import android.location.Location;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline2_6_Yellow_4Red extends OpenCvPipeline {


    // These constants are used to threshold the image for Yellow pixels
    public static Scalar scalarLowerYCrCb = new Scalar(0, 182.8, 0); //specifically made for Centerstage with a vase appearance Team Prop
    public static Scalar scalarUpperYCrCb = new Scalar(198.3, 255.0, 109.1); //Y Cr Cb  //specifically made for Centerstage with a vase Team Prop 1/26/24

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    public String position = "RIGHT";


    public ContourPipeline2_6_Yellow_4Red(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    private double maxArea = 0;
    private boolean first = false;

    private Mat ycrcbMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();
    Mat thresholdMat = new Mat();

    private Rect maxRect = new Rect(600, 1, 1, 1);
    private Rect rect = new Rect(600, 1, 1, 1);

   /* final int LEFT = 150;
    final int CENTER = 200;
    final int RIGHT = 255; */

    //private Location location;

    public final Scalar BLUE = new Scalar(0, 0, 255);
    public final Scalar GREEN = new Scalar(0, 255, 0);

    // Constants for the 3 regions of interest
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(360, 310);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(860, 280);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1300, 280);
    static final int REGION_WIDTH = 290;
    static final int REGION_HEIGHT = 390;

    // Create the Point and Rect constructs for the regions of interest
    static final Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
   static final Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    static final Rect region1 = new Rect(region1_pointA, region1_pointB);

    static final Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    static final Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    static final Rect region2 = new Rect(region2_pointA, region2_pointB);

    static final Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    static final Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    static final Rect region3 = new Rect(region3_pointA, region3_pointB);


    //Working variables
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;

    private Telemetry telemetry;

    private Mat workingMatrix = new Mat();

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

    }

    List<MatOfPoint> contours = new ArrayList<>();


    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        // Convert the image to a YCrCb format where channel 0 = Y (B/W intensity), channel 1 = Cr hue, channel 2 = Cb hue
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        // Threshold the image so pixels that are in the range of scalarLowerYCrCb and scalarUpperYCrCb show up as white
        // and all other pixels show up as black. For this code, scalarLowerYCrCb/upperYCrCb are set to filter for Yellow
        // so all yellow pixels in this range show up as white.
        Core.inRange(workingMatrix, scalarLowerYCrCb,scalarUpperYCrCb, workingMatrix);

        // Now we have the thresholded image and will extract out a submatrix of the 3 regions of interest from this image
        Mat matLeft = workingMatrix.submat(region1);
        Mat matCenter = workingMatrix.submat(region2);
        Mat matRight = workingMatrix.submat(region3);

        // Just draw the boxes (for simulation) for illustration of the regions that we are extracting from the image
        Imgproc.rectangle(workingMatrix, region1, new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, region2, new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, region3, new Scalar(0,255,0));

        // We add up all the pixels of each region that we extracted from the original image
        double leftTotal = Core.sumElems(matLeft).val[0];
        double centerTotal = Core.sumElems(matCenter).val[0];
        double rightTotal = Core.sumElems(matRight).val[0];

        telemetry.addData("leftTotal", leftTotal);
        telemetry.addData("centerTotal", centerTotal);
        telemetry.addData("rightTotal", rightTotal);

        // Now we figure out which region had the largest yellow pixel count
        // and that most likely contains the shipping element so we can infer it's position.
        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                //left is TSE
                position = "LEFT";
            } else {
                //right is TSE
                position = "RIGHT";
            }
        } else {
            if (centerTotal > rightTotal) {
                //center is TSE
                position = "CENTER";
            } else {
                //right is TSE
                position = "RIGHT";
            }
        }
        telemetry.addData("[Pattern]", position);
        telemetry.update();
        return workingMatrix;

    }

    public String getLocation() {
        return position;
    }

}
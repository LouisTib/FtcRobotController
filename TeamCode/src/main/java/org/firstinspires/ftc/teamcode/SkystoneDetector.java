package org.firstinspires.ftc.teamcode;



import org.opencv.core.Rect;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    //global varibles for the location of Color value found
    public enum location {LEFT, RIGHT, NOT_FOUND}
    private location location;

    //Rectangle corner positions
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;


    //Telemetry establishment
    public SkystoneDetector(Telemetry t) {
        telemetry = t;
    }

    //Color conversion view
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        //converting percentage based on a threshold deciding if the color is actually there.
        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        //initializing the boolean variables to the global variables with telemetry
        if (stoneLeft && stoneRight) {
            location = location.NOT_FOUND;
            telemetry.addData("Skystone Location", "Not Found");
        } else if(stoneLeft){
            location = location.RIGHT;
            telemetry.addData("Skystone Location", "Right");
        }else {
            location =location.LEFT;
            telemetry.addData("Skystone Location", "Left");
        }
        telemetry.update();

        //end of the video is at 11:23- FTC easy open CTV tutorial
    }
}




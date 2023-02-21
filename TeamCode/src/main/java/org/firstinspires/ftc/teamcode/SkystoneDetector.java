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
    Mat tempMat = new Mat();

    //global variables for the location of Color value found
    public enum Location {
        LEFT, CENTER, RIGHT, NOT_FOUND
    }

    private Location location;

    //Rectangle corner positions
    static final Rect LEFT_ROI = new Rect(
            new Point(55, 75),
            new Point(115, 115));
    static final Rect CENTER_ROI = new Rect(
            new Point(135, 75),
            new Point(195, 115));
    static final Rect RIGHT_ROI = new Rect(
            new Point(215, 75),
            new Point(275, 115));
    static double PERCENT_COLOR_THRESHOLD = 0.4;


    //Telemetry establishment
    public SkystoneDetector(Telemetry t) {
        telemetry = t;
    }

    private final Scalar[][] colors = {
            // Low, High
            {new Scalar(171, 235, 52), new Scalar(107, 235, 52)}, // Lime
            {new Scalar(125, 106, 21), new Scalar(79, 67, 13)}, // Brown
            //{new Scalar(230, 22, 250), new Scalar(250, 22, 180)}, // Pink
            {new Scalar(0, 0, 0), new Scalar(255, 255, 255)}
    };
    //Color conversion view
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Location[] locations = { Location.NOT_FOUND, Location.NOT_FOUND, Location.NOT_FOUND };
        for (int i = 0; i < colors.length; i++) {
            Core.inRange(mat, colors[i][0], colors[i][1], tempMat);
            Mat left = tempMat.submat(LEFT_ROI);
            Mat center = tempMat.submat(CENTER_ROI);
            Mat right = tempMat.submat(RIGHT_ROI);

            telemetry.addData("test", left.get(0,0));
            double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
            double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

            left.release();
            center.release();
            right.release();

            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            telemetry.addData("center percentage", Math.round(centerValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

            if ((leftValue > centerValue) && (leftValue > rightValue)) {
                locations[i] = Location.LEFT;
            } else if ((rightValue > leftValue) && (rightValue > centerValue)) {
                locations[i] = Location.CENTER;
            } else if ((centerValue > leftValue) && (centerValue > rightValue)) {
                locations[i] = Location.RIGHT;
            }

            telemetry.update();
        }

        telemetry.addData("Lime Location", locations[1]);
        telemetry.addData("Brown Location", locations[0]);
        telemetry.addData("Pink Location", locations[2]);


        return tempMat;
    }

    public Location getLocation() {
        return location;
    }
}




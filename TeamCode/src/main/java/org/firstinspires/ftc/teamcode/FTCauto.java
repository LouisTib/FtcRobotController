package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name = "FTC_Auto", group = "Concept")


public class FTCauto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor rightDriveBack = null;
    private DcMotor leftDriveBack = null;

    private DcMotorEx lift = null;
    private Servo claw = null;

    TestPipeline pipeline = new TestPipeline();



    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        initWebcam();
        waitForStart();
        initDrive();
        telemetry.update();

    }

    private void runLiftToPosition(int pos) {
        telemetry.log().add("Running lift to position: " + pos);

        lift.setPower(0.6); // 60% power
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the lift to arrive at it's target position
        while (lift.isBusy()) {
            telemetry.addData("Lift position:", pos);
            telemetry.addData("Lift amps", lift.getCurrent(AMPS));
            telemetry.update();

            // Overcurrent protection
            if (lift.getCurrent(AMPS) > 7) {
                telemetry.log().add("OVERCURRENT! Aborting!");
                telemetry.log().add("AMPS: " + lift.getCurrent(AMPS));
                lift.setPower(0);
                throw new RuntimeException("Overcurrent");
                //return;
            }
        }
        telemetry.log().add("Done running lift to position: " + pos);
    }

    private void setPowerAll(double a) {
        leftDriveFront.setPower(a);
        rightDriveFront.setPower(a);
        leftDriveBack.setPower(a);
        rightDriveBack.setPower(a);
    }

    private void strafeLeft() {
        rightDriveBack.setPower(-0.5);
        leftDriveBack.setPower(0.5);
        rightDriveFront.setPower(0.5);
        leftDriveFront.setPower(-0.5);

    }

    private void strafeRight() {
        rightDriveBack.setPower(0.5);
        leftDriveBack.setPower(-0.5);
        rightDriveFront.setPower(-0.5);
        leftDriveFront.setPower(0.5);

    }

    private double calculateInchesTraveled(double startingTicks) {
        double TICKS_PER_REV = 537.7; // GoBuilda 5203 312RPM motor
        double WHEEL_DIAMETER = 4; // Self-explanatory
        double INCHES_PER_REV = WHEEL_DIAMETER * Math.PI;

        double ticks_traveled = Math.abs(leftDriveFront.getCurrentPosition() - startingTicks);
        double revolutions = ticks_traveled * TICKS_PER_REV;
        return revolutions * INCHES_PER_REV;
    }

    private void initDrive() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDriveFront = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        leftDriveBack = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);

        if (opModeIsActive()) {

            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Lift Pos", lift.getCurrentPosition());

        }


        claw.setPosition(0.4);
        double startingPosition = leftDriveFront.getCurrentPosition();
        while (calculateInchesTraveled(startingPosition) < 10) {
            strafeRight();
        }
        setPowerAll(0);
        runLiftToPosition(1500);
        startingPosition = leftDriveFront.getCurrentPosition();
        while (calculateInchesTraveled(startingPosition) < 10) {
            setPowerAll(0.5);
        }
        setPowerAll(0);
        claw.setPosition(1);
        sleep(1000);
        startingPosition = leftDriveFront.getCurrentPosition();
        while (calculateInchesTraveled(startingPosition) < 5) {
            setPowerAll(-0.5);
        }
        setPowerAll(0);
        runLiftToPosition(60);


    }

    private void initWebcam() {



    }
}

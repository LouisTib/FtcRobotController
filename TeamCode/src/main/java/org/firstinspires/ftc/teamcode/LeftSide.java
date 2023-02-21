package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Left Side")
public class LeftSide extends LinearOpMode {


    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor rightDriveBack = null;
    private DcMotor leftDriveBack = null;

    private DcMotorEx lift = null;
    private Servo claw = null;

    // Our sensors, motors, and other devices go here, along with other long term state
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    // Name of the Webcam to be set in the config

    private String webcamName = "Webcam 1";

    float curHeading;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();
        initDrive();
    }


    private void setPowerAll(double a) {
        leftDriveFront.setPower(a);
        rightDriveFront.setPower(a);
        leftDriveBack.setPower(a);
        rightDriveBack.setPower(a);
    }

    private void setPowerLeft(double a) {
        leftDriveFront.setPower(a);
        leftDriveBack.setPower(a);
    }

    private void setPowerRight(double a) {
        rightDriveFront.setPower(a);
        rightDriveBack.setPower(a);
    }

    private void strafeLeft(double speed) {
        rightDriveBack.setPower(-speed);
        leftDriveBack.setPower(speed);
        rightDriveFront.setPower(speed);
        leftDriveFront.setPower(-speed);

    }

    private double strafeRight(double speed) {
        rightDriveBack.setPower(speed);
        leftDriveBack.setPower(-speed);
        rightDriveFront.setPower(-speed);
        leftDriveFront.setPower(speed);

        return speed;
    }


    private double calculateInchesTraveled(double startingTicks) {
        double TICKS_PER_REV = 537.7; // GoBuilda 5203 312RPM motor
        double WHEEL_DIAMETER = 4; // Self-explanatory
        double INCHES_PER_REV = WHEEL_DIAMETER * Math.PI;

        double ticks_traveled = Math.abs(leftDriveFront.getCurrentPosition() - startingTicks);
        double revolutions = ticks_traveled * TICKS_PER_REV;
        return revolutions * INCHES_PER_REV;
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

            // Over-current protection
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

    private void slowDownForward(){
        double targetSpeed = 0;
        double currentSpeed = 0.5;
        while(currentSpeed > targetSpeed){
            setPowerAll(currentSpeed);
            sleep(100);
            currentSpeed -= 0.01;
        }

        setPowerAll(0);
    }

    private void speedUpForward(){
        double targetSpeed = 0.4;
        double currentSpeed = 0.25;
        while(currentSpeed < targetSpeed){
            setPowerAll(currentSpeed);
            sleep(20);
            currentSpeed += 0.01;
        }

        setPowerAll(0);

    }

    private void slowDownBack(){
        double targetSpeed = 0;
        double currentSpeed = -0.5;
        while(currentSpeed < targetSpeed){
            setPowerAll(currentSpeed);
            sleep(100);
            currentSpeed += 0.01;
        }

        setPowerAll(0);
    }

    private void speedUpBack(){
        double targetSpeed = -0.4;
        double currentSpeed = -0.25;
        while(currentSpeed > targetSpeed){
            setPowerAll(currentSpeed);
            sleep(15);
            currentSpeed -= 0.01;
        }

        setPowerAll(0);


    }

    private void slowDownRight(){
        double targetSpeed = 0;
        double currentSpeed = 0.5;
        while(currentSpeed > targetSpeed){
            strafeRight(currentSpeed);
            sleep(25);
            currentSpeed -= 0.01;
        }

        setPowerAll(0);
    }

    private void speedUpRight(){
        double targetSpeed = 0.5;
        double currentSpeed = 0.3;
        while(currentSpeed < targetSpeed){
            strafeRight(currentSpeed);
            sleep(53);
            currentSpeed += 0.01;
        }

        setPowerAll(0);


    }

    private void slowDownLeft(){
        double targetSpeed = 0;
        double currentSpeed = 0.5;
        while(currentSpeed > targetSpeed){
            strafeLeft(currentSpeed);
            sleep(25);
            currentSpeed -= 0.01;
        }

        setPowerAll(0);


    }

    private void speedUpLeft(){
        double targetSpeed = 0.5;
        double currentSpeed = 0.3;
        while(currentSpeed < targetSpeed){
            strafeLeft(currentSpeed);
            sleep(50);
            currentSpeed += 0.01;
        }

        setPowerAll(0);




    }


    private float checkOrientation() {
        // read the orientation of the robot
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        return angles.firstAngle;
    }

    private void centerRobot(){
        //Loop to check if robot hits 0
        while (imu.getAngularOrientation().firstAngle != 0) {
            if (imu.getAngularOrientation().firstAngle < 0) {
                // We're too far right, turn left
                setPowerRight(0.12);
                setPowerLeft(-0.12);
            } else if (imu.getAngularOrientation().firstAngle > 0) {
                // We're too far left, turn right
                setPowerRight(-0.12);
                setPowerLeft(0.12);
            }
        }
        setPowerAll(0);
    }



    private void initDrive() throws InterruptedException {

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double angle = imu.getAngularOrientation().firstAngle;


        if (opModeIsActive()) {

            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Lift Pos", lift.getCurrentPosition());
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());

        }


        sleeveDetection.getPosition();
        sleep(1000);


        if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER) {
            //CYAN - 3
            claw.setPosition(0.8);

            sleep(500);

            speedUpRight();

            sleep(500);

            centerRobot();

            runLiftToPosition(1500);

            sleep(500);

            speedUpForward();

            sleep(500);

            centerRobot();

            sleep(500);

            claw.setPosition(0.9);

            sleep(500);

            speedUpBack();

            sleep(500);

            runLiftToPosition(10);

            sleep(500);

            centerRobot();

            sleep(500);

            for(int i = 0; i < 2000; i++) {
                setPowerAll(0.3);
            }

            setPowerAll(0);

            sleep(500);

            centerRobot();

            sleep(500);

            slowDownRight();

            sleep(500);

            centerRobot();

            stop();
        }

        if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT) {
            //MAGENTA - 2
            claw.setPosition(0.8);

            sleep(500);

            speedUpRight();

            sleep(500);

            centerRobot();

            runLiftToPosition(1500);

            sleep(500);

            speedUpForward();

            sleep(500);

            centerRobot();

            sleep(500);

            claw.setPosition(0.9);

            sleep(500);

            speedUpBack();

            sleep(500);

            runLiftToPosition(10);

            sleep(500);

            centerRobot();

            sleep(500);

            for(int i = 0; i < 2000; i++) {
                setPowerAll(0.3);
            }

            setPowerAll(0);

            sleep(500);

            centerRobot();

            sleep(500);

            slowDownLeft();

            sleep(500);

            centerRobot();

            stop();
        }

        if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.LEFT) {
            //GREEN - 1
            claw.setPosition(0.8);

            sleep(500);

            speedUpRight();

            sleep(500);

            centerRobot();

            runLiftToPosition(1500);

            sleep(500);

            speedUpForward();

            sleep(500);

            centerRobot();

            sleep(500);

            claw.setPosition(0.9);

            sleep(500);

            speedUpBack();

            sleep(500);

            runLiftToPosition(10);

            sleep(500);

            centerRobot();

            sleep(500);

            for(int i = 0; i < 2000; i++) {
                setPowerAll(0.3);
            }

            setPowerAll(0);

            sleep(500);

            centerRobot();

            sleep(500);

            speedUpLeft();

            for(int i = 0; i < 8000; i++) {
               strafeLeft(0.5);
            }

            slowDownLeft();

            sleep(500);

            centerRobot();






            stop();
        }











    }


}











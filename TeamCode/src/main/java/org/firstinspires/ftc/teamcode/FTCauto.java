package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };


    private static final String VUFORIA_KEY = "AUmiib//////AAABmbrFAeBqbkd/hmTwBoU6jXFUjeYK8xAgeYu6r9ZuLmpgb4tqNl4oIhXkpbBXmCusnhPlxJ3DHEkExTnQKhvCU49Yu2jslI6vaQ+V5F21ZAbbBod6lm9zyBEpkujo7IOq2TdOaJSIdN5wW3zxrHTksfrzBuKZKRsArompruh7jrm/B4W3F/EunA8ymkVoi29W84q81XMwJyonWlS2sd3pebXvLW0YOKmA63QgdmtSpp9XVAccwiH8ND8rk7FXlIIucim1Ig5FmVPLIx88t7doptXh8uiXfHHMqXc1T1MrRvfemYaUqyg7I5lYLNjLhuRmBZO3BM/qoyjPhpMVtGNh6+z3VgaKhP7O6zI07W0mmMfO";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        waitForStart();
        initDrive();
        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
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
        leftDriveBack.setPower(-0.5);
        rightDriveFront.setPower(0.5);


    }

    private void strafeRight() {
        rightDriveBack.setPower(0.5);
        leftDriveBack.setPower(-0.5);
        rightDriveFront.setPower(-0.5);
        leftDriveFront.setPower(0.5);


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





            claw.setPosition(0.4);
            for (int i = 0; i < 13000; i++) {
                strafeRight();
            }
            setPowerAll(0);
            runLiftToPosition(1500);
            double startPosition1 = leftDriveFront.getCurrentPosition();
            while ((leftDriveFront.getCurrentPosition() - startPosition1) < 225) {
                setPowerAll(0.5);
            }
            setPowerAll(0);
            claw.setPosition(1);
            sleep(1000);
            double startPosition2 = leftDriveFront.getCurrentPosition();
            while ((leftDriveFront.getCurrentPosition() - startPosition2) > -150) {
                setPowerAll(-0.5);
            }
            setPowerAll(0);
            runLiftToPosition(60);


        }
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData("Status", "Initialized");

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        }

                    }
                }
                telemetry.update();
            }
        }
    }
}
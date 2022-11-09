package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Ez", group="Thread Group")

public class combination extends OpMode {
    private initMultiThread drive = new initMultiThread();
    private FTCCode cam = new FTCCode();

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        drive.start();
        cam.start();


    }


    @Override
    public void stop() {
    }
}

package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "EncoderTester")

public class EncoderTester extends LinearOpMode {

    public void runOpMode() {
        waitForStart();

        DcMotor frontSlide  = null;
        frontSlide   = hardwareMap.get(DcMotor.class, "cm2");

        int position = 0;
        int actualPosition = 0;
        boolean pressinga = false;
        boolean pressingy = false;
        boolean pressingx = false;
        boolean pressingb = false;
        double speed = 0.2;
        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {
            telemetry.addData("position", frontSlide.getCurrentPosition());
            telemetry.addData("actualPosition", actualPosition);
            telemetry.addData("speedInterval", speed);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pandaPathing.follower.Follower;

import java.util.*;

@Config
@TeleOp(name = "Telepepepop", group = "Drive")
public class TelePOP extends OpMode {
    private Follower robot = new Follower(hardwareMap);

    // hardware definitions
    public DcMotorEx leftFront, leftRear, rightFront, rightRear, frontSlides, backSlides, hangerL, hangerR;
    public List<DcMotorEx> tracking    = new ArrayList<DcMotorEx>(Arrays.asList(leftFront  , leftRear  , rightFront, rightRear));
    public List<DcMotorEx> nonTracking = new ArrayList<DcMotorEx>(Arrays.asList(frontSlides, backSlides, hangerL   , hangerR  ));
    public List<DcMotorEx> allMotors   = new ArrayList<DcMotorEx>(nonTracking);
    public Servo drv4bL, drv4bR, timy, vbaR, vbaL, v4b;

    // slide PIDF
    public PIDController slideyController;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double TICKS_PER_DEG = 103.8/360;

    boolean dUpPressed, dDownPressed, yPressed, aPressed;
    double strafePow;

    public void init() {
        // motor configs
        allMotors.addAll(tracking);
        leftFront   = hardwareMap.get(DcMotorEx.class, "cm0");
        leftRear    = hardwareMap.get(DcMotorEx.class, "em1");
        rightRear   = hardwareMap.get(DcMotorEx.class, "em2");
        rightFront  = hardwareMap.get(DcMotorEx.class, "cm1");
        frontSlides = hardwareMap.get(DcMotorEx.class, "cm2");
        backSlides  = hardwareMap.get(DcMotorEx.class, "cm3");
        hangerL     = hardwareMap.get(DcMotorEx.class, "em2");
        hangerR     = hardwareMap.get(DcMotorEx.class, "em2");
        for(DcMotorEx motor :   allMotors) motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        for(DcMotorEx motor :    tracking) motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        for(DcMotorEx motor : nonTracking) motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFront .setDirection(DcMotorEx.Direction.REVERSE);
        leftRear  .setDirection(DcMotorEx.Direction.REVERSE);
        backSlides.setDirection(DcMotorEx.Direction.REVERSE);

        // servo configs
        drv4bL = hardwareMap.get(Servo.class, "cs0");
        drv4bR = hardwareMap.get(Servo.class, "cs1");
        timy   = hardwareMap.get(Servo.class, "cs2");
        vbaL   = hardwareMap.get(Servo.class, "cs3");
        vbaR   = hardwareMap.get(Servo.class, "cs4");
        v4b    = hardwareMap.get(Servo.class, "cs5");

        // pid configs
        slideyController = new PIDController(p, i, d);

        // configs
        robot.startTeleopDrive();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop(){
        // driving
        strafePow = gamepad2.left_trigger != 0 ? -gamepad2.left_trigger : gamepad2.right_trigger;
        robot.setTeleOpMovementVectors(-gamepad1.left_stick_y, strafePow, -gamepad1.right_stick_x);
        robot.update();

        // calculate the slide power using pid
        slideyController.setPID(p, i, d);
        int slidePos = frontSlides.getCurrentPosition();
        double pid = slideyController.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_PER_DEG)) * f;
        double power = pid + ff;

        // change slide target based on controller control
        if(gamepad2.left_stick_y != 0) {
            if (gamepad2.left_stick_y > 0 && slidePos < 1000) // upper limit
                target += 10 * gamepad2.left_stick_y;
            if (gamepad2.left_stick_y < 0 && slidePos > 0) // lower limit
                target += 10 * gamepad2.left_stick_y;
            frontSlides.setPower(power);
            backSlides.setPower(power);
        } else {
            if (gamepad2.dpad_up && !dUpPressed) {
                target = 700; // top basket
                dUpPressed = true;
            } else if (!gamepad2.dpad_up) dUpPressed = false;
            if (gamepad2.dpad_down && !dDownPressed) {
                target = 0; // bottom
                dDownPressed = true;
            } else if (!gamepad2.dpad_down) dDownPressed = false;
        }

        // four bar
        if(gamepad2.y && !yPressed){
            drv4bR.setPosition(1);
            drv4bL.setPosition(1);
            yPressed = true;
        } else if(!gamepad2.y){
            yPressed = false;
        }
        if(gamepad2.a && !aPressed){
            drv4bR.setPosition(0);
            drv4bL.setPosition(0);
            aPressed = true;
        } else if(!gamepad2.a){
            aPressed = false;
        }

        telemetry.addData("pos", slidePos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
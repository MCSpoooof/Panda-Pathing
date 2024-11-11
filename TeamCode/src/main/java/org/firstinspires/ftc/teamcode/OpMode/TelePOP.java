package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.*;

@Config
@TeleOp(name = "Telelel op", group = "Drive")
public class TelePOP extends OpMode {
    //private Follower robot = new Follower(hardwareMap);

    // hardware definitions
    public DcMotorEx leftFront, leftRear, rightFront, rightRear, frontSlides, backSlides;
    public List<DcMotorEx> tracking    = new ArrayList<DcMotorEx>(Arrays.asList(leftFront  , leftRear  , rightFront, rightRear));
    public List<DcMotorEx> nonTracking = new ArrayList<DcMotorEx>(Arrays.asList(frontSlides, backSlides));
    public List<DcMotorEx> allMotors   = new ArrayList<DcMotorEx>(nonTracking);
    public Servo drv4bL, drv4bR, timy, neckR, neckL;

    // slide PIDF
    public PIDController slideyController;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0.1;
    public static int target = 0;
    private final double TICKS_PER_DEG = 103.8/360;

    boolean dUpPressed, dDownPressed, yPressed, aPressed, rbumpPressed, extended, backPressed, xPressed, bPressed, timyON;
    double strafePow, extendPosR = 0, extendPosL = 1, speed, mult;

    boolean slowmode = false;

    public void init() {
        // motor configs
        allMotors.addAll(tracking);
        leftFront   = hardwareMap.get(DcMotorEx.class, "cm0");
        leftRear    = hardwareMap.get(DcMotorEx.class, "em1");
        rightRear   = hardwareMap.get(DcMotorEx.class, "em2");
        rightFront  = hardwareMap.get(DcMotorEx.class, "cm3");
        frontSlides = hardwareMap.get(DcMotorEx.class, "cm2");
        backSlides  = hardwareMap.get(DcMotorEx.class, "cm3");
//        hangerL     = hardwareMap.get(DcMotorEx.class, "em2");
//        hangerR     = hardwareMap.get(DcMotorEx.class, "em2");
        leftFront .setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear  .setDirection(DcMotorEx.Direction.FORWARD);
        rightRear .setDirection(DcMotorEx.Direction.REVERSE);

        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // servo configs
        drv4bL = hardwareMap.get(Servo.class, "cs0");
        drv4bR = hardwareMap.get(Servo.class, "es0");
        timy   = hardwareMap.get(Servo.class, "es2");
        neckL = hardwareMap.get(Servo.class, "cs2");
        neckR = hardwareMap.get(Servo.class, "cs4");

        // pid configs
        slideyController = new PIDController(p, i, d);

        // configs
        //robot.startTeleopDrive();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drv4bL.setPosition(1);
        drv4bR.setPosition(0);
    }

    public void loop(){

        // epxperimental mecanum drive
        /*strafePow = gamepad2.left_trigger != 0 ? -gamepad2.left_trigger : gamepad2.right_trigger;
        robot.setTeleOpMovementVectors(-gamepad1.left_stick_y, strafePow, -gamepad1.right_stick_x);
        robot.update();*/
        // classic mecanum drive

        if(gamepad1.left_bumper)
            mult = 0.33;
        if(gamepad1.right_bumper)
            mult = 1;


        double twist  = (-gamepad1.right_stick_x * speed);
        double strafe = (gamepad1.left_trigger != 0 ? gamepad1.left_trigger : gamepad1.right_trigger != 0 ? -gamepad1.right_trigger : 0 * speed);
        double drive  = (gamepad1.left_stick_y * speed);
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++)
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        if (max > 1)
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftRear.setPower(speeds[2]);
        rightRear.setPower(speeds[3]);

        // calculate the slide power using pid
        slideyController.setPID(p, i, d);
        int slidePos = frontSlides.getCurrentPosition();
        double pid = slideyController.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_PER_DEG)) * f;
        double power = pid + ff;

        // change slide target based on controller control
        frontSlides.setTargetPosition(target);
        frontSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontSlides.setPower(1);
        if(gamepad2.left_stick_y != 0) {
            if (-gamepad2.left_stick_y > 0 && target > -6300) // upper limit
                target += 10 * gamepad2.left_stick_y;
            if (-gamepad2.left_stick_y < 0 && target < 0) // lower limit
                target += 10 * gamepad2.left_stick_y;
        } else {
            if (gamepad2.dpad_up && !dUpPressed) {
                target = -6300; // top basket
                dUpPressed = true;
            } else if (!gamepad2.dpad_up) dUpPressed = false;
            if (gamepad2.dpad_down && !dDownPressed) {
                target = 0; // bottom
                dDownPressed = true;
            } else if (!gamepad2.dpad_down) dDownPressed = false;
        }
        if(frontSlides.getCurrentPosition() < -1000)
            speed = -(double)800/frontSlides.getCurrentPosition();
        else speed = mult;

        // extendys
        if(gamepad2.right_stick_y != 0) {
            if (gamepad2.right_stick_y > 0 && drv4bR.getPosition() < 0.654) { // upper limit
                extendPosR += 0.01*gamepad2.right_stick_y;
                extendPosL -= 0.01*gamepad2.right_stick_y;
            }
            if (gamepad2.right_stick_y < 0 && drv4bR.getPosition() > 0) { // lower limit
                extendPosR += 0.01*gamepad2.right_stick_y;
                extendPosL -= 0.01*gamepad2.right_stick_y;
            }
        } else {
            if (gamepad2.y && !rbumpPressed) {
                if (extended){
                    extendPosR = 0;
                    extendPosL = 1;
                    extended = false;
                }
                else {
                    extendPosR = 0.654;
                    extendPosL = 0.305;
                    extended = true;
                }
                rbumpPressed = true;
            } else if (!gamepad2.y) rbumpPressed = false;
        }
        drv4bR.resetDeviceConfigurationForOpMode();
        drv4bL.resetDeviceConfigurationForOpMode();
        drv4bR.setPosition(extendPosR);
        drv4bL.setPosition(extendPosL);

        //v4b
        if (gamepad2.x) {
            neckL.setPosition(0.6);
            //neckR.setPosition(0.5);
        }
        else if (gamepad2.b) {
            neckL.setPosition(0);
            //neckR.setPosition(0.25);
        }

        //timmy
        if (gamepad2.left_bumper) {
            timy.setPosition(0);
            } else if(gamepad2.right_bumper){
                timy.setPosition(1);
            }
        if (gamepad2.back) {
            timy.setPosition(0.5);
        }

       // telemetry.addData("pid calc", slideyController.calculate(slidePos, target));
        //telemetry.addData("pos", slidePos);
        telemetry.addData("pos2", backSlides.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("extebd pos", drv4bR.getPosition());
        telemetry.addData("vbr pos", neckR.getPosition());
        telemetry.addData("timy", timy.getPosition() == 0.5 ? "STOPPED" : timy.getPosition() == 1 ? "REVERSE" : "INTAKING");
        telemetry.update();
    }
}

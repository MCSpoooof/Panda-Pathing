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

import org.firstinspires.ftc.teamcode.pandaPathing.follower.Follower;

import java.util.*;

@Config
@TeleOp(name = "Telepepepop", group = "Drive")
public class TelePOP extends OpMode {
    //private Follower robot = new Follower(hardwareMap);

    // hardware definitions
    public DcMotorEx leftFront, leftRear, rightFront, rightRear, frontSlides, backSlides;
    public List<DcMotorEx> tracking    = new ArrayList<DcMotorEx>(Arrays.asList(leftFront  , leftRear  , rightFront, rightRear));
    public List<DcMotorEx> nonTracking = new ArrayList<DcMotorEx>(Arrays.asList(frontSlides, backSlides));
    public List<DcMotorEx> allMotors   = new ArrayList<DcMotorEx>(nonTracking);
    public Servo drv4bL, drv4bR, timy, vbaR, vbaL;

    // slide PIDF
    public PIDController slideyController;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double TICKS_PER_DEG = 103.8/360;

    boolean dUpPressed, dDownPressed, yPressed, aPressed, rbumpPressed, extended;
    double strafePow, extendPosR, extendPosL;

    public void init() {
        // motor configs
        allMotors.addAll(tracking);
        leftFront   = hardwareMap.get(DcMotorEx.class, "cm0");
        leftRear    = hardwareMap.get(DcMotorEx.class, "em1");
        rightRear   = hardwareMap.get(DcMotorEx.class, "em2");
        rightFront  = hardwareMap.get(DcMotorEx.class, "cm1");
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
        drv4bL = hardwareMap.get(Servo.class, "es0");
        drv4bR = hardwareMap.get(Servo.class, "es1");
        timy   = hardwareMap.get(Servo.class, "es2");
        vbaL   = hardwareMap.get(Servo.class, "es3");
        vbaR   = hardwareMap.get(Servo.class, "es4");

        // pid configs
        slideyController = new PIDController(p, i, d);

        // configs
        //robot.startTeleopDrive();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop(){
        // epxperimental mecanum drive
        /*strafePow = gamepad2.left_trigger != 0 ? -gamepad2.left_trigger : gamepad2.right_trigger;
        robot.setTeleOpMovementVectors(-gamepad1.left_stick_y, strafePow, -gamepad1.right_stick_x);
        robot.update();*/
        // classic mecanum drive
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad2.left_trigger != 0 ? -gamepad2.left_trigger : 0;
        double twist  = -gamepad1.right_stick_x;
        double speed = 1.0;
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
        frontSlides.setPower(power);
        backSlides.setPower(power);
        if(gamepad2.left_stick_y != 0) {
            if (-gamepad2.left_stick_y > 0 && target > -1000) // upper limit
                target += 10 * gamepad2.left_stick_y;
            if (-gamepad2.left_stick_y < 0 && target < 0) // lower limit
                target += 10 * gamepad2.left_stick_y;
        } else {
            if (gamepad2.dpad_up && !dUpPressed) {
                target = -700; // top basket
                dUpPressed = true;
            } else if (!gamepad2.dpad_up) dUpPressed = false;
            if (gamepad2.dpad_down && !dDownPressed) {
                target = 0; // bottom
                dDownPressed = true;
            } else if (!gamepad2.dpad_down) dDownPressed = false;
        }

        // extendys
        if(gamepad2.right_stick_y != 0) {
            if (-gamepad2.right_stick_y > 0 && drv4bR.getPosition() < 0.654) { // upper limit
                extendPosL += 0.1*gamepad2.right_stick_y;
                extendPosR -= 0.1*gamepad2.right_stick_y;
                drv4bL.setPosition(extendPosL);
                drv4bR.setPosition(extendPosR);
            }
            if (-gamepad2.right_stick_y < 0 && drv4bR.getPosition() > 0) { // lower limit
                extendPosL += 0.1*gamepad2.right_stick_y;
                extendPosR -= 0.1*gamepad2.right_stick_y;
                drv4bL.setPosition(extendPosL);
                drv4bR.setPosition(extendPosR);
            }
        } else {
            if (gamepad2.right_bumper && !rbumpPressed) {
                if (extended){
                    drv4bR.setPosition(0);
                    drv4bL.setPosition(1);
                    extended = false;
                }
                else {
                    drv4bR.setPosition(0.654);
                    drv4bL.setPosition(0.305);
                    extended = true;
                }
                rbumpPressed = true;
            } else if (!gamepad2.right_bumper) rbumpPressed = false;
        }
        telemetry.addData("pid calc", slideyController.calculate(slidePos, target));
        telemetry.addData("pos", slidePos);
        telemetry.addData("pos2", backSlides.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("servo pos", drv4bR.getPosition());
        telemetry.update();
    }
}
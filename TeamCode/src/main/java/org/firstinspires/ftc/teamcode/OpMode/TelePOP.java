package org.firstinspires.ftc.teamcode.OpMode;

import androidx.loader.content.Loader;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pandaPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pandaPathing.util.SingleRunAction;

import java.util.*;

@Config
@TeleOp(name = "Two Person Drive", group = "Drive")
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

    public void init() {
        // motor configs
        allMotors.addAll(tracking);
        leftFront   = hardwareMap.get(DcMotorEx.class, "cm0");
        leftRear    = hardwareMap.get(DcMotorEx.class, "cm2");
        rightRear   = hardwareMap.get(DcMotorEx.class, "cm3");
        rightFront  = hardwareMap.get(DcMotorEx.class, "cm1");
        frontSlides = hardwareMap.get(DcMotorEx.class, "em0");
        backSlides  = hardwareMap.get(DcMotorEx.class, "em1");
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
        robot.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        robot.update();

        // correction math!
        slideyController.setPID(p, i, d);
        int slidePos = frontSlides.getCurrentPosition();
        double pid = slideyController.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_PER_DEG)) * f;

        double power = pid + ff;

        frontSlides.setPower(power); // set both to the same power -->
        backSlides.setPower(power);  // as the front encoder calculation

        telemetry.addData("pos", slidePos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
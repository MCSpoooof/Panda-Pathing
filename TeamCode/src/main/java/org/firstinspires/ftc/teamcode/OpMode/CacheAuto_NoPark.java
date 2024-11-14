package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "IM NOT TIRED", group = "Stupid")
public class CacheAuto_NoPark extends LinearOpMode {

    // Declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor frontSlides;

    private Servo timy;
    private Servo neckL;

    @Override
    public void runOpMode() {

        // Initialize hardware
        leftFront   = hardwareMap.get(DcMotorEx.class, "cm0");
        leftRear    = hardwareMap.get(DcMotorEx.class, "em1");
        rightRear   = hardwareMap.get(DcMotorEx.class, "em2");
        rightFront  = hardwareMap.get(DcMotorEx.class, "cm3");
        frontSlides = hardwareMap.get(DcMotorEx.class, "cm2");

        timy = hardwareMap.get(Servo.class, "es2");
        neckL = hardwareMap.get(Servo.class, "cs2");

        // Set all motors to use power-based control (without encoders)
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront .setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear  .setDirection(DcMotorEx.Direction.REVERSE);
        rightRear .setDirection(DcMotorEx.Direction.FORWARD);

        // Wait for the start button
        waitForStart();

        timy.setPosition(0);

        slides(6200);

        sleep(2000);

        neckL.setPosition(0);

        sleep(500);

        // Move forward by setting power
        moveForward(0.3); // Adjust power level here

        // Duration for forward movement (e.g., 2 seconds)
        sleep(600);

        sleep(1000);

        turnLeft(0.3);

        sleep(200);

        timy.setPosition(1);

        moveBackward(0.2);

        sleep(1000);

        slides(0);

        sleep(3000);

        turnRight(0.5);

        sleep(1000);

        // Stop all motors
        stopMoving();
    }

    private void moveForward(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
    }

    private void moveBackward(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(-power);
    }

    private void turnLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
    }

    private void turnRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }

    private void slides(int positin) {
        frontSlides.setTargetPosition(-positin);
        frontSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontSlides.setPower(1);
    }

    private void stopMoving() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}

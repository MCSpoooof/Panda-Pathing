package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pandaPathing.follower.Follower;

@Config
@TeleOp(name = "Telelel op", group = "Drive")
public class TelePOP extends OpMode {
    //private Follower robot = new Follower(hardwareMap);
    private Follower robot;

    // slide PIDF
    public PIDController slideyController;
    public static double p = 0.001, i = 0, d = 0, kg_ = 0.1;
    public static double f = 0.001;
    public int target = 0;
    private final double TICKS_PER_DEG = 103.8/360;

    boolean dUpPressed, dDownPressed, yPressed, aPressed, rbumpPressed, extended = false, backPressed, xPressed, bPressed, clawOpen, depositing = false, grabbing = false, deposited = false, extending = false;
    double strafePow, extendPosR = 0.5, extendPosL = 0.5, speed, mult = 1, depositTimer = 0, grabbingTimer = 0, extendTimer = 0, neckPos = 0.3;

    boolean slowmode = false;

    public void init() {
        robot = new Follower(hardwareMap);
        robot.initialize();
        for (DcMotorEx motor : robot.motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // pid configs
        slideyController = new PIDController(p, i, d);

        // configs
        //robot.startTeleopDrive();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.neck.setPosition(0.3);
        robot.wrist.setPosition(1);
        robot.timmy.setPosition(0.3);
        robot.drv4bL.setPosition(0.5);
        robot.drv4bR.setPosition(0.5);
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


        double twist  = speed*(gamepad1.right_stick_x);
        double strafe = speed*(gamepad1.left_trigger != 0 ? gamepad1.left_trigger : gamepad1.right_trigger != 0 ? -gamepad1.right_trigger : 0 * speed);
        double drive  = speed*(-gamepad1.left_stick_y);
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

        robot.leftFront.setPower(speeds[0]);
        robot.rightFront.setPower(speeds[1]);
        robot.leftRear.setPower(speeds[2]);
        robot.rightRear.setPower(speeds[3]);

        // calculate the slide power using pid
        slideyController.setPID(p, i, d);
        int slidePos = robot.frontSlides.getCurrentPosition();
        double pid = slideyController.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_PER_DEG)) * f;
        double elevator = kg_ * Math.signum(robot.frontSlides.getCurrentPosition() - target);
        double power = (pid + ff)*elevator;

        // change slide target based on controller control
        robot.frontSlides.setPower(power);
        robot.backSlides.setPower(power);

        if(gamepad2.left_stick_y != 0) {
            if (-gamepad2.left_stick_y > 0 && target < 6300) // upper limit
                target += 10 * -gamepad2.left_stick_y;
            if (-gamepad2.left_stick_y < 0 && target > 0) // lower limit
                target += 10 * -gamepad2.left_stick_y;
        } else {
            if (gamepad2.dpad_up && !dUpPressed) {
                target = 6300; // top basket
                dUpPressed = true;
            } else if (!gamepad2.dpad_up) dUpPressed = false;
            if (gamepad2.dpad_down && !dDownPressed) {
                target = 0; // bottom
                dDownPressed = true;
            } else if (!gamepad2.dpad_down) dDownPressed = false;
        }
        if(target <= 0) target = 0; // reset position if out of bounds
        if(target >= 6300) target = 6300;
        if(robot.frontSlides.getCurrentPosition() > 1000)
            speed = (double)800/robot.frontSlides.getCurrentPosition();
        else if(extendPosR >= 0.9)
            speed = 0.5;
        else speed = mult;

        // extendys
        if(gamepad2.right_stick_y != 0) {
            if (-gamepad2.right_stick_y > 0 && robot.drv4bR.getPosition() < 1) { // upper limit
                extendPosR += 0.01*-gamepad2.right_stick_y;
                extendPosL -= 0.01*-gamepad2.right_stick_y;
                neckPos += 0.01*-gamepad2.right_stick_y;
            }
            if (-gamepad2.right_stick_y < 0 && robot.drv4bR.getPosition() > 0.5) { // lower limit
                extendPosR += 0.01*-gamepad2.right_stick_y;
                extendPosL -= 0.01*-gamepad2.right_stick_y;
                neckPos += 0.01*-gamepad2.right_stick_y;
            }
        } else {
            if (gamepad2.a && !aPressed) {
                if (!extended){
                    //extend --> sequence if extending
                    neckPos = 0.8;
                    robot.timmy.setPosition(0.3);
                    extending = true;
                    extendPosR = 1;
                    extendPosL = 0;
                    extended = true;
                }
                else {
                    //retract --> sequence if grabbing
                    grabbing = true;
                    neckPos = 0.93;
                    extended = false;
                }
                aPressed = true;
            } else if (!gamepad2.a) aPressed = false;
        }
        robot.drv4bR.resetDeviceConfigurationForOpMode();
        robot.drv4bL.resetDeviceConfigurationForOpMode();
        robot.drv4bR.setPosition(extendPosR);
        robot.drv4bL.setPosition(extendPosL);
        robot.neck.setPosition(neckPos);

        //claw extend sequence
        if(extending) extendTimer++;
        if (extendTimer >= 20) {
            robot.wrist.setPosition(1);
            robot.timmy.setPosition(0);
            extending = false;
            extendTimer = 0;
        }
        //claw grab sequence
        if(grabbing) grabbingTimer++;
        if(grabbingTimer >= 20){
            robot.timmy.setPosition(0.339);
            clawOpen = false;
            clawOpen = false;
        }
        if(grabbingTimer >= 40){
            neckPos = 0.3;
            extendPosR = 0.5;
            extendPosL = 0.5;
            grabbing = false;
            grabbingTimer = 0;
        }
        //claw deposot sequence
        if (depositing) depositTimer++;
        if (depositTimer >= 40) {
            robot.timmy.setPosition(0);
            clawOpen = true;
            depositing = false;
            deposited = true;
            depositTimer = 0;
        }

        //claw deposit logic --> sequence if depositing
        if(gamepad2.b && !bPressed) {
            if (!deposited && robot.drv4bR.getPosition() <= 0.5) {
                // deposit
                robot.wrist.setPosition(0.228);
                neckPos = 0.08;
                depositing = true;
            } else if (deposited && robot.drv4bR.getPosition() <= 0.5) {
                // retract
                neckPos = 0.3;
                robot.wrist.setPosition(0.75);
                robot.timmy.setPosition(0.3);
                deposited = false;
            }
            bPressed = true;
        } else if(!gamepad2.b) bPressed = false;
        if(robot.drv4bR.getPosition() >= 0.95 && gamepad2.b) {robot.timmy.setPosition(0); clawOpen = true;}

        //timmy independent control
        if(gamepad2.x && !xPressed) {
            if (!clawOpen) {
                robot.timmy.setPosition(0);
                clawOpen = true;
            } else if (clawOpen) {
                robot.timmy.setPosition(0.339);
                clawOpen = false;
            }
            xPressed = true;
        } else if(!gamepad2.x) xPressed = false;

        //hanging
        if((robot.hangerL.getCurrentPosition() < 0 || robot.hangerR.getCurrentPosition() < 0)) {
            robot.hangerL.setTargetPosition(0);
            robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangerL.setPower(1);
            robot.hangerR.setTargetPosition(0);
            robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangerR.setPower(1);
            telemetry.addLine("hanger stabilizing");
        } else if(robot.hangerL.getCurrentPosition() > 6304 || robot.hangerR.getCurrentPosition() > 6304){
            robot.hangerL.setTargetPosition(6304);
            robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangerL.setPower(1);
            robot.hangerR.setTargetPosition(6304);
            robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangerR.setPower(1);
            telemetry.addLine("hanger stabilizing");
        } else if(gamepad1.a){
            robot.hangerL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hangerL.setPower(1);
            robot.hangerR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hangerR.setPower(1);
            telemetry.addLine("hanger up");
        } else if (gamepad1.y){
            robot.hangerL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hangerL.setPower(-1);
            robot.hangerR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hangerR.setPower(-1);
            telemetry.addLine("hanger down");
        } else{
            robot.hangerL.setPower(0);
            robot.hangerR.setPower(0);
            telemetry.addLine("hanger stopped");
        }

        //telemetry.addData("pid calc", pid + ff);
        telemetry.addData("neck pos", robot.neck.getPosition());
        telemetry.addData("slide power", power);
        telemetry.addData("slide target", target);
        telemetry.addData("slide pos", slidePos);
        telemetry.addData("timy", robot.timmy.getPosition() == 0.5 ? "STOPPED" : robot.timmy.getPosition() == 1 ? "REVERSE" : "INTAKING");
        telemetry.addLine("deposit " + depositing +", " + depositTimer);
        telemetry.update();
    }
}

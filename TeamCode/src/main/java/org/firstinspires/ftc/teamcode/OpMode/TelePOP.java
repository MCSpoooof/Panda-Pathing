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
    public static double p = 0.0009, i = 0, d = 0.000001;
    public static double f = 0;
    public int target = 0;
    private final double TICKS_PER_DEG = 103.8/360;

    boolean dUpPressed, dDownPressed, yPressed, aPressed, rbumpPressed, lBumpPressed, backPressed, xPressed, bPressed, clawOpen,
            extended = false, depositing = false, grabbing = false, extending = false, grabbingSpec = false, scoringSpec = false,
            specimen = false, neckUp = true, slowMode = false, clawOut = false;
    double strafePow, extendPosR = 0.379, extendPosL = 0.567, driveSpeed, turnSpeed, mult = 1, depositTimer = 0, grabbingTimer = 0, extendTimer = 0, neckPos = 0.3, clawTimer = 0, scoringTimer = 0, grabbingTimer2 = 0;
    public static double wristFDown = 0.8, wristFOut = 0.536, wristBOut = 0.055, wristBDown = 0;
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
        robot.wrist.setPosition(wristFDown);
        robot.timmy.setPosition(0.3);
        robot.drv4bL.setPosition(0.567);
        robot.drv4bR.setPosition(0.379);
    }

    public void loop(){

        // epxperimental mecanum drive
        /*strafePow = gamepad2.left_trigger != 0 ? -gamepad2.left_trigger : gamepad2.right_trigger;
        robot.setTeleOpMovementVectors(-gamepad1.left_stick_y, strafePow, -gamepad1.right_stick_x);
        robot.update();*/

        //speed control
        if(robot.frontSlides.getCurrentPosition() > 1000 || extendPosR >= 0.9)
            driveSpeed = 0.5;
        else driveSpeed = mult;

        if(gamepad1.left_bumper && !lBumpPressed) {
            if (!slowMode) {
                mult = 0.33;
                slowMode = true;
            } else if (slowMode) {
                mult = 1;
                slowMode = false;
            }
            lBumpPressed = true;
        } else if(!gamepad1.left_bumper) lBumpPressed = false;

        // classic mecanum drive
        double twist  = driveSpeed*(gamepad1.right_stick_x);
        double strafe = -0.8*driveSpeed*(gamepad1.left_trigger != 0 ? gamepad1.left_trigger : gamepad1.right_trigger != 0 ? -gamepad1.right_trigger : 0);
        double drive  = driveSpeed*(-gamepad1.left_stick_y);
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

        // error correction calc (pid)
        slideyController.setPID(p, i, d);
        int slidePos = robot.frontSlides.getCurrentPosition();
        double pid = slideyController.calculate(slidePos, target); //p*(target-slidePos)
        // gravity comp calc (kG)
        double gearRatio = 13.7; //motor revolutions per shaft revolution
        double shaftRadius = 0.008; //m
        double torque = 1.834; //N.m per motor revolution
        double maxLinearForce = torque*gearRatio / shaftRadius; //N per shaft revolution
        double mass = 3; //kg
        double force = mass *(9.81); //N
        double kG = force/maxLinearForce;
        // slide power
        double power = pid + kG;

        // change slide target based on controller control
        robot.frontSlides.setPower(power);
        robot.backSlides.setPower(power);
        if(gamepad2.left_stick_y != 0) {
            //manual slides
            if (-gamepad2.left_stick_y > 0 && target < 4520) // upper limit
                target += 10 * -gamepad2.left_stick_y;
            if (-gamepad2.left_stick_y < 0 && target > 0) // lower limit
                target += 10 * -gamepad2.left_stick_y;
        } else {
            //set position slides
            if (gamepad2.dpad_up && !dUpPressed) {
                target = specimen ? 2000 : 4520; // top bar : top basket
                clawOut = specimen ? false : true; //--> make claw g oout
                dUpPressed = true;
            } else if (!gamepad2.dpad_up) dUpPressed = false;
            if (gamepad2.dpad_down && !dDownPressed) {
                target = specimen ? 1370 : 0; // bottom
                scoringSpec = true; //--> specimen sequence
                dDownPressed = true;
                //reset claw on down
                robot.wrist.setPosition(wristFDown);
                neckPos = 0.3;
                robot.timmy.setPosition(0.3);
                specimen = false;
            } else if (!gamepad2.dpad_down) dDownPressed = false;
        }
        if(target <= 0) target = 0; // reset position if out of bounds
        if(target >= 4520) target = 4520;

        //extend manual
        /*if(gamepad2.right_stick_y != 0) {
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
        } else {*/
        //extendys
        if (gamepad2.a && !aPressed) {
            if (extendPosR <= 0.4){
                //extend initiate
                neckPos = 0.8;
                if(!specimen) robot.wrist.setPosition(wristFDown);
                robot.timmy.setPosition(0.339);
                extending = true; //--> sequence to open claw once out
                extendPosR = 1;
                extendPosL = 0;
                extended = true;
            }
            else{
                //retract after driver1 grabs
                neckPos = 0.3;
                robot.timmy.setPosition(0.3);
                extendPosR = 0.369;
                extendPosL = 0.577;
                robot.wrist.setPosition(wristFDown);
                extended = false;
            }
            aPressed = true;
        } else if (!gamepad2.a) aPressed = false;
        //}

        //driver 1 initiate grabbing sequence
        if(gamepad1.right_bumper && extended && !rbumpPressed){
            if(!neckUp){
                neckPos = 0.8;
                scoringSpec = true;
                neckUp = true;
            } else {
                neckPos = 0.95;
                grabbing = true;
                neckUp = false;
            }
            rbumpPressed = true;
        } if(!gamepad1.right_bumper) rbumpPressed = false;
        robot.drv4bR.resetDeviceConfigurationForOpMode();
        robot.drv4bL.resetDeviceConfigurationForOpMode();
        robot.drv4bR.setPosition(extendPosR);
        robot.drv4bL.setPosition(extendPosL);
        robot.neck.setPosition(neckPos);

        //neck toggle
        if(gamepad2.y && !yPressed) {
            if(!neckUp){
                neckPos = 0.5;
                neckUp = true;
            } else {
                neckPos = 1;
                neckUp = false;
            }
            yPressed = true;
        } else if(!gamepad2.y)
            yPressed = false;

        //claw extend sequence
        if(extending) extendTimer++;
        if(specimen){
            if (extendTimer >= 30 && extendTimer <= 31) {
                neckPos = 1;
                extending = false;
                extendTimer = 0;
            }
        }else if (extendTimer >= 20) {
            robot.timmy.setPosition(0);
            extending = false;
            extendTimer = 0;
        }
        //claw grab sequence
        if(grabbing) grabbingTimer++;
        if(grabbingTimer >= 20){
            robot.timmy.setPosition(0.339);
            clawOpen = false;
            grabbing = false;
            grabbingTimer = 0;
        }
        //claw out sequence
        if(clawOut){
            robot.wrist.setPosition(wristBOut);
            neckPos = 0.25;
            clawOut = false;
        }
        //specimen scoring sequence
        if(scoringSpec) scoringTimer++;
        if(scoringTimer >= (specimen ? 40 : 20)){
            robot.timmy.setPosition(0);
            clawOpen = true;
            scoringSpec = false;
            scoringTimer = 0;
        }
        //specimen grabbing sequence
        if(grabbingSpec) grabbingTimer2++;
        if(grabbingTimer2 >= 40){
            robot.wrist.setPosition(wristBDown);
            grabbingSpec = false;
            grabbingTimer = 0;
        }

        //claw deposit logic --> sequence if depositing
        if(gamepad2.b && !bPressed) {
            if (robot.drv4bR.getPosition() <= 0.5 && !specimen) {
                // deposit
                clawOut = true; //--> make claw go out
                depositing = true; //--> initiate claw go back in/specimen possibility sequence
            }
            if (specimen && robot.drv4bR.getPosition() <= 0.5) { // if specimen has been activated, special grab
                // retract
                neckPos = 0.35;
                robot.timmy.setPosition(0.338);
                robot.wrist.setPosition(wristBOut+0.1); // pick spec off wall
                grabbingSpec = true; //--> sequence to angle claw down
                depositing = false;
            }
            bPressed = true;
        } else if(!gamepad2.b) bPressed = false;
        if(robot.drv4bR.getPosition() >= 0.95 && gamepad2.b) {robot.timmy.setPosition(0); clawOpen = true;}
        //claw deposit sequence
        if (depositing) depositTimer++;
        if(depositTimer >= 5 && !bPressed && !specimen) { // make claw drop after return
            if (depositTimer >= (target >= 3000 ? 20 : 50)) {
                neckPos = 0.3;
                robot.wrist.setPosition(wristFDown);
                robot.timmy.setPosition(0.3); // go back in
                clawOpen = false;
            } else if(depositTimer >= (target >= 3000 ? 10 : 40)){
                robot.timmy.setPosition(0); // drop
            }
        } else if (bPressed && depositTimer >= 10 && target <= 3000) { // press b during sequence to make claw go back out
            robot.wrist.setPosition(wristBDown);
            robot.timmy.setPosition(0);
            clawOpen = false;
            neckPos = 0.25;
            specimen = true;
        }
        if(depositTimer >= 80 || !depositing){ // cancel sequence after 80 iterations
            depositing = false;
            depositTimer = 0;
        }

        //timmy independent control
        if(gamepad2.x && !xPressed) {
            if (!clawOpen) {
                robot.timmy.setPosition(0);
                if(specimen) specimen = false;
                clawOpen = true;
            } else {
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

        telemetry.addData("neck pos", robot.neck.getPosition());
        telemetry.addData("slide power", robot.frontSlides);
        telemetry.addData("slide target", target);
        telemetry.addData("slide pos", slidePos);
        telemetry.addData("timy", robot.timmy.getPosition() == 0.5 ? "STOPPED" : robot.timmy.getPosition() == 1 ? "REVERSE" : "INTAKING");
        telemetry.addLine("deposit " + depositing +", " + depositTimer);
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.pandaPathing.follower.Follower.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pandaPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pandaPathing.util.Timer;

@Autonomous(name = "Automus")
public class Automus extends LinearOpMode {
    private Follower robot;
    public PIDController slideyController;
    public static double p = 0.0009, i = 0, d = 0.000001;
    public double target = 0;
    private double lastX = 0;
    private double lastY = 0;
    private static double lastH = 0;
    public boolean stopped = true;
    public int pathState;
    public Timer pathTimer = new Timer();

    public void initi() {
        robot = new Follower(hardwareMap);
        robot.initialize();
        for (DcMotorEx motor : robot.motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // pid configs
        slideyController = new PIDController(p, i, d);
    }

    public void runOpMode() {
        initi();
        setPathState(0);
        while(opModeInInit()) pathUpdate();
        waitForStart();

        setPathState(1);
        while(opModeIsActive()) {
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
            robot.frontSlides.setPower(power);
            robot.backSlides.setPower(power);

            pathUpdate();
            robot.update();

            telemetry.addData("path state", pathState);
            telemetry.update();
        }
        // preload to bar
//        robot.followPath(addPath(31, 6, 0));
//        while(!robot.getCurrentPath().isAtParametricEnd()) {
//            robot.frontSlides.setTargetPosition(1000);
//            robot.frontSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontSlides.setPower(1);
//            robot.update();
//        }
//        robot.backSlides.setPower(0);
//
//        //sample grab 1
//        robot.followPath(addPath(20, -23, -60));
//        while(!robot.getCurrentPath().isAtParametricEnd()) {
//            robot.frontSlides.setTargetPosition(0);
//            robot.frontSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontSlides.setPower(0.5);
//            robot.update();
//        }
//
//        //sample drop off 1
//        robot.followPath(addPath(25, -23, -150));
//        while(!robot.getCurrentPath().isAtParametricEnd()){
//            robot.update();
//        }
//
//        robot.followPath(addPath(25, -25, -66));  // sample grab 2
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//        sleep(1);
//        robot.followPath(addPath(25, -25, -156)); // sample drop off 2
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//        sleep(1);
//        robot.followPath(addPath(25, -27, -72));  // sample grab 3
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//        sleep(1);
//        robot.followPath(addPath(25, -27, -162)); // sample drop off 3
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//        sleep(1);
//
//        robot.followPath(addPath(26, 10, 0));
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//        robot.followPath(addPath(25, -28, 0));
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//
//        robot.followPath(addPath(26, 10, 0));
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//        robot.followPath(addPath(25, -28, 0));
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//
//        robot.followPath(addPath(26, 10, 0));
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
//        robot.followPath(addPath(25, -28, 0));
//        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}


    }

    public Path addPath(double targetX, double targetY, double targetH) {
        Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
        Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
        Path path = new Path(new BezierLine(startPoint, endPoint));
        if(lastX == targetX && lastY == targetY) path = new Path(new BezierPoint(endPoint));
        path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH));
        lastX = targetX;
        lastY = targetY;
        lastH = targetH;
        return path;
    }

     public void pathUpdate() {
        double time = pathTimer.getElapsedTimeSeconds();
         //TODO: USE timer FOR TIMING ACTIONS IN THE SAME CASE, FOR EXAMPLE
         // if(timer >= 5) do somethin
         // if(timer >= 2) something else!
         // else if(timer >= 1)
        double slidePos = robot.frontSlides.getCurrentPosition();
        switch (pathState) {
            case 0: //Runs to the position of the preload and holds it's point at 0.5 power
                if (pathTimer.getElapsedTimeSeconds() > 2)
                    robot.timmy.setPosition(clawClose);
                else {
                    robot.setMaxPower(1);
                    robot.neck.setPosition(neckBack);
                    robot.wrist.setPosition(wristBOut);
                    robot.timmy.setPosition(clawOpen);
                    robot.drv4bL.setPosition(extendLMin);
                    robot.drv4bR.setPosition(extendRMin);
                }
                break;
            case 1: //run robo to bucket and lift slides
                if(stopped){
                    robot.followPath(addPath(-30, 10, 45));
                    stopped = false;
                }
                target = slideMax-300;
                if(slidePos >= target){
                    setPathState(2);
                    stopped = true;
                }
                break;
            case 2: //path to
                if(stopped){
                    robot.followPath(addPath(-32.5, 7.5, 45));
                    stopped = false;
                }
                if(robot.atParametricEnd()){
                    robot.timmy.setPosition(clawOpen);
                    setPathState(3);
                    stopped = true;
                }
                break;
            case 3: //drop sample and back up
                if(stopped){
                    robot.followPath(addPath(-25, 15, 90));
                    stopped = false;
                }
                if (time > 1.5) robot.wrist.setPosition(TelePOP.wristFDown);
                else if(time > 1) robot.timmy.setPosition(clawClose);
                else if(time > 0.5) {target = 0; robot.neck.setPosition(neckBack);}

                if(robot.atParametricEnd() && time > 1.5){
                    setPathState(4);
                    stopped = true;
                }
                break;
            case 4:
                robot.drv4bR.setPosition(extendRMax);
                robot.drv4bL.setPosition(extendLMax);
                robot.neck.setPosition(neckOutDown);
                if(time > 1) robot.timmy.setPosition(clawOpen);
                if(robot.timmy.getPosition() == clawOpen) setPathState(5);
                break;
            case 5:
        }
    }

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}

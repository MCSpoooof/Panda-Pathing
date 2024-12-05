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
        double slidePos = robot.frontSlides.getCurrentPosition();
        switch (pathState) {
            case 0: //Runs to the position of the preload and holds it's point at 0.5 power
                if (pathTimer.getElapsedTimeSeconds() > 2)
                    robot.timmy.setPosition(0.338);
                else {
                    robot.setMaxPower(1);
                    robot.neck.setPosition(0.3);
                    robot.wrist.setPosition(wristBOut);
                    robot.timmy.setPosition(0);
                    robot.drv4bL.setPosition(0.567);
                    robot.drv4bR.setPosition(0.379);
                }
                break;
            case 1: //run robo to bucket and lift slides
                if(!robot.isBusy()) robot.followPath(addPath(-30, 10, 45));
                target = 4200;
                if(slidePos >= 4200) setPathState(2);
                break;
            case 2:
                if(!robot.isBusy()) robot.followPath(addPath(-32.5, 7.5, 45));
                if (time > 0.5) {
                    robot.timmy.setPosition(0);
                }
                if(robot.atParametricEnd()) setPathState(3);
                break;
            case 3: //drop sample and back up
                if(!robot.isBusy()) robot.followPath(addPath(-30, 10, 90));
                target = 0;
                robot.neck.setPosition(0.3);
                robot.wrist.setPosition(TelePOP.wristFDown);
                if(time > 0.2)
                    robot.timmy.setPosition(0.3);
                if(robot.atParametricEnd()) setPathState(4);
                break;
            case 4:
                robot.drv4bR.setPosition(1);
                robot.drv4bL.setPosition(0);
                if(robot.drv4bR.getPosition() == 1) setPathState(5);
                break;
            case 5:

        }
    }

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}

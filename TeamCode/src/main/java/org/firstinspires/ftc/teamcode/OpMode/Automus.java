package org.firstinspires.ftc.teamcode.OpMode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.pandaPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pandaPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pandaPathing.util.Timer;

import java.sql.Array;
import java.util.ArrayList;

//@Disabled
@Autonomous(name = "Automus")
public class Automus extends LinearOpMode {
    private Follower robot;
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
    }

    public void runOpMode() {
        initi();

        waitForStart();

        // preload to bar
        robot.followPath(addPath(31, 6, 0));
        while(!robot.getCurrentPath().isAtParametricEnd()) {
            robot.frontSlides.setTargetPosition(1000);
            robot.frontSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontSlides.setPower(1);
            robot.update();
        }
        robot.backSlides.setPower(0);

        //sample grab 1
        robot.followPath(addPath(20, -23, -60));
        while(!robot.getCurrentPath().isAtParametricEnd()) {
            robot.frontSlides.setTargetPosition(0);
            robot.frontSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontSlides.setPower(0.5);
            robot.update();
        }

        //sample drop off 1
        robot.followPath(addPath(25, -23, -150));
        while(!robot.getCurrentPath().isAtParametricEnd()){
            robot.update();
        }

        robot.followPath(addPath(25, -25, -66));  // sample grab 2
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
        sleep(1);
        robot.followPath(addPath(25, -25, -156)); // sample drop off 2
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
        sleep(1);
        robot.followPath(addPath(25, -27, -72));  // sample grab 3
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
        sleep(1);
        robot.followPath(addPath(25, -27, -162)); // sample drop off 3
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
        sleep(1);

        robot.followPath(addPath(26, 10, 0));
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
        robot.followPath(addPath(25, -28, 0));
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}

        robot.followPath(addPath(26, 10, 0));
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
        robot.followPath(addPath(25, -28, 0));
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}

        robot.followPath(addPath(26, 10, 0));
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}
        robot.followPath(addPath(25, -28, 0));
        while(!robot.getCurrentPath().isAtParametricEnd()){robot.update();}


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

    /*public void pathUpdate() {
        switch (pathState) {
            case 0: //Runs to the position of the preload and holds it's point at 0.5 power
                robot.liftPIDF = false;
                robot.extend.toZero();
                robot.startChamber();
                setPathState(999);
                break;
            case 999:
                if(pathTimer.getElapsedTimeSeconds() > 0.375) {
                    robot.follower.setMaxPower(1);
                    robot.follower.followPath(auto.preload, false);
                    setPathState(1);
                }
                break;
            case 1: //Once Chamber State Machine finishes, begins Pathchain to push elements to the submersible
                if(!robot.isBusy()) {
                    robot.claw.open();
                    robot.setMaxPower(0.9);
                    robot.followPath(auto.pushSamples, true);
                    setPathState(2);
                }
                break;
            case 2: //Once the Pathchain finishes, begins the Specimen State Machine
                if(!robot.isBusy()) {
                    robot.startSpecimen();
                    setPathState(3);
                }
                break;
            case 3: //Once the Specimen State Machine finishes, begins the grab path
                if(auto.actionNotBusy()) {
                    auto.follower.setMaxPower(0.9);
                    auto.follower.followPath(auto.grab1, false);
                    setPathState(4);
                }
                break;
        }
    }*/

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}

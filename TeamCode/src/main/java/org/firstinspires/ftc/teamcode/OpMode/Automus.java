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
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.Point;

import java.sql.Array;
import java.util.ArrayList;

//@Disabled
@Autonomous(name = "Automus")
public class Automus extends LinearOpMode {
    private Follower robot;
    private ArrayList<Path> mainPaths = new ArrayList<Path>();
    private ArrayList<ArrayList<Point>> pathPoints = new ArrayList<ArrayList<Point>>();
    private double lastX = 0;
    private double lastY = 0;
    private double lastH = 0;
    private int currentPath = -1;

    public void initi() {
        robot = new Follower(hardwareMap);
        robot.initialize();
        for (DcMotorEx motor : robot.motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        addPath(24, 0, true);
        addPath(21, -30, true);
        addPath(50, -30);
        addPath(50, -40);
        addPath(15, -40);
        addPath(50, -40);
        addPath(50, -50);
        addPath(15, -50);
        addPath(50, -50);
        addPath(50, -55.7);
        addPath(17, -55.7);
        addPath(6, -40);
        addPath(30, 5);
        addPath(6, -36);
        addPath(30, 5);
        addPath(7, -36);
        addPath(30, 5);
        addPath(7, -36);
        addPath(30, 5);
        addPath(8, -36);
    }

    public void runOpMode() {
        initi();
        waitForStart();
        robot.followPath(mainPaths.get(0));
        while (robot.isBusy()) {

            robot.update();
        }
        //slides up and stuff

        robot.followPath(mainPaths.get(1));
        while (robot.isBusy())
            robot.update();

    }

    public void addPath(double targetX, double targetY, double targetH, boolean breakPath) {
        Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
        Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
        if(breakPath) {
            Path path = new Path(new BezierCurve(startPoint, endPoint));
            if (targetH != lastH && targetH != -69)
                path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH), 0.8);
            else
                path.setConstantHeadingInterpolation(Math.toRadians(lastH));
            mainPaths.add(path);
            currentPath++;
        } else {
            mainPaths.get(currentPath).getCurve().addPoint(endPoint);
            mainPaths.get(currentPath).setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH));
        }
        lastX = targetX;
        lastY = targetY;
        lastH = targetH;
    }
    public void addPath(double targetX, double targetY) {
        addPath(targetX, targetY, -69, false);
    }
    public void addPath(double targetX, double targetY, boolean breakPath) {
        addPath(targetX, targetY, -69, breakPath);
    }
    public void addPath(double targetX, double targetY, double targetH) {
        addPath(targetX, targetY, targetH, false);
    }
}

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
    private PathChain mainPath = new PathChain();
    private int lastX = 0;
    private int lastY = 0;
    private int lastH = 0;

    public void initi() {
        robot = new Follower(hardwareMap);
        robot.initialize();
        for (DcMotorEx motor : robot.motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        addPath(28, 0);
        addPath(22, -10);
        addPath(40, -15);
        addPath(10, -15);
        robot.followPath(mainPath);
    }
    public void runOpMode(){
        initi();
        waitForStart();
        while(opModeIsActive() ) {
            robot.update();
        }
    }

    public void addPath(int targetX, int targetY, int targetH){
        Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
        Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
        Path path = new Path(new BezierLine(startPoint, endPoint));
        if (targetH != lastH && targetH != -69)
            path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH), 0.8);
        else
            path.setConstantHeadingInterpolation(Math.toRadians(lastH));

        lastX = targetX;
        lastY = targetY;
        mainPath.addPath(path);
    }
    public void addPath(int targetX, int targetY){
        addPath(targetX, targetY, -69);
    }
}

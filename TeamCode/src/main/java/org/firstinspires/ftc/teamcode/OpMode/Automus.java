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
import org.firstinspires.ftc.teamcode.pandaPathing.pathGeneration.Point;

//@Disabled
@Autonomous(name = "Automus")
public class Automus extends LinearOpMode {
    private Follower robot;

    public void initi() {
        robot = new Follower(hardwareMap);
        robot.initialize();
        for (DcMotorEx motor : robot.motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Point startPos = new Point(0, 0, Point.CARTESIAN);
        Point Pos1 = new Point(28, 0, Point.CARTESIAN);
        Point Pos2 = new Point(22, -10, Point.CARTESIAN);
        Point Pos3 = new Point(40, -15, Point.CARTESIAN);
        Point Pos4 = new Point(10, -15, Point.CARTESIAN);
        Path path1 = new Path(new BezierLine(startPos, Pos1));
        Path path2 = new Path(new BezierLine(Pos1, Pos2));
        Path path3 = new Path(new BezierLine(Pos2, Pos3));
        Path path4 = new Path(new BezierLine(Pos3, Pos4));
        path1.setConstantHeadingInterpolation(0);
        robot.followPath(path1);
        robot.followPath(path2);
        robot.followPath(path3);
        //robot.followPath(path4);
    }
    public void runOpMode(){
        initi();
        waitForStart();
        while(opModeIsActive() ) {
            robot.update();
            sleep(20);
        }
    }
}

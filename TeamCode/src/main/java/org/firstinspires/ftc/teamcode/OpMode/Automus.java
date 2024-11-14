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
        Point endPos = new Point(24, 0, Point.CARTESIAN);
        Path path1 = new Path(new BezierLine(startPos, endPos));
        path1.setConstantHeadingInterpolation(0);
        robot.followPath(path1);
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

package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.Constants.RobotConstants.liftPIDFCoefficients;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pandaPathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.pandaPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.pandaPathing.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.pandaPathing.util.Timer;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TelePOP extends LinearOpMode {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, frontSlides, backSlides, liftEncoder;

    public Servo leftIntakeArm, rightIntakeArm, intakeClaw, leftOuttakeArm, rightOuttakeArm, outtakeWrist, outerOuttakeClaw, innerOuttakeClaw, plane;

    public Telemetry telemetryA;

    public VoltageSensor controlHubVoltageSensor;

    public PIDFController liftPIDF;

    public SingleRunAction outtakeArmIn, outtakeArmOut, outtakeArmWait, liftManualControlReset, startTransfer, highPreset, middlePreset, lowPreset, resetPreset, intakeClawOpen, innerOuttakeClawClose, outerOuttakeClawClose, transferPresetHold, putOuttakeOut, outtakeClawsOpen, transferReset, intakeReset, intakeOut, innerClawToggle, outerClawToggle, intakeClawToggle, intakeArmMoveUpOnePixel, intakeArmMoveDownOnePixel, intakeTransferInTimerReset, setOuttakeWait, resetIntakeInPosition, toggleOuttakeArmPWM, dropOuttakeArm, intakeAvoid;

    public Timer outtakeTimer, transferTimer, pickUpAdjustTimer;

    public NanoTimer frameTimer;

    public boolean autonomous = false, pickUpAdjustIntakeArm, turnOffOuttakeArmPWM, intakeClawIsOpen;

    public long outtakeMovementTime;

    public double deltaTimeSeconds, outtakeWristDirection, outtakeWristOffset, intakeArmTargetPosition, outtakeArmTargetPosition, intakeArmOutPosition, outtakePreviousStaticPosition, pickUpAdjustDirection;

    public int intakeState, outtakeState, intakeArmTargetState, outtakeArmTargetState, liftTargetPosition, liftPresetTargetPosition, transferState, intakeSpeed, outtakeSpeed;

    public TelePOP() {
    }

    public TelePOP(boolean setAuto) {
        autonomous = setAuto;
    }

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "cm0");
        leftRear = hardwareMap.get(DcMotorEx.class, "cm2");
        rightRear = hardwareMap.get(DcMotorEx.class, "cm3");
        rightFront = hardwareMap.get(DcMotorEx.class, "cm1");
        frontSlides = hardwareMap.get(DcMotorEx.class, "leftLift");
        backSlides = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        backSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntakeArm = hardwareMap.get(Servo.class, "leftIntakeArm");
        rightIntakeArm = hardwareMap.get(Servo.class, "rightIntakeArm");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        innerOuttakeClaw = hardwareMap.get(Servo.class, "innerOuttakeClaw");
        outerOuttakeClaw = hardwareMap.get(Servo.class, "outerOuttakeClaw");
        leftOuttakeArm = hardwareMap.get(Servo.class, "leftOuttakeArm");
        rightOuttakeArm = hardwareMap.get(Servo.class, "rightOuttakeArm");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        plane = hardwareMap.get(Servo.class, "plane");

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        outtakeTimer = new Timer();
        frameTimer = new NanoTimer();
        pickUpAdjustTimer = new Timer();
        transferTimer = new Timer();


        liftPIDF = new PIDFController(liftPIDFCoefficients);

        liftTargetPosition = 0;
        liftPresetTargetPosition = 0;
    }
}
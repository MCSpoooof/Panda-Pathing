package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@Disabled
@Config
@TeleOp(name = "TelePop", group = "Drive")
public class MecanumDrive extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    private DcMotor frontSlide  = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "cm0");
        front_right  = hardwareMap.get(DcMotor.class, "cm1");
        back_left    = hardwareMap.get(DcMotor.class, "em1");
        back_right   = hardwareMap.get(DcMotor.class, "em2");
        frontSlide   = hardwareMap.get(DcMotor.class, "cm2");

        front_left .setDirection(DcMotorEx.Direction.FORWARD);
        front_right  .setDirection(DcMotorEx.Direction.REVERSE);
        back_left  .setDirection(DcMotorEx.Direction.FORWARD);
        back_right  .setDirection(DcMotorEx.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        int position = 0;

        double speed = 1.0;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);

//    if (frontSlide.getCurrentPosition() > 3900 && gamepad2.left_stick_y < 0) {
//                position = 3200;
//            } else if (gamepad2.left_stick_y > 0) {
//                frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                position = 0;
//            } else if (gamepad2.left_stick_y != 0) {
//                position += 50 * -gamepad2.left_stick_y;
//            } else {
//                position = frontSlide.getCurrentPosition();
//            }


            //speed multiplier
            if(frontSlide.getCurrentPosition() > 800 && gamepad1.left_trigger == 0){
                speed = (double)800/frontSlide.getCurrentPosition();
                telemetry.addData("Speed", speed);
            } else {
                speed = 1.0;
            }
    }
}
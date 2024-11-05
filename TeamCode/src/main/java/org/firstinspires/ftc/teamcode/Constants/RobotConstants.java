package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static int

            // max extension of the scoring slides TODO: find this later
            LIFT_MAX_POSITION = 2700,

    // low bucket slide position TODO: find this later
    LOW_BUCKET = 20,

    // high bucket slide position TODO: find this later
    HIGH_BUCKET = 0,

    // low bar slide position TODO: find this later
    LOW_BAR = 0,

    // high bar slide position TODO: find this later
    HIGH_BAR = 1400,

    // high bar slide position TODO: find this later
    LOW_BAR_SCORE = 1400,

    // high bar slide position TODO: find this later
    HIGH_BAR_SCORE = 1400;

    public static final int
            // states for mechanisms TODO* Replace with correct order
            INTAKE_IN = 0,
            INTAKE_OUT = 1,
            OUTTAKE_IN = 2,
            OUTTAKE_GOING_OUT = 3,
            OUTTAKE_OUT = 4,
            OUTTAKE_GOING_IN = 5,
            INTAKE_AVOID = 6,
            OUTTAKE_WAIT = 7,
            TRANSFER_IDLE = 8,
            TRANSFER_POSITIONING = 9,
            TRANSFER_DROPPING = 10,
            TRANSFER_PRESET_HOLD = 11,
            TRANSFER_OUT = 12,
            TRANSFER_RESET = 13,
            TRANSFER_RESET_CLAW_DROP = 14,
            TRANSFER_GRAB = 15,
            TRANSFER_INTAKE_AVOID = 16,
            TRANSFER_INTAKE_AVOID_RUN_OUT = 17;


    /**
     * IMPORTANT: all arm servo positions are from the left side
     */
    public static double

    // the claw value TODO: find this later
    CLAW_POSITION = 16.25,

    // the right DRV4B value TODO: find this later
    DRV4B_R = 16.25,

    // the left  DRV4B value TODO: find this later
    DRV4B_L = 16.25;

    /**
     * These are all in milliseconds!
     */
    public static long

    // the time it takes for the pixels to fall through the transfer
    CLAW_CLOSING_TIME = 200; // TODO: find this later


}
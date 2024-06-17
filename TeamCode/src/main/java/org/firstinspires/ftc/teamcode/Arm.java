package org.firstinspires.ftc.teamcode;

// Imports

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

// Variables
    static private DcMotor rightArm;
    static private DcMotor leftArm;
    static public final int MINIMAL_HOLD_POSITION = -500;
    static private int hold_position1 = 0;
    static private int hold_position2 = 0;
    static private int MAX_SPEED = 300;
    static public double CONST_SPEED;
    static private int HANGING_POSITION = -1195;
    static public int UNLOADING_POSITION = -1595;
    static public boolean HANGING_MODE_ACTIVE = false;
    static public boolean DPAD_PRESSED = false;
    static public boolean LOADING_MODE_ACTIVE = false;
    static private boolean got_position_to_hold = false;
    static public final int COLLECT_WHITE_PIXEL_POSITION = -140;

// Initializing
    public static void init(DcMotor motor1, DcMotor motor2) {
        rightArm = motor1;
        leftArm = motor2;

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DPAD_PRESSED = false;
        LOADING_MODE_ACTIVE = false;
        HANGING_MODE_ACTIVE = false;
        got_position_to_hold = false;
    }

// System's functions
    public static void moveUp(double SPEED, double CONST_SPEED) {
        got_position_to_hold = false;

        rightArm.setPower(1);
        rightArm.setTargetPosition(rightArm.getCurrentPosition() - (int) (MAX_SPEED * SPEED * CONST_SPEED));
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArm.setPower(1);
        leftArm.setTargetPosition(leftArm.getCurrentPosition() + (int) (MAX_SPEED * SPEED * CONST_SPEED));
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public static void moveDown(double SPEED, double CONST_SPEED) {
        got_position_to_hold = false;

        rightArm.setPower(1);
        rightArm.setTargetPosition(rightArm.getCurrentPosition() + (int) (MAX_SPEED * SPEED * CONST_SPEED));
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArm.setPower(1);
        leftArm.setTargetPosition(leftArm.getCurrentPosition() - (int) (MAX_SPEED * SPEED * CONST_SPEED));
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public static void hangingModeArm() {
        got_position_to_hold = false;

        HANGING_MODE_ACTIVE = true;
        if (rightArm.getCurrentPosition() > -1180 || rightArm.getCurrentPosition() < -1220){
            rightArm.setPower(1);
            rightArm.setTargetPosition(HANGING_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftArm.setPower(1);
            leftArm.setTargetPosition(-HANGING_POSITION);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            HANGING_MODE_ACTIVE = false;
            if (DPAD_PRESSED) {
                DPAD_PRESSED = false;
            } else {
                DPAD_PRESSED = true;
            }
        }
    }
    public static void loadingModeArm(){
        if (rightArm.getCurrentPosition() < -5 && leftArm.getCurrentPosition() > 5) {
            got_position_to_hold = false;
            rightArm.setPower(1);
            rightArm.setTargetPosition(rightArm.getCurrentPosition() - rightArm.getCurrentPosition());
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftArm.setPower(1);
            leftArm.setTargetPosition(leftArm.getCurrentPosition() - leftArm.getCurrentPosition());
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LOADING_MODE_ACTIVE = true;
        } else {
            got_position_to_hold = false;
            rightArm.setPower(0);
            leftArm.setPower(0);

            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LOADING_MODE_ACTIVE = false;
        }
    }
    public static void BRAKE() {
        if (!got_position_to_hold) {
            got_position_to_hold = true;
            hold_position1 = rightArm.getCurrentPosition();
            hold_position2 = leftArm.getCurrentPosition();
        }
        rightArm.setPower(1);
        leftArm.setPower(1);

        rightArm.setTargetPosition(hold_position1);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArm.setTargetPosition(hold_position2);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public static void brake(){
        if (!got_position_to_hold) {
            got_position_to_hold = true;
            hold_position1 = rightArm.getCurrentPosition();
            hold_position2 = leftArm.getCurrentPosition();
        }
        if (!DPAD_PRESSED) {
            if (passedMinimalHoldPosition() && !LOADING_MODE_ACTIVE) {
                rightArm.setPower(1);
                leftArm.setPower(1);

                rightArm.setTargetPosition(hold_position1);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftArm.setTargetPosition(hold_position2);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                rightArm.setPower(0);
                leftArm.setPower(0);
            }
        }
        else {
            rightArm.setPower(1);
            leftArm.setPower(1);

            rightArm.setTargetPosition(hold_position1);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftArm.setTargetPosition(hold_position2);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public static void stopMoving() {
        rightArm.setPower(0);
        leftArm.setPower(0);
    }

// Getting variables
    public static int getArm1Position() {
        return rightArm.getCurrentPosition();
    }
    public static int getArm2Position() {
        return leftArm.getCurrentPosition();
    }
    public static boolean passedMinimalHoldPosition() {
        return rightArm.getCurrentPosition() <= MINIMAL_HOLD_POSITION;
    }
    public static boolean arrivedPosition (int current_position, int finish_position, boolean finish_bigger_than_start) {
        if (finish_bigger_than_start) {
            return current_position >= finish_position;
        }
        else {
            return current_position <= finish_position;
        }
    }

// Telemetry
    public static void addDataToTelemetry(Telemetry telemetry) {
        telemetry.addData("right arm encoder", rightArm.getCurrentPosition());
        telemetry.addData("left arm encoder", leftArm.getCurrentPosition());
    }
}
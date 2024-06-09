package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.robotcore.hardware.Servo;

public class Claws {

// Variables
    private static Servo left_claw;
    private static Servo right_claw;
    static final double LEFT_CLAW_CLOSED_POSITION = 0.64;
    static final double LEFT_CLAW_OPENED_POSITION = 0.47;
    static final double LEFT_CLAW_UNLOADING_POSITION = 0.54;
    static final double RIGHT_CLAW_CLOSED_POSITION = 0.66;
    static final double RIGHT_CLAW_OPENED_POSITION = 0.82;
    static final double RIGHT_CLAW_UNLOADING_POSITION = 0.76;
    private static boolean was_right_bumper_pressed;
    private static boolean was_left_bumper_pressed;

// Initializing
    public static void init(Servo left_claw, Servo right_claw) {
        Claws.left_claw = left_claw;
        Claws.right_claw = right_claw;

        was_right_bumper_pressed = false;
        was_left_bumper_pressed = false;
    }

// System's variables
    public static void moveToStartPosition() {
        left_claw.setPosition(LEFT_CLAW_CLOSED_POSITION);
        right_claw.setPosition(RIGHT_CLAW_CLOSED_POSITION);
    }
    public static void closeRightClaw() {
        right_claw.setPosition(RIGHT_CLAW_CLOSED_POSITION);
    }
    public static void openRightClaw() {
        right_claw.setPosition(RIGHT_CLAW_OPENED_POSITION);
    }
    public static void closeLeftClaw() {
        left_claw.setPosition(LEFT_CLAW_CLOSED_POSITION);
    }
    public static void openLeftClaw() {
        left_claw.setPosition(LEFT_CLAW_OPENED_POSITION);
    }
    public static void loadingModeClaws() {
        right_claw.setPosition(RIGHT_CLAW_OPENED_POSITION);
        left_claw.setPosition(LEFT_CLAW_OPENED_POSITION);
    }

// Operating system
    public static void runClawsTeleop1(boolean left_bumper_pressed, boolean right_bumper_pressed) {
        if (right_bumper_pressed && !was_right_bumper_pressed) {
            if (Math.abs(right_claw.getPosition() - RIGHT_CLAW_CLOSED_POSITION) < Math.abs(right_claw.getPosition() - RIGHT_CLAW_OPENED_POSITION)) {
                right_claw.setPosition(RIGHT_CLAW_OPENED_POSITION);
            } else {
                right_claw.setPosition(RIGHT_CLAW_CLOSED_POSITION);
            }
            was_right_bumper_pressed = true;
        } else if (!right_bumper_pressed) {
            was_right_bumper_pressed = false;
        }
        if (left_bumper_pressed && !was_left_bumper_pressed) {
            if (Math.abs(left_claw.getPosition() - LEFT_CLAW_CLOSED_POSITION) < Math.abs(left_claw.getPosition() - LEFT_CLAW_OPENED_POSITION)) {
                left_claw.setPosition(LEFT_CLAW_OPENED_POSITION);
            } else {
                left_claw.setPosition(LEFT_CLAW_CLOSED_POSITION);
            }
            was_left_bumper_pressed = true;
        } else if (!left_bumper_pressed) {
            was_left_bumper_pressed = false;
        }
    }
    public static void runClawsTeleop2(boolean left_bumper_pressed, boolean right_bumper_pressed) {
        if (right_bumper_pressed && !was_right_bumper_pressed) {
            if (Math.abs(right_claw.getPosition() - RIGHT_CLAW_CLOSED_POSITION) < Math.abs(right_claw.getPosition() - RIGHT_CLAW_UNLOADING_POSITION)) {
                right_claw.setPosition(RIGHT_CLAW_UNLOADING_POSITION);
            } else {
                right_claw.setPosition(RIGHT_CLAW_CLOSED_POSITION);
            }
            was_right_bumper_pressed = true;
        } else if (!right_bumper_pressed) {
            was_right_bumper_pressed = false;
        }
        if (left_bumper_pressed && !was_left_bumper_pressed) {
            if (Math.abs(left_claw.getPosition() - LEFT_CLAW_CLOSED_POSITION) < Math.abs(left_claw.getPosition() - LEFT_CLAW_UNLOADING_POSITION)) {
                left_claw.setPosition(LEFT_CLAW_UNLOADING_POSITION);
            } else {
                left_claw.setPosition(LEFT_CLAW_CLOSED_POSITION);
            }
            was_left_bumper_pressed = true;
        } else if (!left_bumper_pressed) {
            was_left_bumper_pressed = false;
        }
    }

// Getting variables
    public static double getLeftClawPosition() {
        return left_claw.getPosition();
    }
    public static boolean isLeftOpen() {
        return left_claw.getPosition() <= LEFT_CLAW_OPENED_POSITION + 0.000001 && left_claw.getPosition() >= LEFT_CLAW_OPENED_POSITION - 0.000001;
    }
    public static boolean isLeftClose() {
        return left_claw.getPosition() <= LEFT_CLAW_CLOSED_POSITION + 0.000001 && left_claw.getPosition() >= LEFT_CLAW_CLOSED_POSITION - 0.000001;
    }
    public static double getRightClawPosition() {
        return right_claw.getPosition();
    }
    public static boolean isRightOpen() {
        return right_claw.getPosition() <= RIGHT_CLAW_OPENED_POSITION + 0.000001 && right_claw.getPosition() >= RIGHT_CLAW_OPENED_POSITION - 0.000001;
    }
    public static boolean isRightClose() {
        return right_claw.getPosition() <= RIGHT_CLAW_CLOSED_POSITION + 0.000001 && right_claw.getPosition() >= RIGHT_CLAW_CLOSED_POSITION - 0.000001;
    }
}
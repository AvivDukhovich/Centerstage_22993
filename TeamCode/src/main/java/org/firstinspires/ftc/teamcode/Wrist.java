package org.firstinspires.ftc.teamcode;

// Imports

import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

// Variables
    static Servo wrist;
    static final double WRIST_UP_POSITION = 0.13;
    static final double WRIST_DOWN_POSITION = 0.69;
    static final double WRIST_DOWN_POSITION_Autonomous = 0.59;
    static final double WRIST_UNLOADING_POSITION = 0.18;
    static public boolean UP = false;
    static boolean was_Y_pressed;

// Initializing
    public static void init(Servo servo) {
        wrist = servo;
        was_Y_pressed = false;
        UP = false;
    }

// System's functions
    public static void moveUp() {
        wrist.setPosition(WRIST_UP_POSITION);
    }
    public static void moveDown() {
        wrist.setPosition(WRIST_DOWN_POSITION);
    }
    public static void moveToStartPosition() {
        wrist.setPosition(WRIST_DOWN_POSITION);
    }

//    Set & get Variables
    public static void setPosition(double position) {
        wrist.setPosition(position);
    }
    public static double getPosition() {
        return wrist.getPosition();
    }
    public static boolean isWristUp() {return wrist.getPosition() <= Wrist.WRIST_UP_POSITION + 0.000001 && wrist.getPosition() >= Wrist.WRIST_UP_POSITION - 0.000001;}
    public static boolean isWristDown() {return wrist.getPosition() <= Wrist.WRIST_DOWN_POSITION + 0.000001 && wrist.getPosition() >= Wrist.WRIST_DOWN_POSITION - 0.000001;}

// Operating System
    public static void runWrist(boolean Y_pressed){
        if (Y_pressed && !was_Y_pressed) {
            if (Math.abs(wrist.getPosition() - WRIST_DOWN_POSITION) < Math.abs(wrist.getPosition() - WRIST_UP_POSITION)) {
                wrist.setPosition(WRIST_UP_POSITION);
            }
            else {
                wrist.setPosition(WRIST_DOWN_POSITION);
            }
            was_Y_pressed = true;
        }
        if (!Y_pressed) {
            was_Y_pressed = false;
        }
    }
}

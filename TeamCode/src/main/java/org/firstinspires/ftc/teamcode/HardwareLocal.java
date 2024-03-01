package org.firstinspires.ftc.teamcode;

// Imports

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareLocal {

    // Variables
    static private ColorRangeSensor distanceSensorLeft;
    static private ColorRangeSensor distanceSensorRight;
    static public boolean PIXEL_IN_R;
    static public boolean PIXEL_IN_L;

    // Initializing
    public static void init(ColorRangeSensor distanceSensorRight, ColorRangeSensor distanceSensorLeft) {
        HardwareLocal.distanceSensorRight = distanceSensorRight;
//        HardwareLocal.distanceSensorLeft = colorSensorLeft;
        PIXEL_IN_R = false;
        PIXEL_IN_L = false;
    }

    // Getting values
    public static int getRedValueRight() {return distanceSensorRight.red();}
    public static int getBlueValueRight() {return distanceSensorRight.blue();}
    public static int getGreenValueRight() {return distanceSensorRight.green();}
    public static int getAlphaValueRight() {return distanceSensorRight.alpha();}
    public static int getProximityValueRight() {return (int) distanceSensorRight.getDistance(DistanceUnit.MM);}
    public static int getRedValueLeft() {return distanceSensorLeft.red();}
    public static int getBlueValueLeft() {return distanceSensorLeft.blue();}
    public static int getGreenValueLeft() {return distanceSensorLeft.green();}
    public static int getAlphaValueLeft() {return distanceSensorLeft.alpha();}
    public static int getProximityValueLeft() {return (int) distanceSensorLeft.getDistance(DistanceUnit.MM);}

//// System's functions
    public static boolean pixelRight() {
        return distanceSensorRight.getDistance(DistanceUnit.MM) <= 18;
    }
    public static boolean pixelLeft() {
        return distanceSensorLeft.getDistance(DistanceUnit.MM) <= 18;
    }
}
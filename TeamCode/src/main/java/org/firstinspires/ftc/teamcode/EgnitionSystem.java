package org.firstinspires.ftc.teamcode;

// Imports

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class EgnitionSystem {

    // Variables
    static private DcMotor fl_wheel;
    static private DcMotor fr_wheel;
    static private DcMotor bl_wheel;
    static private DcMotor br_wheel;
    static private IMU imu;
    static private double max;
    static private double lx;
    static private double ly;
    static private double rx;
    static private double heading;
    static private double adjustedLx;
    static private double adjustedLy;
    static public double POWER;
    static public double SLOW_POWER;
    static public boolean SLOW_MODE;
    static public boolean PIXELS_IN;
    static public boolean WAS_PRESSED;
    static public boolean A_WAS_PRESSED;
    static public boolean B_WAS_PRESSED;
    static private final double AUTONOMOUS_MOVING_POWER_ORIGINAL = 0.5;
    static private double AUTONOMOUS_MOVING_POWER = AUTONOMOUS_MOVING_POWER_ORIGINAL;
    static private final int ENCODER_CHANGING_SPEED = 1000;

    // Initializing
    public static void init(DcMotor fl_wheel, DcMotor fr_wheel, DcMotor bl_wheel, DcMotor br_wheel, IMU imu) {
        EgnitionSystem.fl_wheel = fl_wheel;
        EgnitionSystem.fr_wheel = fr_wheel;
        EgnitionSystem.bl_wheel = bl_wheel;
        EgnitionSystem.br_wheel = br_wheel;

        EgnitionSystem.bl_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        EgnitionSystem.fl_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        fl_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EgnitionSystem.imu = imu;

        EgnitionSystem.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        EgnitionSystem.imu.resetYaw();

        max = 1;
        lx = 0;
        ly = 0;
        rx = 0;
        heading = 0;
        adjustedLx = 0;
        adjustedLy = 0;
        POWER = 1;
        SLOW_POWER = 0.4;
        SLOW_MODE = false;
        WAS_PRESSED = false;
        PIXELS_IN = false;
        B_WAS_PRESSED = false;
        A_WAS_PRESSED = false;
        AUTONOMOUS_MOVING_POWER = AUTONOMOUS_MOVING_POWER_ORIGINAL;
    }

    public static void initEncoders() {
        fl_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Teleop functions

    /**
     * Teleop mecanum drive train (normal power)
     */
    public static void runTeleopFast() {
        fl_wheel.setPower(((adjustedLy + adjustedLx + rx) / max) * POWER);
        bl_wheel.setPower(((adjustedLy - adjustedLx + rx) / max) * POWER);
        fr_wheel.setPower(((adjustedLy - adjustedLx - rx) / max) * POWER);
        br_wheel.setPower(((adjustedLy + adjustedLx - rx) / max) * POWER);
    }

    /**
     * Teleop mecanum drive train (slow power)
     */
    public static void runTeleopSlow() {
        fl_wheel.setPower(((adjustedLy + adjustedLx + rx) / max) * SLOW_POWER);
        bl_wheel.setPower(((adjustedLy - adjustedLx + rx) / max) * SLOW_POWER);
        fr_wheel.setPower(((adjustedLy - adjustedLx - rx) / max) * SLOW_POWER);
        br_wheel.setPower(((adjustedLy + adjustedLx - rx) / max) * SLOW_POWER);
    }

    public static void runTeleopPickUp() {
        fl_wheel.setPower(((adjustedLy + adjustedLx + rx) / max) * (SLOW_POWER + 0.1));
        bl_wheel.setPower(((adjustedLy - adjustedLx + rx) / max) * (SLOW_POWER + 0.1));
        fr_wheel.setPower(((adjustedLy - adjustedLx - rx) / max) * (SLOW_POWER + 0.1));
        br_wheel.setPower(((adjustedLy + adjustedLx - rx) / max) * (SLOW_POWER + 0.1));
    }
    public static void updateVariablesTeleop(Gamepad gamepad1, Telemetry telemetry) {
        lx = gamepad1.left_stick_x;
        ly = -gamepad1.left_stick_y;
        rx = gamepad1.right_stick_x;

        max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

        heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
        adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

        telemetry.addLine("Press B to reset robot's head direction");
        telemetry.addData("heading: ", heading);
    }

    public static void resetEncoders() {
        fl_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Setting Variables
    public static void setHorizontalPower(double power) {
        lx = power;
    }

    public static void setVerticalPower(double power) {
        ly = power;
    }

    public static void setRotPower(double power) {
        rx = power;
    }

    public static void updateVariablesAutonomous() {
        max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

        heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
        adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);
    }

    private static void setPowerEncoders(DcMotor motor, double power) {
        motor.setPower(power);
        motor.setTargetPosition((int) (motor.getCurrentPosition() + power * ENCODER_CHANGING_SPEED));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Getting variables
    public static int getFlEncoderPosition() {
        return fl_wheel.getCurrentPosition();
    }

    public static int getFrEncoderPosition() {
        return fr_wheel.getCurrentPosition();
    }

    public static int getBlEncoderPosition() {
        return bl_wheel.getCurrentPosition();
    }

    public static int getBrEncoderPosition() {
        return br_wheel.getCurrentPosition();
    }

    // Autonomous functions
    public static void runAutonomous() {
        setPowerEncoders(fl_wheel, ((adjustedLy + adjustedLx + rx) / max) * AUTONOMOUS_MOVING_POWER);
        setPowerEncoders(bl_wheel, ((adjustedLy - adjustedLx + rx) / max) * AUTONOMOUS_MOVING_POWER);
        setPowerEncoders(fr_wheel, ((adjustedLy - adjustedLx - rx) / max) * AUTONOMOUS_MOVING_POWER);
        setPowerEncoders(br_wheel, ((adjustedLy + adjustedLx - rx) / max) * AUTONOMOUS_MOVING_POWER);
    }

    public static boolean arrivedPosition(int current_position, int finish_position, boolean finish_positive) {
        if (finish_positive) {
            return current_position >= finish_position;
        } else {
            return current_position <= finish_position;
        }
    }

    public static void setAutonomousMovingPower(double power) {
        AUTONOMOUS_MOVING_POWER = power;
    }

    public static double getHeading() {
        return heading;
    }

    public static double getFlPower() {
        return fl_wheel.getPower();
    }

    public static double getFrPower() {
        return fr_wheel.getPower();
    }

    public static double getBlPower() {
        return bl_wheel.getPower();
    }

    public static double getBrPower() {
        return br_wheel.getPower();
    }
}
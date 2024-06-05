package org.firstinspires.ftc.teamcode;

// Imports

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

// Teleop
@TeleOp(name = "CenterStage TeleOp")
public class Teleop extends LinearOpMode {

    // Configuring
    Servo drone;
    double DRONE_LUNCH = 0.5;
    double DRONE_INIT = 1;
    @Override
    public void runOpMode() {

// Initialize
        initArm();
        initWrist();
        initClaws();
        initEgnitionSystem();
        initColorRangeSensor();
        initLed();
        initDrone();

        waitForStart();


        while (opModeIsActive()) {

// Running systems
            runDrone();
            runEgnitionSystem();
            runArm();
            runClaws();
            runWrist();
            touchAndGo();
            ledChange();

// Telemetry update
            telemetry.addData("left" , HardwareLocal.getProximityValueLeft());
            telemetry.addData("right" , HardwareLocal.getProximityValueRight());
            telemetry.update();
        }
//        drone.setPosition(0);
    }

// Initializing & running system functions

    public void initDrone() {
        drone = hardwareMap.get(Servo.class, "drone");
        drone.setPosition(DRONE_INIT);
    }
    public void runDrone() {
        if (gamepad1.dpad_down) {
            drone.setPosition(DRONE_LUNCH);
        }
    }
    public void initClaws() {
        Servo left_claw = hardwareMap.get(Servo.class, "left_claw");
        Servo right_claw = hardwareMap.get(Servo.class, "right_claw");
        Claws.init(left_claw, right_claw);
        Claws.closeRightClaw();
        Claws.closeLeftClaw();
    }

    public void runClaws() {
        Claws.runClawsTeleop(gamepad1.left_bumper, gamepad1.right_bumper);
        if (!HardwareLocal.pixelLeft() && Claws.isLeftClose() && Wrist.isWristDown() && HardwareLocal.SENSOR_USAGE) {
            Claws.openLeftClaw();
        }
        if (!HardwareLocal.pixelRight() && Claws.isRightClose() && Wrist.isWristDown() && HardwareLocal.SENSOR_USAGE) {
            Claws.openRightClaw();
        }
        if (Wrist.isWristUp() && !HardwareLocal.pixelRight() && !HardwareLocal.pixelLeft() && Claws.isLeftOpen() && Claws.isRightOpen() && HardwareLocal.SENSOR_USAGE) {
            Claws.closeLeftClaw();
            Claws.closeRightClaw();
        }
    }

    public void initWrist() {
        Servo servo = hardwareMap.get(Servo.class, "wrist");
        Wrist.init(servo);
        Wrist.setPosition(Wrist.WRIST_UP_POSITION);
    }

    public void runWrist() {
        if (Arm.getArm1Position() <= Arm.UNLOADING_POSITION) {
            Wrist.setPosition(Wrist.WRIST_UNLOADING_POSITION + 0.01 * ((int) ((Arm.getArm1Position() - Arm.UNLOADING_POSITION) / -25)));
            EgnitionSystem.SLOW_MODE = true;
            EgnitionSystem.WAS_PRESSED = false;
        } else if (Wrist.isWristDown() && HardwareLocal.pixelRight() && HardwareLocal.pixelLeft() && Claws.isLeftClose() && Claws.isRightClose() && !Wrist.UP && HardwareLocal.SENSOR_USAGE) {
            sleep(100);
            Wrist.UP = true;
            Wrist.moveUp();
        } else if (Wrist.isWristDown() && (!HardwareLocal.pixelRight() || !HardwareLocal.pixelLeft()) && Wrist.UP && HardwareLocal.SENSOR_USAGE) {
            Wrist.UP = false;
        } else {
            Wrist.runWrist(gamepad1.y);
        }
    }

    public void initArm() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "right_arm");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "left_arm");
        Arm.init(motor, motor2);
    }

    public void runArm() {
        if (gamepad1.right_trigger > 0 && Arm.getArm1Position() >= -2200) {
            Arm.moveUp(gamepad1.right_trigger);
            HardwareLocal.HANGING_LAD = false;
        } else if (gamepad1.left_trigger > 0 && Arm.getArm1Position() <= -20 && Arm.getArm2Position() >= 20) {
            Arm.moveDown(gamepad1.left_trigger);
            HardwareLocal.HANGING_LAD = false;
        } else if (gamepad1.dpad_up && !Arm.HANGING_MODE_ACTIVE || !gamepad1.dpad_up && Arm.HANGING_MODE_ACTIVE) {
            Claws.closeRightClaw();
            Claws.closeLeftClaw();
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            EgnitionSystem.SLOW_MODE = false;
            EgnitionSystem.WAS_PRESSED = false;
            HardwareLocal.HANGING_LAD = true;
            Arm.hangingModeArm();
        } else if (gamepad1.x && !Arm.LOADING_MODE_ACTIVE || !gamepad1.x && Arm.LOADING_MODE_ACTIVE) {
            Claws.closeLeftClaw();
            Claws.closeRightClaw();
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            EgnitionSystem.SLOW_MODE = false;
            EgnitionSystem.WAS_PRESSED = false;
            Wrist.UP = false;
            HardwareLocal.HANGING_LAD = false;
            Arm.HANGING_MODE_ACTIVE = false;
            Arm.DPAD_PRESSED = false;
            Arm.loadingModeArm();
        } else {
            Arm.brake();
        }
    }

    public void initEgnitionSystem() {
        DcMotor fl_wheel = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor fr_wheel = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor bl_wheel = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor br_wheel = hardwareMap.get(DcMotor.class, "rightBack");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        EgnitionSystem.init(fl_wheel, fr_wheel, bl_wheel, br_wheel, imu);
    }

    public void runEgnitionSystem() {
        EgnitionSystem.updateVariablesTeleop(gamepad1, telemetry);
        if (gamepad1.b) {
            initEgnitionSystem();
            initEgnitionSystem();
            HardwareLocal.BLINK_IN_TIME = 20;
            HardwareLocal.NOTIFICATION_LAD = true;
        }
        if (Wrist.isWristDown()) {
            EgnitionSystem.SLOW_MODE = true;
            EgnitionSystem.WAS_PRESSED = false;
            EgnitionSystem.PIXELS_IN = false;
        } else if (Wrist.isWristUp() && EgnitionSystem.SLOW_MODE && !EgnitionSystem.PIXELS_IN) {
            EgnitionSystem.SLOW_MODE = false;
            EgnitionSystem.WAS_PRESSED = false;
            EgnitionSystem.PIXELS_IN = true;
        }
        if (Arm.getArm1Position() <= Arm.UNLOADING_POSITION) {
            EgnitionSystem.SLOW_MODE = true;
            EgnitionSystem.WAS_PRESSED = false;
        }
        if ((gamepad1.left_stick_button || gamepad1.right_stick_button) && !EgnitionSystem.WAS_PRESSED && !EgnitionSystem.SLOW_MODE) {
            EgnitionSystem.SLOW_MODE = true;
            EgnitionSystem.WAS_PRESSED = true;
        } else if ((gamepad1.left_stick_button || gamepad1.right_stick_button) && !EgnitionSystem.WAS_PRESSED && EgnitionSystem.SLOW_MODE) {
            EgnitionSystem.SLOW_MODE = false;
            EgnitionSystem.WAS_PRESSED = true;
        }
        if (!gamepad1.left_stick_button && !gamepad1.right_stick_button) {
            EgnitionSystem.WAS_PRESSED = false;
        }
        if (!EgnitionSystem.SLOW_MODE) {
            EgnitionSystem.runTeleop1();
        }
        else {
            EgnitionSystem.runTeleop2();
        }
    }

    public void initColorRangeSensor() {
        ColorRangeSensor distanceSensorRight = hardwareMap.get(ColorRangeSensor.class, "distanceSensorRight");
        ColorRangeSensor distanceSensorLeft = hardwareMap.get(ColorRangeSensor.class, "distanceSensorLeft");
        HardwareLocal.init(distanceSensorRight, distanceSensorLeft);
    }

    public void initLed() {
        RevBlinkinLedDriver ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDrive");
        HardwareLocal.init(ledDriver);
    }

    public void touchAndGo() {
        if (HardwareLocal.getProximityValueLeft() == 152 || HardwareLocal.getProximityValueRight() == 152) {
            HardwareLocal.SENSOR_USAGE = false;
        }
        if (Arm.getArm1Position() <= Arm.MINIMAL_HOLD_POSITION) {
            HardwareLocal.PIXEL_IN_L = true;
            HardwareLocal.PIXEL_IN_R = true;
        }
        if (!HardwareLocal.PIXEL_IN_R && HardwareLocal.pixelRight() && Claws.isRightOpen() && HardwareLocal.SENSOR_USAGE) {
            Claws.closeRightClaw();
            HardwareLocal.PIXEL_IN_R = true;
        } else if (!HardwareLocal.pixelRight()) {
            HardwareLocal.PIXEL_IN_R = false;
        }
        if (!HardwareLocal.PIXEL_IN_L && HardwareLocal.pixelLeft() && Claws.isLeftOpen() && HardwareLocal.SENSOR_USAGE) {
            Claws.closeLeftClaw();
            HardwareLocal.PIXEL_IN_L = true;
        } else if (!HardwareLocal.pixelLeft()) {
            HardwareLocal.PIXEL_IN_L = false;
        }
    }

    public void ledChange() {
        if (HardwareLocal.NOTIFICATION_LAD) {
            if (HardwareLocal.BLINK_IN_TIME % 2 == 0) {
                HardwareLocal.black();
                HardwareLocal.BLINK_IN_TIME--;
            } else if (HardwareLocal.BLINK_IN_TIME >= 0){
                HardwareLocal.red();
                HardwareLocal.BLINK_IN_TIME--;
            } else {
                HardwareLocal.NOTIFICATION_LAD = false;
            }
        } else if (HardwareLocal.HANGING_LAD) {
            HardwareLocal.blue();
        } else if (HardwareLocal.pixelRight() && HardwareLocal.pixelLeft()) {
            HardwareLocal.green();
        } else if (!HardwareLocal.pixelRight() && HardwareLocal.pixelLeft() || HardwareLocal.pixelRight() && !HardwareLocal.pixelLeft()) {
            if (HardwareLocal.BLINK_IN_TIME >= 0 && HardwareLocal.BLINK_IN_TIME < 5) {
                HardwareLocal.black();
                HardwareLocal.BLINK_IN_TIME++;
            } else if (HardwareLocal.BLINK_IN_TIME >= 5 && HardwareLocal.BLINK_IN_TIME < 10) {
                HardwareLocal.red();
                HardwareLocal.BLINK_IN_TIME++;
            } else {
                HardwareLocal.BLINK_IN_TIME = 0;
            }
        } else {
            HardwareLocal.red();
        }
    }
}
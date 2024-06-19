package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BF")

public class BF extends LinearOpMode {

    double ARM_SPEED = 0.6;
    int ARM_UP_POSITION = -2000;
    int ARM_DOWN_POSITION = -300;

    spike_position position = spike_position.RIGHT;

    enum spike_position {
        LEFT,
        RIGHT,
        CENTER
    }

    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;
    Trajectory traj6;
    Trajectory traj7;
    Trajectory traj8;
    Trajectory traj9;
    Trajectory traj10;
    Trajectory traj11;
    @Override
    public void runOpMode() {

        // Initialize everything
        initArm();
        initClaws();
        initWrist();
        initCamera();
        initLed();

        // Close the claws
        Claws.closeRightClaw();
        Claws.closeLeftClaw();

        Wrist.setPosition(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (opModeInInit()) {
            if (PixelDetectorBF.getSpike_position() == 0) {
                position = spike_position.CENTER;
                HardwareLocal.green();
            }
            else if (PixelDetectorBF.getSpike_position() == 1) {
                position = spike_position.LEFT;
                HardwareLocal.green();
            }
            else {
                position = spike_position.RIGHT;
                HardwareLocal.green();
            }

            telemetry.addData("Spike Position: ", position);
            telemetry.addData("Right Region avg: ", Camera.getRightRegion_avg(4));
            telemetry.addData("Left Region avg: ", Camera.getLeftRegion_avg(4));
            telemetry.update();
        }
        Camera.close(4);

        waitForStart();
        Wrist.setPosition(Wrist.WRIST_DOWN_POSITION_Autonomous);
        sleep(500);

        // Choose a path according to the spike position
        if (position == spike_position.RIGHT) {

            // Initialize right path

        }
        else if (position == spike_position.LEFT) {

            // Initialize left path

        }
        else {

            // Initialize Center path

        }
        if (isStopRequested()) return;

        if (position == spike_position.RIGHT) {

            // Execute right path

        }
        else if (position == spike_position.LEFT) {

            // Execute left path

        }
        else {

            // Execute Center path

        }
    }

    public void initClaws() {
        Servo left_claw = hardwareMap.get(Servo.class, "left_claw");
        Servo right_claw = hardwareMap.get(Servo.class, "right_claw");
        Claws.init(left_claw, right_claw);
    }

    public void initWrist() {
        Servo servo = hardwareMap.get(Servo.class, "wrist");
        Wrist.init(servo);
    }

    public void initArm() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "right_arm");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "left_arm");
        Arm.init(motor, motor2);
        Arm.addDataToTelemetry(telemetry);
    }

    public void initCamera() {
        Camera.init(this, hardwareMap, 4);
    }

    public void initLed() {
        RevBlinkinLedDriver ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDrive");
        HardwareLocal.init(ledDriver);
    }
    public void collectWhitePixel1() {
        while (!(Arm.arrivedPosition(Arm.getArm1Position(), Arm.COLLECT_WHITE_PIXEL_POSITION, false) && opModeIsActive())) {
            Arm.moveUp(0.1 , 1);
        }
        Arm.BRAKE();
        Wrist.setPosition(0.9);
        int time = 100;
        while (time > 0 && opModeIsActive()) {
            Arm.BRAKE();
            time --;
        }
    }
    public void collectWhitePixel2() {
        Claws.closeRightClaw();
        int time = 50;
        while (time > 0 && opModeIsActive()) {
            Arm.BRAKE();
            time --;
        }
        Wrist.setPosition(0);
    }
}
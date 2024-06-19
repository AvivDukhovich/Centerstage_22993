package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RRRF")

public class RRRF extends LinearOpMode {

    double ARM_SPEED = 0.6;
    int ARM_UP_POSITION = -2000;
    int ARM_DOWN_POSITION = -300;

    spike_position position;

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
    Trajectory traj12;

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

//        while (opModeInInit()) {
//            if (PixelDetectorRF.getSpike_position() == 0) {
//                position = spike_position.CENTER;
//                HardwareLocal.green();
//            }
//            else if (PixelDetectorRF.getSpike_position() == 1) {
//                position = spike_position.LEFT;
//                HardwareLocal.green();
//            }
//            else {
//                position = spike_position.RIGHT;
//                HardwareLocal.green();
//            }
//
//            telemetry.addData("Spike Position: ", position);
//            telemetry.addData("Right Region avg: ", Camera.getRightRegion_avg(2));
//            telemetry.addData("Left Region avg: ", Camera.getLeftRegion_avg(2));
//            telemetry.update();
//        }
//        Camera.close(2);
        position = spike_position.RIGHT;

        waitForStart();
//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                Arm.brake();
//            }
//        }).start();

        Wrist.setPosition(Wrist.WRIST_DOWN_POSITION_Autonomous);
        sleep(500);

        // Choose a path according to the spike position
        if (position == spike_position.RIGHT) {

            // Initialize right path

            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + 31, drive.getPoseEstimate().getY() + 4))
                    .build();
            traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().getX(), traj1.end().getY(), Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(traj1.end().getX() - 2, traj1.end().getY() + 36))
                    .build();
            traj3 = drive.trajectoryBuilder(new Pose2d(traj2.end().getX(), traj2.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj2.end().getX() + 22, traj2.end().getY() - 160))
                    .build();
            traj4 = drive.trajectoryBuilder(new Pose2d(traj3.end().getX(), traj3.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj3.end().getX() - 26, traj3.end().getY()))
                    .build();
            traj5 = drive.trajectoryBuilder(new Pose2d(traj4.end().getX(), traj4.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj4.end().getX() - 34, traj4.end().getY()))
                    .build();
            traj6 = drive.trajectoryBuilder(new Pose2d(traj5.end().getX(), traj5.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj5.end().getX(), traj5.end().getY() - 43))
                    .build();

        } else if (position == spike_position.LEFT) {

            // Initialize left path

            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + 40, drive.getPoseEstimate().getY()))
                    .build();
            traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().getX(), traj1.end().getY(), Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(traj1.end().getX() - 2, traj1.end().getY() + 34))
                    .build();
            traj3 = drive.trajectoryBuilder(new Pose2d(traj2.end().getX(), traj2.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj2.end().getX() + 42, traj2.end().getY() - 160))
                    .build();
            traj4 = drive.trajectoryBuilder(new Pose2d(traj3.end().getX(), traj3.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj3.end().getX() - 46, traj3.end().getY()))
                    .build();
            traj5 = drive.trajectoryBuilder(new Pose2d(traj4.end().getX(), traj4.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj4.end().getX() - 34, traj4.end().getY()))
                    .build();
            traj6 = drive.trajectoryBuilder(new Pose2d(traj5.end().getX(), traj5.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj5.end().getX(), traj5.end().getY() - 43))
                    .build();

        } else {

            // Initialize Center path

            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + 40, drive.getPoseEstimate().getY()))
                    .build();
            traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().getX(), traj1.end().getY(), Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(traj1.end().getX() - 2, traj1.end().getY() + 34))
                    .build();
            traj3 = drive.trajectoryBuilder(new Pose2d(traj2.end().getX(), traj2.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj2.end().getX() + 42, traj2.end().getY() - 160))
                    .build();
            traj4 = drive.trajectoryBuilder(new Pose2d(traj3.end().getX(), traj3.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj3.end().getX() - 46, traj3.end().getY()))
                    .build();
            traj5 = drive.trajectoryBuilder(new Pose2d(traj4.end().getX(), traj4.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj4.end().getX() - 34, traj4.end().getY()))
                    .build();
            traj6 = drive.trajectoryBuilder(new Pose2d(traj5.end().getX(), traj5.end().getY()))
                    .lineToConstantHeading(new Vector2d(traj5.end().getX(), traj5.end().getY() - 43))
                    .build();
        }

        if (isStopRequested()) return;

        // Choose a path according to the spike position
        if (position == spike_position.RIGHT) {

            // Execute right path

            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(-90));
            Claws.openRightClaw();
            sleep(200);
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            drive.turn(Math.toRadians(180));
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), -150, false)) && opModeIsActive()) {
                Arm.moveUp(ARM_SPEED, 1);
            }
            Arm.brake();
            Wrist.setPosition(0.71);
            sleep(500);
            drive.followTrajectory(traj2);
            Claws.closeRightClaw();
            Wrist.moveUp();
            sleep(300);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_DOWN_POSITION, true)) && opModeIsActive()) {
                Arm.moveDown(ARM_SPEED, 1);
            }
            Arm.brake();
            sleep(500);
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
            Wrist.setPosition(0.4);
            sleep(200);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_UP_POSITION-50, false)) && opModeIsActive()) {
                Arm.moveUp(ARM_SPEED, 1);
            }
            Arm.brake();
            sleep(800);
            Claws.openLeftClaw();
            Claws.openRightClaw();
            sleep(500);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_DOWN_POSITION, true)) && opModeIsActive()) {
                Arm.moveDown(ARM_SPEED, 1);
            }
            Arm.brake();
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            sleep(500);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);
        }
        else if (position == spike_position.LEFT) {

            // Execute left path

            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(-90));
            Claws.openRightClaw();
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            drive.turn(Math.toRadians(90));
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), -150, false)) && opModeIsActive()) {
                Arm.moveUp(ARM_SPEED, 1);
            }
            Arm.brake();
            Wrist.setPosition(0.71);
            sleep(500);
            drive.followTrajectory(traj2);
            Claws.closeRightClaw();
            Wrist.moveUp();
            sleep(300);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_DOWN_POSITION, true)) && opModeIsActive()) {
                Arm.moveDown(ARM_SPEED, 1);
            }
            Arm.brake();
            sleep(500);
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
            Wrist.setPosition(0.4);
            sleep(200);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_UP_POSITION-50, false)) && opModeIsActive()) {
                Arm.moveUp(ARM_SPEED, 1);
            }
            Arm.brake();
            sleep(800);
            Claws.openLeftClaw();
            Claws.openRightClaw();
            sleep(500);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_DOWN_POSITION, true)) && opModeIsActive()) {
                Arm.moveDown(ARM_SPEED, 1);
            }
            Arm.brake();
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            sleep(500);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);
        }
        else {

            // Execute Center path

            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(-90));
            Claws.openRightClaw();
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            drive.turn(Math.toRadians(90));
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), -150, false)) && opModeIsActive()) {
                Arm.moveUp(ARM_SPEED, 1);
            }
            Arm.brake();
            Wrist.setPosition(0.71);
            sleep(500);
            drive.followTrajectory(traj2);
            Claws.closeRightClaw();
            Wrist.moveUp();
            sleep(300);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_DOWN_POSITION, true)) && opModeIsActive()) {
                Arm.moveDown(ARM_SPEED, 1);
            }
            Arm.brake();
            sleep(500);
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
            Wrist.setPosition(0.4);
            sleep(200);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_UP_POSITION-50, false)) && opModeIsActive()) {
                Arm.moveUp(ARM_SPEED, 1);
            }
            Arm.brake();
            sleep(800);
            Claws.openLeftClaw();
            Claws.openRightClaw();
            sleep(500);
            while (!(Arm.arrivedPosition(Arm.getArm1Position(), ARM_DOWN_POSITION, true)) && opModeIsActive()) {
                Arm.moveDown(ARM_SPEED, 1);
            }
            Arm.brake();
            Wrist.setPosition(Wrist.WRIST_UP_POSITION);
            sleep(500);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);
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
        Camera.init(this, hardwareMap, 2);
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
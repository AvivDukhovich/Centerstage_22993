//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Autonomous (name = "Red Back")
//
//public class AutonomousRB extends LinearOpMode {
//    spike_position position;
//    public int phase;
//    static double SLOW_SPEED = 0.2;
//    static double FAST_SPEED = 0.5;
//
//    static double ARM_SPEED = 0.6;
//
//    int PHASE_1_L = 690; // Forward
//    int PHASE_2_L = -510; // Rotate left
//    int PHASE_3_L = 0; // Left (slow)
//    int PHASE_5_L = -200; // Right
//    int PHASE_7_L = -670; // Right
//    int PHASE_8_L = 30; // Forward (slow)
//    int PHASE_9_L = -20; // Rotate left (slow)
//    int PHASE_10_L = -2350; // Arm up
//    int PHASE_12_L = Arm.MINIMAL_HOLD_POSITION; // Arm down
//    int PHASE_13_L = -1000; // Backward
//    int PHASE_14_L = -500; // Right
//
//    int PHASE_1_C = 670; // Forward
//    int PHASE_3_C = -5; // Backward
//    double PHASE_4_C = 0.45; // Wrist position
//    int PHASE_5_C = -520; // Rotate left
//    int PHASE_6_C = -890; // Right
//    int PHASE_7_C = 310; // Forward
//    int PHASE_8_C = -2350 ; // Arm up
//    int PHASE_10_C = Arm.MINIMAL_HOLD_POSITION; // Arm down
//    int PHASE_11_C = -780; // Backward
//    int PHASE_12_C = -550; // Right
//
//    int PHASE_1_R = 100; // Forward
//    int PHASE_2_R = 650; // Right
//    int PHASE_3_R = -530; // Rotate left
//    int PHASE_4_R = 150; // Forward
//    int PHASE_6_R = -100; // Right
//    int PHASE_8_R = -40; // Backward (slow)
//    int PHASE_9_R = -2350; // Arm up
//    int PHASE_11_R = Arm.MINIMAL_HOLD_POSITION; // Arm down
//    int PHASE_12_R = -300; // Backward
//    int PHASE_13_R = -400; // Right
//
//    enum spike_position {
//        LEFT,
//        RIGHT,
//        CENTER
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        // Initialize everything
//        initArm();
//        initClaws();
//        initWrist();
//        initEgnitionSystem();
//        initCamera();
//        initLed();
//        phase = 1;
//
//        // Close the claws
//        Claws.closeRightClaw();
//        Claws.closeLeftClaw();
//
//        Wrist.setPosition(0);
//
//        // Find the spike with pixel and print in telemetry
//        while (opModeInInit()) {
//            if (PixelDetectorRB.getSpike_position() == 0) {
//                position = spike_position.LEFT;
//                HardwareLocal.green();
//            }
//            else if (PixelDetectorRB.getSpike_position() == 1) {
//                position = spike_position.CENTER;
//                HardwareLocal.green();
//            }
//            else {
//                position = spike_position.RIGHT;
//                HardwareLocal.green();
//            }
//
//            telemetry.addData("Spike Position: ", position);
//            telemetry.addData("Right Region avg: ", Camera.getRightRegion_avg(1));
//            telemetry.addData("Left Region avg: ", Camera.getLeftRegion_avg(1));
//            telemetry.update();
//        }
//        Camera.close(1);
//
//        waitForStart();
//
//        // move the wrist down
//        Wrist.setPosition(Wrist.WRIST_DOWN_POSITION-0.2);
//        sleep(700);
//
//        while (opModeIsActive()) {
//
//            // Choose a path according to the spike position
//            if (position == spike_position.RIGHT) {
//                Right();
//            }
//            else if (position == spike_position.LEFT) {
//                Left();
//            }
//            else {
//                Center();
//            }
//
//            // Adjust the wrist position according to the arm position
//            if (Arm.getArm1Position() <= Arm.UNLOADING_POSITION) {
//                Wrist.setPosition(Wrist.WRIST_UNLOADING_POSITION + 0.018 * ((int) ((Arm.getArm1Position() - Arm.UNLOADING_POSITION) / -50)));
//            }
//
//            // Move the Egnition system
//            EgnitionSystem.updateVariablesAutonomous();
//            EgnitionSystem.runAutonomous();
//
//            // Add data to telemetry
//            telemetry.addData("Arm1 encoder: ", Arm.getArm1Position());
//            telemetry.addData("FL encoder position: ", EgnitionSystem.getFlEncoderPosition());
//            telemetry.update();
//        }
//    }
//    public void Left() {
//
//        if (phase == 1) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_1_L, true)) {
//                EgnitionSystem.setVerticalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(1);
//            }
//        }
//        else if (phase == 2) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_2_L, false)) {
//                EgnitionSystem.setRotPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setRotPower(-1);
//            }
//        }
//        else if (phase == 3) {
////            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_3_L, true)) {
////                EgnitionSystem.setHorizontalPower(0);
////                EgnitionSystem.setAutonomousMovingPower(FAST_SPEED);
////                sleep(500);
////                EgnitionSystem.resetEncoders();
////                phase ++;
////            }
////            else {
////                EgnitionSystem.setHorizontalPower(-1);
////                EgnitionSystem.setAutonomousMovingPower(SLOW_SPEED);
////            }
//            phase ++;
//        }
//        else if (phase == 4) {
//            Claws.openRightClaw();
//            phase++;
//        }
//        else if (phase == 5) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_5_L, false)) {
//                EgnitionSystem.setHorizontalPower(0);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//        else if (phase == 6) {
//            Wrist.moveUp();
//            phase ++;
//        }
//        else if (phase == 7) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_7_L, false)) {
//                EgnitionSystem.setHorizontalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//        else if (phase == 8) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_8_L, true)) {
//                EgnitionSystem.setVerticalPower(0);
//                EgnitionSystem.setAutonomousMovingPower(FAST_SPEED);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(1);
//                EgnitionSystem.setAutonomousMovingPower(SLOW_SPEED);
//            }
//        }
//        else if (phase == 9) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_9_L, false)) {
//                EgnitionSystem.setRotPower(0);
//                EgnitionSystem.setAutonomousMovingPower(FAST_SPEED);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setRotPower(-1);
//                EgnitionSystem.setAutonomousMovingPower(SLOW_SPEED-0.15);
//            }
//        }
//        else if (phase == 10) {
//            if (Arm.arrivedPosition(Arm.getArm1Position(), PHASE_10_L, false)) {
//                Arm.brake();
//                sleep(1800);
//                phase ++;
//            }
//            else {
//                Arm.moveUp(ARM_SPEED);
//            }
//        }
//        else if (phase == 11) {
//            Claws.openLeftClaw();
//            phase ++;
//            sleep(500);
//        }
//        else if (phase == 12) {
//            if (Arm.arrivedPosition(Arm.getArm1Position(), PHASE_12_L, true)) {
//                Arm.brake();
//                phase ++;
//            }
//            else {
//                Arm.moveDown(ARM_SPEED);
//            }
//        }
//        else if (phase == 13) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_13_L, false)) {
//                EgnitionSystem.setVerticalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(-1);
//            }
//        }
//        else if (phase == 14) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_14_L, false)) {
//                EgnitionSystem.setHorizontalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//    }
//    public void Center() {
//
//        if (phase == 1) {
//            Wrist.setPosition(Wrist.WRIST_DOWN_POSITION-0.05);
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_1_C, true)) {
//                EgnitionSystem.setVerticalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(1);
//            }
//        }
//        else if (phase == 2) {
//            Claws.openRightClaw();
//            phase ++;
//        }
//        else if (phase == 3) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_3_C, false)) {
//                EgnitionSystem.setVerticalPower(0);
//                EgnitionSystem.setAutonomousMovingPower(FAST_SPEED);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(-1);
//                EgnitionSystem.setAutonomousMovingPower(SLOW_SPEED);
//            }
//        }
//        else if (phase == 4) {
//            Wrist.setPosition(PHASE_4_C);
//            phase ++;
//        }
//        else if (phase == 5) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_5_C, false)) {
//                EgnitionSystem.setRotPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setRotPower(-1);
//            }
//        }
//        else if (phase == 6) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_6_C, false)) {
//                EgnitionSystem.setHorizontalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//        else if (phase == 7) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_7_C, true)) {
//                EgnitionSystem.setVerticalPower(0);
//                EgnitionSystem.setAutonomousMovingPower(FAST_SPEED);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(1);
//                EgnitionSystem.setAutonomousMovingPower(SLOW_SPEED);
//            }
//        }
//        else if (phase == 8) {
//            if (Arm.arrivedPosition(Arm.getArm1Position(), PHASE_8_C, false)) {
//                Arm.brake();
//                phase ++;
//            }
//            else {
//                Arm.moveUp(ARM_SPEED);
//            }
//        }
//        else if (phase == 9) {
//            sleep(1500);
//            Claws.openLeftClaw();
//            phase++;
//            sleep(500);
//        }
//        else if (phase == 10) {
//            if (Arm.arrivedPosition(Arm.getArm1Position(), PHASE_10_C, true)) {
//                Arm.brake();
//                phase ++;
//            }
//            else {
//                Arm.moveDown(ARM_SPEED);
//            }
//        }
//        else if (phase == 11) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_11_C, false)) {
//                EgnitionSystem.setVerticalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(-1);
//            }
//        }
//        else if (phase == 12) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_12_C, false)) {
//                EgnitionSystem.setHorizontalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//    }
//    public void Right () {
//        if (phase == 1) {
//            Wrist.setPosition(Wrist.WRIST_DOWN_POSITION - 0.05);
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_1_R, true)) {
//                EgnitionSystem.setVerticalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(1);
//            }
//        }
//        else if (phase == 2) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_2_R, true)) {
//                EgnitionSystem.setHorizontalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//        else if (phase == 3) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_3_R, false)) {
//                EgnitionSystem.setRotPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setRotPower(-1);
//            }
//        }
//        else if (phase == 4) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_4_R, true)) {
//                EgnitionSystem.setVerticalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(1);
//            }
//        }
//        else if (phase == 5) {
//            Claws.openRightClaw();
//            phase++;
//        }
//        else if (phase == 6) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_6_R, false)) {
//                EgnitionSystem.setHorizontalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//        else if (phase == 7) {
//            Wrist.moveUp();
//            phase++;
//        }
//        else if (phase == 8) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_8_R, false)) {
//                EgnitionSystem.setVerticalPower(0);
//                EgnitionSystem.setAutonomousMovingPower(FAST_SPEED);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setAutonomousMovingPower(SLOW_SPEED);
//                EgnitionSystem.setVerticalPower(-1);
//            }
//        }
//        else if (phase == 9) {
//            if (Arm.arrivedPosition(Arm.getArm1Position(), PHASE_9_R, false)) {
//                Arm.brake();
//                phase ++;
//            }
//            else {
//                Arm.moveUp(ARM_SPEED);
//            }
//        }
//        else if (phase == 10) {
//            sleep(1500);
//            Claws.openLeftClaw();
//            sleep(500);
//            phase++;
//        }
//        else if (phase == 11) {
//            if (Arm.arrivedPosition(Arm.getArm1Position(), PHASE_11_R, true)) {
//                Arm.brake();
//                phase ++;
//            }
//            else {
//                Arm.moveDown(ARM_SPEED);
//            }
//        }
//        else if (phase == 12) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_12_R, false)) {
//                EgnitionSystem.setVerticalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setVerticalPower(-1);
//            }
//        }
//        else if (phase == 13) {
//            if (EgnitionSystem.arrivedPosition(EgnitionSystem.getFlEncoderPosition(), PHASE_13_R, false)) {
//                EgnitionSystem.setHorizontalPower(0);
//                sleep(500);
//                EgnitionSystem.resetEncoders();
//                phase ++;
//            }
//            else {
//                EgnitionSystem.setHorizontalPower(1);
//            }
//        }
//    }
//    public void initClaws(){
//        Servo left_claw = hardwareMap.get(Servo.class, "left_claw");
//        Servo right_claw = hardwareMap.get(Servo.class, "right_claw");
//        Claws.init(left_claw, right_claw);
//    }
//    public void initWrist() {
//        Servo servo = hardwareMap.get(Servo.class, "wrist");
//        Wrist.init(servo);
//    }
//    public void initArm() {
//        DcMotor motor = hardwareMap.get(DcMotor.class, "rightArm");
//        DcMotor motor2 = hardwareMap.get(DcMotor.class, "leftArm");
//        Arm.init(motor, motor2);
//        Arm.addDataToTelemetry(telemetry);
//    }
//    public void initEgnitionSystem() {
//        DcMotor fl_wheel = hardwareMap.get(DcMotor.class, "leftFront");
//        DcMotor fr_wheel = hardwareMap.get(DcMotor.class, "rightFront");
//        DcMotor bl_wheel = hardwareMap.get(DcMotor.class, "leftBack");
//        DcMotor br_wheel = hardwareMap.get(DcMotor.class, "rightBack");
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//
//        EgnitionSystem.init(fl_wheel, fr_wheel, bl_wheel, br_wheel, imu);
//        EgnitionSystem.initEncoders();
//    }
//    public void initCamera() {
//        Camera.init(this, hardwareMap, 1);
//    }
//    public void initLed() {
//        RevBlinkinLedDriver ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDrive");
//        HardwareLocal.init(ledDriver);
//    }
//}
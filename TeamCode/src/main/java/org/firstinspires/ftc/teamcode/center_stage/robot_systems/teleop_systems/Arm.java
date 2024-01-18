package org.firstinspires.ftc.teamcode.center_stage.robot_systems.teleop_systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private DcMotor arm;

    private final double ARM_POWER = 0.1;

    private int hold_position = 0;
    private boolean got_position_to_hold = false;

    private double arm_next_position = 0;
    public Arm(DcMotor motor) {
        this.arm = motor;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveUp() {
        got_position_to_hold = false;
        arm_next_position = arm.getCurrentPosition() + 0.1;
        arm.setTargetPosition((int)arm_next_position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveDown() {
        got_position_to_hold = false;
        arm_next_position = arm.getCurrentPosition() - 0.1;
        arm.setTargetPosition((int)arm_next_position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void brake(){
        if (!got_position_to_hold) {
            hold_position = arm.getCurrentPosition();
            got_position_to_hold = true;
        }
        arm.setTargetPosition(hold_position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public void addDataToTelemetry(Telemetry telemetry) {
        telemetry.addData("Arm position: ", arm.getCurrentPosition());
        telemetry.addData("Arm hold position: ", hold_position);
        telemetry.update();
    }

}
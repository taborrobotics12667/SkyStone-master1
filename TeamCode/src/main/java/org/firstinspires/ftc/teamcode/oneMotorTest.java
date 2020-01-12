package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDriveTrain;

import java.util.ArrayList;

@TeleOp
public class oneMotorTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        DcMotor motor = hardwareMap.get(DcMotor.class, "Motor 1");
        waitForStart();

        while (opModeIsActive()) {
            double power = Range.clip(gamepad1.left_stick_y, -0.05, 0.05);
            motor.setPower(power);








        }
    }
}
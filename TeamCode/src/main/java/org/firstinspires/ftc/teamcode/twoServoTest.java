package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDriveTrain;

import java.util.ArrayList;

@TeleOp
@Disabled
public class twoServoTest extends LinearOpMode {

    private Servo servo;


    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "Servo 1");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(1);
            } else if (gamepad1.b) {
                servo.setPosition(0);
            }
        }



    }
}

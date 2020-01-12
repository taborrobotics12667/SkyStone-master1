package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDriveTrain;

import java.util.ArrayList;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    private double maxPower1 = 0.7;
    private double maxPower2 = 0.7;

    private int conditional = 1;

    private double maxPower = maxPower1 * conditional;

    private MecanumDriveTrain driveTrain;
    private Mechanisms mechanisms;


    @Override
    public void runOpMode() {

        driveTrain = new MecanumDriveTrain(hardwareMap, "Motor 1", "Motor 2", "Motor 3", "Motor 4");
        mechanisms = new Mechanisms(hardwareMap, "Flip", "Servo 1");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mechanisms.foundation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            double y = Range.clip(gamepad1.right_stick_y, -maxPower, maxPower);
            double x = Range.clip(gamepad1.right_stick_x, -maxPower, maxPower);

            double rot = Range.clip(gamepad1.left_stick_x, -0.5, 0.5);
            if (rot != 0){
                driveTrain.rotate(rot);
            }
            else {
                driveTrain.setPower((float)x,(float)y);
            }
            double flipMotorUp = Range.clip(gamepad1.left_trigger, -0.2, 0.2);
            double flipMotorDown = -Range.clip(gamepad1.right_trigger, -0.2, 0.2);
            if (flipMotorUp != 0){
                mechanisms.setFlipPower(flipMotorUp);
            }
            if (flipMotorDown != 0){
                mechanisms.setFlipPower(flipMotorDown);
            }

            if(gamepad1.a){
                mechanisms.flipCapDown();
            }
            else if(gamepad1.b){
                mechanisms.flipCapUp();
            }

            telemetry.addData("Positions: ",driveTrain.getPositions());
            telemetry.update();



        }
    }
}
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms {

    double flipMin = 0;
    double flipMax = 1;
    double target2;
    double target;
    Servo capFlip;
    DcMotor foundation;

    public Mechanisms(HardwareMap hardwareMap, String DcMotorName, String ServoName){
        foundation = hardwareMap.get(DcMotor.class, DcMotorName);
        capFlip = hardwareMap.get(Servo.class, ServoName);
        foundation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        foundation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foundation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void flipCapUp(){
        target = flipMin;
        capFlip.setPosition(target);
    }
    public void flipCapDown(){
        target = flipMax;
        capFlip.setPosition(target);
    }
    public void flipMotorUp(){
        int targetPosition = foundation.getCurrentPosition() + 250;
        foundation.setTargetPosition(targetPosition);
    }
    public void flipMotorDown(){
        int targetPosition = foundation.getCurrentPosition() - 250;
        foundation.setTargetPosition(targetPosition);

    }
    public void setFlipPower (double power){
        foundation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foundation.setPower(power);
    }
    public int getPosition(){
        int position = foundation.getCurrentPosition();
        return position;
    }
}
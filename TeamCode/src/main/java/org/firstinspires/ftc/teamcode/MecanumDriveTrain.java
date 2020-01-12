package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class MecanumDriveTrain  {

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    int[] right  = {1,3};
    int[] left   = {0,2};
    int[] diagTL = {0,3};
    int[] diagTR = {1,2};
    int [] posList = new int [4];

    public  MecanumDriveTrain(HardwareMap hardwareMap,String ... motorNames){
        for (int i = 0;i<motorNames.length;i++){
            DcMotor currentMotor = hardwareMap.get(DcMotor.class,motorNames[i]);
            motors.add(currentMotor);
            currentMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            currentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for(int index:left){
            motors.get(index).setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void setPower(float x, float y){
        double theta = java.lang.Math.atan2(y,x) - 3.14159/4;
        double magnitude = java.lang.Math.sqrt(x*x+y*y);
        double newX = java.lang.Math.cos(theta)*magnitude;
        double newY = java.lang.Math.sin(theta)*magnitude;
        for (int index:diagTL){
            motors.get(index).setPower(newX);
        }
        for (int index: diagTR){
            motors.get(index).setPower(newY);
        }
    }
    public void setMode(DcMotor.RunMode mode){
        for (DcMotor motor:motors) {
            motor.setMode(mode);
        }
    }
    public void rotate(double power){
        for (int index:right){
            motors.get(index).setPower(-power);
        }
        for(int index:left){
            motors.get(index).setPower(power);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotor motor:motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public String getPositions(){
        int i = 0;
        for (DcMotor motor:motors) {

            int pos = motor.getCurrentPosition();
            posList[i] = pos;
            i ++;
        }
        return String.valueOf(posList[0]) + ' ' + String.valueOf(posList[1]) + ' ' + String.valueOf(posList[2]) + ' ' + String.valueOf(posList[3]);
    }

}
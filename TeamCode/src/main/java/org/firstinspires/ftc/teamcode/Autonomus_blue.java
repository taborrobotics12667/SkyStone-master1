/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.sql.Driver;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Autonomus_blue")
public class Autonomus_blue extends LinearOpMode {

    /* Declare OpMode members. */

    private MecanumDriveTrain robot;
    private Mechanisms mechanisms;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.25 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 1.295 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot = new MecanumDriveTrain(hardwareMap, "Motor 1", "Motor 2", "Motor 3", "Motor 4");   // Use a Pushbot's hardware
        mechanisms = new Mechanisms(hardwareMap,"Flip","Servo 1" );
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoders", robot.getPositions());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED, -1400, -1400, -1400, -1400, 4, false);
        //encoderFlip(0.2, -1000, 3);

        encoderDrive(DRIVE_SPEED,  492,  -495, -499, 503, 2, false);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(0.3, -889, -1907, -1892, -904, 5, false);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderFlip(0.2, -1000, 3);
        encoderDrive(DRIVE_SPEED, -51, -593,-610, -85, 4,true);
        encoderDrive(DRIVE_SPEED, -299, 473, 377, -312, 4,true);
        encoderDrive(DRIVE_SPEED, 47, 1397, 1256, 20, 4,true);
        encoderDrive(DRIVE_SPEED, -448, 1907, 1764, -470, 4,true);
        encoderDrive(DRIVE_SPEED, -1867, 285, 152, -1911, 4,true);
        encoderDrive(DRIVE_SPEED, -1788, 2179, 1985, -1871, 4, true);
        encoderDrive(DRIVE_SPEED, -3373, 557, 382, -3498, 4, true);
        encoderDrive(DRIVE_SPEED, -4373, -443, -618, -4498, 5, true);
        encoderFlip(0.2, 0, 3);
        sleep(5000);
        encoderDrive(DRIVE_SPEED, -6373, -2443, -2618, -6498, 4, false);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             int rightFrontTarget, int leftFrontTarget, int rightBackTarget, int leftBackTarget,
                             double timeoutS, boolean czech) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.motors.get(0).setTargetPosition(leftFrontTarget);
            robot.motors.get(1).setTargetPosition(rightFrontTarget);
            robot.motors.get(2).setTargetPosition(leftBackTarget);
            robot.motors.get(3).setTargetPosition(rightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motors.get(0).setPower(Math.abs(speed));
            robot.motors.get(1).setPower(Math.abs(speed));
            robot.motors.get(2).setPower(Math.abs(speed));
            robot.motors.get(3).setPower(Math.abs(speed));
            if (czech){
                mechanisms.foundation.setPower(-0.2);
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motors.get(0).isBusy() && robot.motors.get(1).isBusy() && robot.motors.get(2).isBusy() && robot.motors.get(3).isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", leftFrontTarget, rightFrontTarget, leftBackTarget, rightBackTarget);
                telemetry.addData("Path2",  robot.getPositions());
                telemetry.update();
            }

            // Stop all motion;
            robot.setPower(0,0);

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderFlip(double speed, int flipTarget,
                            double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            mechanisms.foundation.setTargetPosition(flipTarget);

            // Turn On RUN_TO_POSITION
            mechanisms.foundation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            mechanisms.foundation.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mechanisms.foundation.isBusy())) {


            }

            // Stop all motion;
            mechanisms.foundation.setPower(0);

            // Turn off RUN_TO_POSITION
            mechanisms.foundation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}

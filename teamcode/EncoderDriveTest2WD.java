/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="EncoderDriveTest-2WD", group="Calibration")  // @Autonomous(...) is the other common choice
//@Disabled
public class EncoderDriveTest2WD extends LinearOpMode
{


    //--1-begin
    DcMotor wheelR;
    DcMotor wheelL;
    //--1-end

    DcMotor wheelR2;
    DcMotor wheelL2;

    DcMotor motors[];


    @Override
    public void runOpMode() {


        //--1-begin
        wheelR = hardwareMap.dcMotor.get("motorR");
        wheelL = hardwareMap.dcMotor.get("motorL");

        wheelR2 = hardwareMap.dcMotor.get("motorR2");
        wheelL2 = hardwareMap.dcMotor.get("motorL2");

        wheelR.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        wheelR2.setDirection(DcMotorSimple.Direction.REVERSE);
        //--2-begin

        motors = new DcMotor[2];
        motors[0] = wheelL;
        motors[1] = wheelR;


        for (DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        wheelL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        boolean a_now;
        boolean a_then = false;
        double inches = 0;
        while(!isStarted() && !isStopRequested()){
            a_now = gamepad1.a;
            if (a_now && !a_then){
                inches += 1;
            }
            a_then = a_now;
            telemetry.addData("inches: ", inches);
            telemetry.update();
        }

        wheelL2.setPower(0);
        wheelR2.setPower(0);


        encoderDrive(inches);
        encoderDrive(-inches);

    }
    public void encoderDrive(double inches){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        if (opModeIsActive()) {
            for (DcMotor motor : motors){
                motor.setTargetPosition(motor.getCurrentPosition() - counts);// yeah... -, not +
            }
            for (DcMotor motor : motors){
                motor.setPower(0.25);
            }
            while (opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy())){
                telemetry.addData("Target position: ", inches);
                telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                telemetry.update();
            }
            for (DcMotor motor : motors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }
    public void s(double time, ElapsedTime r){
        double t = r.milliseconds();
        while (opModeIsActive() && r.milliseconds() <= t + time){}
    }
}
// 1120 cpr on the 40 gearmotor
// seems to consistently run 3/4 inch short of the target per 2 feet, or one inch 32 inches. to compensate: multiply by 32/31.
// I am going to use 33/32 because 32/31 is ugly. 33/32 - 32/31 = 1/992, approximately 0.001, or negligible.
// Error in measurement would be greater than 1/40 inch.
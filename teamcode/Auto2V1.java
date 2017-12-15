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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="2V1 safe zone", group="Autonomous")  // @Autonomous(...) is the other common choice
@Disabled
public class Auto2V1 extends LinearOpMode
{


    //--1-begin
    DcMotor wheelR;
    DcMotor wheelL;
    //--1-end

    //--5V2-begin
    DcMotor wheelR2;
    DcMotor wheelL2;
    //--5V2-end

    //--2-begin
    Servo leftServo;
    Servo rightServo;
    //--2-end

    //--4-begin
    DcMotor lift;
    //--4-end

    //--5-begin
    TouchSensor touchSensor;
    //--5-end


    DcMotor someMotors[] = new DcMotor[2];
    DcMotor moreMotors[] = new DcMotor[4];



    @Override
    public void runOpMode() {

        //--1-begin
        wheelR = hardwareMap.dcMotor.get("motorR");
        wheelL = hardwareMap.dcMotor.get("motorL");

        //--1-begin
        wheelR2 = hardwareMap.dcMotor.get("motorR2");
        wheelL2 = hardwareMap.dcMotor.get("motorL2");

        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        wheelL2.setDirection(DcMotorSimple.Direction.REVERSE);
        //--2-begin

        leftServo = hardwareMap.servo.get("left");
        rightServo = hardwareMap.servo.get("right");

        //--6-begin



        someMotors[0] = wheelL;
        someMotors[1] = wheelR;

        moreMotors[0] = wheelL;
        moreMotors[1] = wheelR;
        moreMotors[2] = wheelL2;
        moreMotors[3] = wheelR2;

        for (DcMotor motor : moreMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        int motorPosition = 0;
        //--6-end

        double LEFT_CENTER = 0.52; //tweak this
        double RIGHT_CENTER = 0.38; //tweak this
        double LEFT_IN = 1.0; //tweak this
        double RIGHT_IN = 0.0; //tweak this
        double LEFT_OUT = 0.32; //tweak this
        double RIGHT_OUT = 0.6; //tweak this //done
        //--2-end

        //--4-begin
        telemetry.addLine("Getting lift motor...");
        telemetry.update();
        lift = hardwareMap.dcMotor.get("lift");

        double UP_SPEED = -0.5; //tweak this
        double DOWN_SPEED = 0.8;
        //--4-end


        //--5-begin
        touchSensor = hardwareMap.touchSensor.get("touchUp");

        wheelL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        waitForStart();
        ElapsedTime r = new ElapsedTime();

        lift.setPower(UP_SPEED);

        s(800, r);
        lift.setPower(0);

        encoder4WDrive(42);

    }


    public void s(double time, ElapsedTime r){
        double t = r.milliseconds();
        while (opModeIsActive() && r.milliseconds() <= t + time){}
    }

    public void encoderDrive(double inches){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        if (opModeIsActive()) {
            for (DcMotor motor : someMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0.25);
            }
            while (opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy())){
                telemetry.addData("Target position: ", inches);
                telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                telemetry.update();
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }

    public void encoder4WDrive(double inches){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);

        if (opModeIsActive()) {
            for (DcMotor motor : moreMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0.25);
            }
            while (opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy() || wheelL2.isBusy() || wheelR2.isBusy())){
                telemetry.addData("Target position: ", inches);
                telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                telemetry.update();
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }

    public void encoderTurn(double degrees){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 280/pi counts per inch
        // turn diameter is 13.75 inches, so a 360 degree turn is 13.75pi inches
        // so one degree is 13.75pi/360 = 55pi/1440 = 11pi/288
        // 280/pi counts per inch times 11pi/288 inches per degree is 11*280/288 counts per degree
        // 11*280/288 = 11*35/36 = 385/36
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 33.0/32.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        if (opModeIsActive()) {
            wheelL.setTargetPosition(wheelL.getCurrentPosition() - counts);
            wheelR.setTargetPosition(wheelR.getCurrentPosition() + counts);
            // should go counterclockwise
            for (DcMotor motor : someMotors){
                motor.setPower(0.25);
            }
            while (opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy())){
                telemetry.addData("Target position: ", counts);
                telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                telemetry.update();
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());
        }
    }


}
// one second per foot at 0.2 power 
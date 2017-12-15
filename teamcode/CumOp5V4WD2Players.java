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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import gamepad
//import telemetry

/*
 * CumOp4: adds lift functionality
 */

@TeleOp(name="CumOp5V2-4WD-2Players", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class CumOp5V4WD2Players extends LinearOpMode
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

    DcMotor jewelDisplacer;

    //--5-begin
    TouchSensor touchSensor;
    //--5-end

    @Override
    public void runOpMode() {


        //--1-begin
        wheelR = hardwareMap.dcMotor.get("motorR");
        wheelL = hardwareMap.dcMotor.get("motorL");

        //--1-begin
        wheelR2 = hardwareMap.dcMotor.get("motorR2");
        wheelL2 = hardwareMap.dcMotor.get("motorL2");

        wheelR.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
	    wheelR2.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        //--2-begin

        leftServo = hardwareMap.servo.get("left");
        rightServo = hardwareMap.servo.get("right");

        jewelDisplacer = hardwareMap.dcMotor.get("jeweler");


        double LEFT_CENTER = 0.56; //tweak this
        double RIGHT_CENTER = 0.35; //tweak this
        double LEFT_SLIGHT_OUT = 0.47;
        double RIGHT_SLIGHT_OUT = 0.48;
        double LEFT_IN = 0.99; //tweak this
        double RIGHT_IN = 0.0; //tweak this
        double LEFT_OUT = 0.34; //tweak this
        double RIGHT_OUT = 0.53; //tweak this //done
        //--2-end

        int JEWEL_OUT = -580; //tweak this
        int JEWEL_IN = 0; //tweak this

        //--4-begin
        telemetry.addLine("Getting lift motor...");
        telemetry.update();
        lift = hardwareMap.dcMotor.get("lift");

        double UP_SPEED = -0.5; //tweak this
        double DOWN_SPEED = 0.8;
        //--4-end


        //--5-begin
        touchSensor = hardwareMap.touchSensor.get("touchUp");

        jewelDisplacer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jewelDisplacer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        jewelDisplacer.setTargetPosition(JEWEL_IN);
        jewelDisplacer.setPower(0.25);

        while (opModeIsActive()) {


            float direction;
            if (gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_x <= -0.1) {
                direction = -gamepad1.left_stick_x;
            } else if (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1) {
                direction = 0.2F * -gamepad1.right_stick_x;
            } else {
                direction = 0;
            }
            //--1-begin
            float throttle;
            if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1){
                throttle = gamepad1.left_stick_y;
            } else if ( gamepad1.right_stick_y >= 0.1 || gamepad1.right_stick_y <= 0.1){
                throttle = 0.2F * gamepad1.right_stick_y;
            } else {
                throttle = 0;
            }
            float right = throttle - direction;
            float left = throttle + direction;
            wheelR.setPower(right);
            wheelL.setPower(left);
            wheelR2.setPower(right);
            wheelL2.setPower(left);

            //--1-end

            //--3-begin
            //grab
            if (gamepad2.x) {
                leftServo.setPosition(LEFT_CENTER);
                rightServo.setPosition(RIGHT_CENTER);
            } else if (gamepad2.b) { //release
                leftServo.setPosition(LEFT_OUT);
                rightServo.setPosition(RIGHT_OUT);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down){
                leftServo.setPosition(LEFT_IN);
                rightServo.setPosition(RIGHT_IN);
            }
            //--3-end

            //--4-begin
            if (gamepad2.a) {
                lift.setPower(DOWN_SPEED);
            } else if (gamepad2.y && (!touchSensor.isPressed())) {
                lift.setPower(UP_SPEED);
            } else {
                lift.setPower(0);
            }
            //--4-end

            if (gamepad2.left_trigger > 0.5){
                jewelDisplacer.setTargetPosition(JEWEL_IN);
            } else if (gamepad2.right_trigger > 0.5){
                jewelDisplacer.setTargetPosition(JEWEL_OUT);
            }

            telemetry.addData("Touch sensor: ", touchSensor.isPressed());
            telemetry.update();

        }
    }
}


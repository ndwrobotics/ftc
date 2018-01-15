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
//import gamepad
//import telemetry

/*
 * CumOp4: adds lift functionality
 */

@TeleOp(name="Mark's program", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class MarksDanceProgram extends LinearOpMode
{
    Selectron bot = null;
    @Override
    public void runOpMode() {

        boolean danceTime = false;
        bot = new Selectron(this);

        waitForStart();
        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_IN);
        bot.jewelDisplacer.setPower(0.25);

        while(opModeIsActive()) {
            while (opModeIsActive() && !danceTime) {
                float direction;
                if (gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_x <= -0.1) {
                    direction = gamepad1.left_stick_x;
                } else if (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1) {
                    direction = 0.2F * gamepad1.right_stick_x;
                } else {
                    direction = 0;
                }
                //--1-begin
                float throttle;
                if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
                    throttle = -gamepad1.left_stick_y;
                } else if (gamepad1.right_stick_y >= 0.1 || gamepad1.right_stick_y <= 0.1) {
                    throttle = 0.2F * -gamepad1.right_stick_y;
                } else {
                    throttle = 0;
                }
                float right = throttle - direction;
                float left = throttle + direction;
                bot.wheelR.setPower(right);
                bot.wheelL.setPower(left);
                bot.wheelR2.setPower(right);
                bot.wheelL2.setPower(left);

                //--1-end

                //--3-begin
                //grab
                if (gamepad1.x) {
                    bot.leftServo.setPosition(bot.LEFT_CENTER);
                    bot.rightServo.setPosition(bot.RIGHT_CENTER);
                } else if (gamepad1.b) { //release
                    bot.leftServo.setPosition(bot.LEFT_OUT);
                    bot.rightServo.setPosition(bot.RIGHT_OUT);
                } else if (gamepad1.dpad_right) {
                    bot.leftServo.setPosition(bot.LEFT_IN);
                    bot.rightServo.setPosition(bot.RIGHT_IN);
                } else if (gamepad1.dpad_left) {
                    bot.leftServo.setPosition(bot.LEFT_SLIGHT_OUT);
                    bot.rightServo.setPosition(bot.RIGHT_SLIGHT_OUT);
                }
                //--3-end

                //--4-begin
                if (gamepad1.a) {
                    bot.lift.setPower(bot.DOWN_SPEED);
                } else if (gamepad1.y && (!bot.touchSensor.isPressed())) {
                    bot.lift.setPower(bot.UP_SPEED);
                } else {
                    bot.lift.setPower(0);
                }
                //--4-end

                if (gamepad1.left_trigger > 0.5) {
                    bot.jewelDisplacer.setTargetPosition(bot.JEWEL_IN);
                } else if (gamepad1.right_trigger > 0.5) {
                    bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
                }

                telemetry.addData("Touch sensor: ", bot.touchSensor.isPressed());
                telemetry.update();

                if (danceChange()) {
                    danceTime = true;
                }
            }

            while(opModeIsActive() && danceChange()){
            }


            while (opModeIsActive() && danceTime) {

                float direction;
                float throttle;
                double right;
                double left;
                if (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1 || gamepad1.right_stick_y >= 0.1 || gamepad1.right_stick_y <= -0.1) {

                    direction = gamepad1.right_stick_x;
                    throttle = -gamepad1.right_stick_y;

                    right = throttle - direction;
                    left = throttle + direction;

                    if(gamepad1.right_trigger > 0.4){
                        right *= 0.2;
                        left *= 0.2;
                    }
                } else {
                    right = 0;
                    left = 0;
                }
                //--1-begin

                bot.wheelR.setPower(right);
                bot.wheelL.setPower(left);
                bot.wheelR2.setPower(right);
                bot.wheelL2.setPower(left);


                //grab
                if(gamepad1.right_trigger < 0.4) {
                    if (gamepad1.x) {
                        bot.leftServo.setPosition(bot.LEFT_CENTER);
                        bot.rightServo.setPosition(bot.RIGHT_CENTER);
                    } else if (gamepad1.b) { //release
                        bot.leftServo.setPosition(bot.LEFT_OUT);
                        bot.rightServo.setPosition(bot.RIGHT_OUT);
                    }
                } else {
                    //--3-end
                    if (gamepad1.x) {
                        bot.leftServo.setPosition(bot.LEFT_IN);
                        bot.rightServo.setPosition(bot.RIGHT_IN);
                    } else if (gamepad1.b) {
                        bot.leftServo.setPosition(bot.LEFT_SLIGHT_OUT);
                        bot.rightServo.setPosition(bot.RIGHT_SLIGHT_OUT);
                    }
                    //--4-begin
                }

                if (gamepad1.a) {
                    bot.lift.setPower(bot.DOWN_SPEED);
                } else if (gamepad1.y && (!bot.touchSensor.isPressed())) {
                    bot.lift.setPower(bot.UP_SPEED);
                } else {
                    bot.lift.setPower(0);
                }
                //--4-end

                //if (gamepad1.left_trigger > 0.5) {
                //    bot.jewelDisplacer.setTargetPosition(bot.JEWEL_IN);
                //} else if (gamepad1.right_trigger > 0.5) {
                //    bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
                //}

                telemetry.addData("Touch sensor: ", bot.touchSensor.isPressed());
                telemetry.update();

                if (danceChange()) {
                    danceTime = false;
                }
            }


            while(opModeIsActive() && danceChange()){
            }
        }
    }

    public boolean danceChange(){
        return ((gamepad1.left_trigger > 0.8) && (gamepad1.right_trigger > 0.8) && gamepad1.dpad_left && gamepad1.b);
    }
}


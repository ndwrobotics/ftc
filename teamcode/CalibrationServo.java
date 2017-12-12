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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Servo Calibration", group="Calibration")  // @Autonomous(...) is the other common choice
//@Disabled
public class CalibrationServo extends LinearOpMode
{


    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.servo.get("servo");
        double servoPosition = 0.5;

        boolean x_then = false;
        boolean x_now;
        boolean y_then = false;
        boolean y_now;
        boolean a_then = false;
        boolean a_now;
        boolean b_then = false;
        boolean b_now;

        waitForStart();

        while (opModeIsActive()){
            a_now = gamepad1.a;
            b_now = gamepad1.b;
            x_now = gamepad1.x;
            y_now = gamepad1.y;
            if (a_now && !a_then) {
                servoPosition += 0.1;
            }
            if (b_now && !b_then) {
                servoPosition -= 0.1;
            }
            if (x_now && !x_then) {
                servoPosition += 0.01;
            }
            if (y_now && !y_then) {
                servoPosition -= 0.001;
            }
            telemetry.addData("servo position: ", servoPosition);
            telemetry.update();
            servo.setPosition(servoPosition);
            a_then = a_now;
            b_then = b_now;
            x_then = x_now;
            y_then = y_now;

        }
    }

}
// one second per foot at 0.2 power
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * CumOp1: adds drive functionality
 */

@TeleOp(name="CumOp1", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class CumOp1 extends LinearOpMode
{

    //--1-begin
    DcMotor wheelR;
    DcMotor wheelL;
    //--1-end

    @Override
    public void runOpMode() {


        //--1-begin
        telemetry.addLine("Getting drive motors...");
        telemetry.update();
        sleep(500);//-----------------------------------------------------------------------------------------
        wheelR = hardwareMap.dcMotor.get("motorR");
        wheelL = hardwareMap.dcMotor.get("motorL");

        telemetry.addLine("Reversing left wheel direction...");
        telemetry.update();
        sleep(500);//-----------------------------------------------------------------------------------------
        wheelR.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        //--1-end


        waitForStart();
        telemetry.addLine("beginning loop");
        sleep(500);//-----------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //--1-begin
            float throttle = gamepad1.left_stick_y;
            float direction = -gamepad1.left_stick_x;
            float right = throttle - direction;
            float left = throttle + direction;
            wheelR.setPower(right);
            wheelL.setPower(left);
            //--1-end

        }
    }

}

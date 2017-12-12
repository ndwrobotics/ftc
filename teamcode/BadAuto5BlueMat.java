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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BadAuto5-BlueMat", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class BadAuto5BlueMat extends LinearOpMode
{

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        //--1-begin
        DcMotor wheelR;
        DcMotor wheelL;
        //--1-end

        //--2-begin
        Servo leftServo;
        Servo rightServo;
        //--2-end

        DcMotor lift;

        //--1-begin
        wheelR = hardwareMap.dcMotor.get("motorR");
        wheelL = hardwareMap.dcMotor.get("motorL");

        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        //--2-begin

        leftServo = hardwareMap.servo.get("left");
        rightServo = hardwareMap.servo.get("right");

        lift = hardwareMap.dcMotor.get("lift");

        double UP_SPEED = -0.5; //tweak this
        double DOWN_SPEED = 0.8; //tweak this //done


        double LEFT_CENTER = 0.26; //tweak this
        double RIGHT_CENTER = 0.2; //tweak this
        double LEFT_IN = 0.7; //tweak this
        double RIGHT_IN = 0.0; //tweak this
        double LEFT_OUT = 0; //tweak this
        double RIGHT_OUT = 0.65; //tweak this //done
        //--2-end


        telemetry.addLine("Code is running.");
        telemetry.update();



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "Aczwc7j/////AAAAGSh6oYrgWE/wvIHsaTUy6ppNpIawgvZu5xiQAUK+xtcW8Zw8P9GhxqBOsB2RKBAP6cOUkwCiyhCaYHKfrhA7ORLNRka9TTTm36bBHMY/WhR02K5Z5Qa2TaqRNjtpZ5rZ2Q8Ee+vnuiVasqj3uAMCB0ceFPYemYJ8snub+w/8gKVwy09n+ZWJ/5yymbJ9lGIQAwlc8Wo0HogRSK6Yk0Z0CiX/UFKhB+wK+I7Vzcwv0pZVOXAhf7fuDJOUr+TcuCBptm6AlFai2evVFuKT1seKaTsX+Sa2jvP3F0O1gbfLYK1dWjIh5/mqEp6yWgk2D+fAhGGWVjeUkN0ftstV6F1U06+f9P52sOd6w0Fuagpm27TP";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        leftServo.setPosition(LEFT_CENTER);
        rightServo.setPosition(RIGHT_CENTER);

        waitForStart();


        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (vuMark == RelicRecoveryVuMark.UNKNOWN) && runtime.seconds() <= 20){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark, ", "%s visible", vuMark);
            telemetry.update();
        }

        lift.setPower(UP_SPEED);

        s(800, runtime);
        lift.setPower(0);


        double timeToDrive;
        if (vuMark == RelicRecoveryVuMark.RIGHT){
            timeToDrive = 1300;
        } else if (vuMark == RelicRecoveryVuMark.LEFT){
            timeToDrive = 750;
        } else {
            timeToDrive = 1000;
        }


        wheelL.setPower(0.2);
        wheelR.setPower(0.2);

        s(timeToDrive, runtime);

        wheelL.setPower(0);
        wheelR.setPower(0);
        s(500, runtime);
        wheelL.setPower(-0.2);
        wheelR.setPower(0.2);


        if (vuMark == RelicRecoveryVuMark.LEFT){
            s(800, runtime);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            s(550, runtime);
        } else {
            s(500, runtime);
        }
        wheelL.setPower(0);
        wheelR.setPower(0);

        s(500, runtime);

        wheelL.setPower(0.2);
        wheelR.setPower(0.2);
        s(900, runtime);
        wheelL.setPower(0);
        wheelR.setPower(0);

        leftServo.setPosition(LEFT_OUT);
        rightServo.setPosition(RIGHT_OUT);

        s(500, runtime);

        wheelL.setPower(-0.2);
        wheelR.setPower(-0.2);
        s(500, runtime);
        wheelL.setPower(0);
        wheelR.setPower(0);


        lift.setPower(-UP_SPEED);
        s(800, runtime);
        lift.setPower(0);

        leftServo.setPosition(LEFT_IN);
        rightServo.setPosition(RIGHT_IN);
        s(500, runtime);

        wheelL.setPower(0.2);
        wheelR.setPower(0.2);
        s(500, runtime);
        wheelL.setPower(0);
        wheelR.setPower(0);
        s(500, runtime);
        wheelL.setPower(-0.2);
        wheelR.setPower(-0.2);
        s(500, runtime);
        wheelL.setPower(0.2);
        s(400, runtime);
        wheelR.setPower(0.2);
        if (vuMark == RelicRecoveryVuMark.RIGHT){
            s(250, runtime);
        } else if (vuMark == RelicRecoveryVuMark.LEFT){
            s(300, runtime);
        } else {
            s(350, runtime);
        }
        wheelL.setPower(0);
        wheelR.setPower(0);

    }

    public void s(double time, ElapsedTime r){
        double t = r.milliseconds();
        while (opModeIsActive() && r.milliseconds() <= t + time){}
    }
}
// one second per foot at 0.2 power

//right:  turn a little less
//left:   turn less
//center: turn a little less
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Newer Encoder Test", group="test")
@Disabled
public class NewerEncoderTest extends LinearOpMode {
    Selectron bot = null;
    ElapsedTime r = null;
    @Override
    public void runOpMode() {
        bot = new Selectron(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        r = new ElapsedTime();

        waitForStart();
        bot.setEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        while(opModeIsActive()){

            bot.newEncoderDrive(24, 0.25);
            bot.newEncoderTurn(90, 0.25);
            bot.newEncoderTurn(-90, 0.25);
            bot.newEncoderDrive(-24, 0.25);
        }
    }
}
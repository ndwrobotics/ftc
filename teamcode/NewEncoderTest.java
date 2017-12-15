package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="New Encoder Test", group="test")
public class NewEncoderTest extends LinearOpMode {
    Selectron bot = null;
    ElapsedTime r = null;
    @Override
    public void runOpMode() {
        bot = new Selectron(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        r = new ElapsedTime();

        waitForStart();
        while(opModeIsActive()){
            bot.newAndImprovedEncoderDrive(24, 0.25, 0);
            bot.newAndImprovedEncoderTurn(90, 0.25, 0);
            bot.newAndImprovedEncoderTurn(-90, 0.25, 0);
            bot.newAndImprovedEncoderDrive(-24, 0.25, 0);
        }
    }
}
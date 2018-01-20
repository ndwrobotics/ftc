package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="4V2 -Red Not Mat", group="Autonomous")
public class Auto4V2RedNotMat extends LinearOpMode {
    SelectronV3 bot = null;
    boolean RED_ALLIANCE = true;

    @Override
    public void runOpMode() {
        bot = new SelectronV3(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        bot.resetJewelArm();

        ElapsedTime r = new ElapsedTime();

        waitForStart();

        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
        bot.jewelDisplacer.setPower(0.25);

        bot.autoRaiseLift();

        bot.displaceJewel(bot.COLOR_THRESHOLD, RED_ALLIANCE);

        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_IN);

        bot.square_up(r, RED_ALLIANCE);
        bot.jewelDisplacer.setPower(0);

        bot.encoderDrive(11);
        bot.encoderTurn(45);
        bot.encoderDrive(8);
        bot.timeDrive(400, true, r);
        bot.release();
        bot.s(250, r);
        bot.encoderDrive(-8);
        bot.autoLowerLift();
        bot.completely_close();
        bot.encoderDrive(8);
        bot.encoderDrive(-12);
        bot.encoderTurn(45);
        bot.encoderDrive(7.5);

    }
}
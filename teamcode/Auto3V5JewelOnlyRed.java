package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="3V1 jewel only -Red", group="test")
public class Auto3V5JewelOnlyRed extends LinearOpMode {
    Selectron bot = null;
    boolean RED_ALLIANCE = true;

    @Override
    public void runOpMode() {
        bot = new Selectron(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
        bot.jewelDisplacer.setPower(0.25);
        while(bot.jewelDisplacer.isBusy() && opModeIsActive()){}

        bot.displaceJewel(bot.COLOR_THRESHOLD, RED_ALLIANCE);

        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_IN);
        while(bot.jewelDisplacer.isBusy() && opModeIsActive()){}

    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="4V1 jewel only -Red", group="test")
//@Disabled
public class Auto4V1JewelOnlyRed extends LinearOpMode {
    SelectronV3 bot = null;
    boolean RED_ALLIANCE = true;

    @Override
    public void runOpMode() {
        bot = new SelectronV3(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        bot.resetJewelArm();

        waitForStart();


        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
        bot.jewelDisplacer.setPower(0.25);
        while(bot.jewelDisplacer.isBusy() && opModeIsActive()){}

        bot.displaceJewel(bot.COLOR_THRESHOLD, RED_ALLIANCE);

        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_IN);
        while(bot.jewelDisplacer.isBusy() && opModeIsActive()){}
    }
}
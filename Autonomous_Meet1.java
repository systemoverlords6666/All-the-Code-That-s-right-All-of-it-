package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous_Meet1_Use_this_on_Sat_27")
public class Autonomous_Meet1 extends LinearOpMode {

    DcMotor lm;
    DcMotor rm;
    DcMotor up;

    @Override
    public void runOpMode() throws InterruptedException {
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");

        rm.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        lm.setPower(.4);
        rm.setPower(.4);
        sleep(3000);

        lm.setPower(-.4);
        rm.setPower(-.4);
        sleep(3000);

        lm.setPower(.4);
        rm.setPower(-.4);
        sleep(3000);

        lm.setPower(0);
        rm.setPower(0);

    }
}


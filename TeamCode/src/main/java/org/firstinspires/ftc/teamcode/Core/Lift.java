package org.firstinspires.ftc.teamcode.Core;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libs.Classes.GameController;
import org.firstinspires.ftc.teamcode.hardware.GameController;
@TeleOp(name = Lift)
public class Lift {

    Lift lift;
    @Override
    protected void preInitialize() {
        super.preInitialize();
        isTeleOp = true;
    }
    @Override
    protected void initialize() {
        Lift = new Lift(HardwareMap);
    }

    @Override
    protected void run(double dt) {

        playerController.onButtonTap(GameController.Button.DPAD_UP, () ->
                lift.up()
        );
        playerController.onButtonTap(GameController.Button.DPAD_DOWN, () ->
                lift.down()
        );
    }
}

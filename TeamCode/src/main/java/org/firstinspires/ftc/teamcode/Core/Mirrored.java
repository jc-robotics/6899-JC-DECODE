package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Mirrored {

    DcMotor Mr, Ml;
    @Override
protected void initialize() {
    Mr = HardwareMap.get(DcMotorEx.class, "MirrorRight");
    Ml = HardwareMap.get(DcMotorEx.class, "MirrorLeft");
}
    @Override
    protected void run(double dt) {
        Mr.setPower(1);
        Ml.setPower(1);
    }
}

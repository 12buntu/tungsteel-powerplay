package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Claw extends SubsystemBase {
    private final SimpleServo clawServo;

    private boolean isOpen = true;

    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 1.0;

    private static final double OPEN = 0.75;
    private static final double CLOSED = 1.0;

    public Claw(HardwareMap hwMap) {
        clawServo = new SimpleServo(hwMap, "claw", MIN_ANGLE, MAX_ANGLE);
        clawServo.turnToAngle(OPEN);
    }

    public void toggle() {
        isOpen = !isOpen;
        double angle = isOpen ? OPEN : CLOSED;
        clawServo.turnToAngle(angle);
    }
}

//MediumAutomaton 2023, for FTC22Robot / PowerNap
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Manipulator {
    //Public members (manipulator components)
    public ServoWrap clawServo;
    public ServoWrap armServo;
    public ElapsedTime toggleTimer;
    public boolean clawFlag;
    public boolean armFlag;

    public Manipulator(ServoWrap clawOriginal, ServoWrap armOriginal) {
        clawServo = clawOriginal;
        armServo = armOriginal;
        toggleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        openClaw();
        clawFlag = true;
        armFront();
        armFlag = true;
    }

    public void openClaw() {
        if (!armServo.isBusy()) {
            clawServo.setPosition(0.05);
        }
    }

    public void closeClaw() {
        if (!armServo.isBusy()) {
            clawServo.setPosition(0.25);
        }
    }

    public void toggleClaw() {
        if (toggleTimer.milliseconds() > 500 && !armServo.isBusy()) {
            if (clawFlag) {
                closeClaw();
            } else {
                openClaw();
            }
            toggleTimer.reset();
            clawFlag = !clawFlag;
        }
    }

    public void armFront() {
        if (!clawServo.isBusy()) {
            armServo.setPosition(0.07);
        }
    }

    public void armRear() {
        if (!clawServo.isBusy()) {
            armServo.setPosition(0.75);
        }
    }

    public void toggleArm() {
        if (toggleTimer.milliseconds() > 500 && !clawServo.isBusy()) {
            if (armFlag) {
                armRear();
            } else {
                armFront();
            }
            toggleTimer.reset();
            armFlag = !armFlag;
        }
    }
}

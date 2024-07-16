package org.firstinspires.ftc.teamcode.fy23.gamepad2.specific.imudrivetuner;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** Stores the state of the controls. The control scheme updates this, then the OpMode reads it. This effectively maps
 * buttons to actions. */
public class IMUDriveTunerState {

    private DTS dts = new DTS();
    private boolean pUp = false;
    private boolean pDown = false;
    private boolean imUp = false;
    private boolean imDown = false;
    private boolean dmUp = false;
    private boolean dmDown = false;
    private boolean changeUp = false;
    private boolean changeDown = false;
    private boolean hdgUp = false;
    private boolean hdgDown = false;
    private boolean save = false;

    public boolean pUp() {
        return pUp;
    }

    public void setpUp(boolean pUp) {
        this.pUp = pUp;
    }

    public boolean pDown() {
        return pDown;
    }

    public void setpDown(boolean pDown) {
        this.pDown = pDown;
    }

    public boolean imUp() {
        return imUp;
    }

    public void setImUp(boolean imUp) {
        this.imUp = imUp;
    }

    public boolean imDown() {
        return imDown;
    }

    public void setImDown(boolean imDown) {
        this.imDown = imDown;
    }

    public boolean dmUp() {
        return dmUp;
    }

    public void setDmUp(boolean dmUp) {
        this.dmUp = dmUp;
    }

    public boolean dmDown() {
        return dmDown;
    }

    public void setDmDown(boolean dmDown) {
        this.dmDown = dmDown;
    }

    public boolean changeUp() {
        return changeUp;
    }

    public void setChangeUp(boolean changeUp) {
        this.changeUp = changeUp;
    }

    public boolean changeDown() {
        return changeDown;
    }

    public void setChangeDown(boolean changeDown) {
        this.changeDown = changeDown;
    }

    public boolean hdgUp() {
        return hdgUp;
    }

    public void setHdgUp(boolean hdgUp) {
        this.hdgUp = hdgUp;
    }

    public boolean hdgDown() {
        return hdgDown;
    }

    public void setHdgDown(boolean hdgDown) {
        this.hdgDown = hdgDown;
    }

    public boolean save() {
        return save;
    }

    public void setSave(boolean save) {
        this.save = save;
    }

    public DTS getDts() {
        return dts;
    }

    public void setDts(DTS dts) {
        this.dts = dts;
    }


}

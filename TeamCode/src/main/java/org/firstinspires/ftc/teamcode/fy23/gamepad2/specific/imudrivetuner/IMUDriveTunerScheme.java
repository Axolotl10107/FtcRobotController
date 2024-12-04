package org.firstinspires.ftc.teamcode.fy23.gamepad2.specific.imudrivetuner;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.Button;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives.buttons.TriggerButton;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** A controller scheme for driving with independent drive, turn, and strafe axes and providing
 * additional buttons for IMUDriveTuner. */
public class IMUDriveTunerScheme {

    private Gamepad driver;
    private Gamepad manipulator;

    private IMUDriveTunerState state;

    private Button pUp;
    private Button pDown;
    private Button imUp;
    private Button imDown;
    private Button dmUp;
    private Button dmDown;
    private Button changeUp;
    private Button changeDown;
    private Button hdgUp;
    private Button hdgDown;
    private Button save;

    public IMUDriveTunerScheme(Gamepad driver, Gamepad manipulator) {
        this.driver = driver;
        this.manipulator = manipulator;

        state = new IMUDriveTunerState();

        pUp = new TriggerButton( () -> driver.dpad_up );
        pDown = new TriggerButton( () -> driver.dpad_down );
        imUp = new TriggerButton( () -> driver.y );
        imDown = new TriggerButton( () -> driver.a );
        dmUp = new TriggerButton( () -> driver.b );
        dmDown = new TriggerButton( () -> driver.x );
        changeUp = new TriggerButton( () -> driver.start );
        changeDown = new TriggerButton( () -> driver.back );
        hdgUp = new TriggerButton( () -> driver.dpad_right );
        hdgDown = new TriggerButton( () -> driver.dpad_left );
        save = new TriggerButton( () -> driver.right_bumper );
    }

    private void updateMovementState() {
        double drive = driver.right_trigger - driver.left_trigger;
        double turn = -driver.left_stick_x; // positive turn is counterclockwise
        double strafe = driver.right_stick_x;
        state.setDts(new DTS(drive, turn, strafe));
    }

    public IMUDriveTunerState getState() {
        updateMovementState();

        state.setpUp(pUp.isActive());
        state.setpDown(pDown.isActive());
        state.setImUp(imUp.isActive());
        state.setImDown(imDown.isActive());
        state.setDmUp(dmUp.isActive());
        state.setDmDown(dmDown.isActive());
        state.setChangeUp(changeUp.isActive());
        state.setChangeDown(changeDown.isActive());
        state.setHdgUp(hdgUp.isActive());
        state.setHdgDown(hdgDown.isActive());
        state.setSave(save.isActive());

        return state;
    }

}

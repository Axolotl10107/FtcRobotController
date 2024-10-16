package org.firstinspires.ftc.teamcode.fy24;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArmImpl;

public class PixelArmImplDynamic extends PixelArmImpl {
    final int tickInchRatio = 100;

    public PixelArmImplDynamic(Parameters parameters) {
        super(parameters);
    }

    @Override
    public void update() {
        super.update();

        int currentPosition = getPivotPosition(); // Hypotenuse adjacent angle
        int armPosition = getElevatorPosition(); // Hypotenuse
        if(currentPosition < 49.5) {
            setElevatorUpperLimit((int) (tickInchRatio * (Math.floor(Math.hypot(Math.tan(currentPosition) * 36, 36)))));
        } else if (currentPosition > 49.5) {
            setElevatorUpperLimit((int) Math.floor(55.5 * tickInchRatio));
        }

        // Find motor tick to inch ratio, currently set to 100.
    }
}

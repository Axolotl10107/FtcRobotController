package org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy24;

/** With this interface, you only need to change one thing to change control schemes: the constructor.
 * Also, anything special about a particular scheme (i.e. fieldy schemes needing the current heading) is passed into the
 * constructor, and getState() (and all your code that uses it) is still the same. */
public interface TeleOpScheme24 {
    TeleOpState24 getState();
}

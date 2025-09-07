package org.firstinspires.ftc.teamcode.fy25.ctlpad;

/** With this interface, you only need to change one thing to change control schemes: the constructor.
 * Also, anything special about a particular scheme (i.e. fieldy schemes needing the current heading) is passed into the
 * constructor, and getState() (and all your code that uses it) is still the same. */
public interface StarterBotScheme25 {
    StarterBotState25 getState();
}

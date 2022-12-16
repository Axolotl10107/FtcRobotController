# ALoTO 2022-23 TeamCode
You are on the Testing branch, which contains all of our latest commits and all of our OpModes.
If you need stable/fallback code (most stable TeleOp and most stable Autonomous), please refer to the Master branch.


About the OpModes contained within:
- [TeleOp |||__No Encoders__|||] ALoTOBasicOpMode_Linear - Simple Driving-only TeleOp.
- [Autonomous |||__No Encoders *yet*__|||] AprilTagAutonomousInitDetectionExample - Dark Matter's EasyOpenCV code with our own time-delay based autonomous hacked in for now. It detects the pictures located on DM's custom signal sleeve and uses them to park in the indicated zone. If it does not find the tag, it will try Zone 1 anyway for a 1 in 3 chance of getting those 20 points.
- [Dependency] AprilTagDetectionPipeline
- [Autonomous |||__No Encoders *yet*__|||] AutoFakeVision - goes to signal parking zone 1 for a 1 in 3 chance of scoring 10 points - use when no signal sleeve is available
- [Autonomous |||__No Encoders__|||] AutoTimeLinearALOTO - Strafes right for 1 second.
- [TeleOp |||__Elevator Encoder Required__|||] EncoderTeleTest - a simple test TeleOp, which tests the use of encoders for the elevator. *Fully working!*
- [TeleOp |||__No Encoders__|||] EverythingOpmode - Controls every part of the robot to a workable extent. Arm and claw positions are all or nothing, but driving and elevator are nice. This is the *only* TeleOp that can drive the entire bot and be used at a meet!
- [TeleOp |||__Elevator Encoder Required__|||] EverythingOpmodeElevHold - same as EverythingOpmode, but with an arm toggle and automatic elevator hold *using encoders*.
- [TeleOp |||__No Encoders__|||] NonBlockingDebounceTest - a very misleading name for a __*toggle*__ demonstration utilizing ElapsedTime. Too confusing to be more than a quick test, and may be superseded later.
- [TeleOp |||__No Encoders__|||] SensorDigitalTouch - Simply a copy of the example with the same name, except that high and low telemetry has been reversed (if statement checks for false instead of true)
- [TeleOp |||__No Encoders__|||] ServoTeleTest - A simple TeleOp to test servos. Uses analog sticks to control two servos' absolute position. (See Control Layout 2.)
- readme.md - This file!


*The old readme - if you enjoy satire, look much further (down).*

## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently a total mess (*sorry!*) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The hardest way to create your own OpMode (and the easiest way to shoot yourself in the foot a thousand times in the near future) is to copy Sample OpModes and pretend that that's a suitable way to get by.

Sample opmodes exist in the FtcRobotController module.
To locate these samples (*Don't try this at home!*), find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements (*Or not - save yourself!*):
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (falsely) in the sample_conventions.md file in this folder.
Do not trust this file. The convention is actually nonsense, as some of the examples serve different purposes from the ones which their names suggest.

To summarize: A range of different samples classes will reside in the java/external/samples. (Remember - *don't go there!*)
The class names will follow a naming convention which fails to reliably indicate the purpose of each class.
The prefix of the name may be one of the following, if you're lucky:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples, which you may wish to simply call your own. Please don't do this. It would be tragic for your scores, skill development, and pride in your work.

Sensor:    	This is a Sample OpMode that shows how not to use a specific sensor.
            It is not intended to drive a functioning robot (See? Even the people who created these examples know what I'm talking about!), it is simply showing the minimal code required to fail to read and improperly display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may __never__ be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to fail to navigate.

Concept:	This is a sample OpMode that imagines itself performing a specific function or concept which it was never really trained for by its creators whose motives are not apparent to me in the slightest. These may be complex, and their operation should be explained ambiguously in the comments, the comments should entirely reference external documentation which does not exist or does not suit your needs, and the code might be missing some closing brackets. Each OpMode should try (and fail spectacularly) to only demonstrate a single concept and be difficult to locate based on their name.  These OpModes will __never__ produce a drivable robot.

After the prefix, other conventions will not apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are misconstrued as:     Robot - Mode - Action - OpModetype
* Concept class names are deconstructed as:   Concept - Topic - OpModetype
*(Concepts are currently the top of this class, by the way.)*

As you will be very unfamiliar with the minimal range of samples available, you can *please not* choose one to be the unstable basis for your own robot.  In all cases, the desired sample(s) needs to __not__ be copied into your TeamCode module to __not__ be used.

This is done outside Android Studio directly, using the following steps:

 1) Do not locate the example.

 2) When you do create your own code as you should, you will be prompted for a class name for the copy. Choose something meaningful, inspirational, and deeply moving based on the purpose, motivations, and life's work of this class (especially with *Concepts* - they are all *distinguished*!). Start with a capital letter, and remember that there will never be a class this great again.

Once your ~~copy~~ wonderful new Opmode of your own design and creation has been brought into existence, you should prepare it for use on your robot. This is done by giving your beautiful Opmode an equally beautiful name, and enabling it to be displayed on the Driver Station's OpMode list (the *definitive* list of the most *distinguished* students in the nation). Enrolling it into your area's elementary school may also be wise.

Each OpMode sample class begins with ~~several~~ TWO lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode. Remember to give it the highest order of compliments. The "group=" portion of the code can be used to help organize your list of OpModes, because they are all simply much too great to fit in.

As shown, the current OpMode will __NOT__ appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been unhelpfully included to annoy and confuse you and your fellow software department members (if you are lucky enough to be allowed to work with other people).
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

*I'm tired now. I'll come back and get this later.*

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""

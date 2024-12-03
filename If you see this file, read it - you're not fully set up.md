## I installed IDEA instead of Android Studio and it's not working!

There's a bit of additional setup involved before IDEA will load Android projects properly:

- Install the Android plugin
  - In Settings, search for "android", and it should take you to the Plugins screen. Select the "Marketplace" tab there,
    and you should see the Android plugin at the top of the list. Install it and restart IDEA.
- Install the Android SDK
  - Go back to Settings and search again for "android". Look in the __Languages and Frameworks__ section.
    You're looking for a page called either "Android SDK Updater" or just "Android SDK". Click the "Edit" link toward the top right.
    You should now see the SDK setup wizard.
- Finish the SDK setup wizard
- Go to the Gradle pane and click the leftmost button in the toolbar - "Reload All Gradle Projects".
- You should now be able to build TeamCode! If you install the REV Hardware Client (Windows only), you can also launch it on a real Control Hub.


## I want to test my code at home!

You can't test everything without a real robot, but you can do quite a bit with [virtual_robot](https://github.com/Beta8397/virtual_robot).
We're still figuring out how exactly that's going to work, so we'll add instructions here when we've set it up the way we want.
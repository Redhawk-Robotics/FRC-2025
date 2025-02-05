# What did I do today?
Please share!

Today I worked on swerve and got advantagekit working with our robot. I figured out how to get it 
to deploy with swerve (the setting we had for our IMU was deprecated) and closed the issue I 
made for it on Friday. I made a new issue because we are unable to retrieve any telementary
data from our absolute encoders, which are making the modules spin. When I simulate it on
advantage scope, the desired module states are (mostly) correct, but it looks like the actual live state for each model can't identify when they match up with the right angle.

Opened an issue for this persisting problem, and comitted changes that fix the following
- The sparkmax ID configs
- The CANCODER ID Configs

The "front" of the robot is now considered to be where the roborio.
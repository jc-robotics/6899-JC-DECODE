## TeamCode Core

Welcome!

This module, Core, **is shared between the two operational modes** and is for the core components and functions of the bot
This is all related to custom written functions designed to be used in the other two operations and **should be clearly documented or atleast readable**

#### ie.
- drive train
- sensors
- motors
- servos
- and anything used in the init phase should probably be included unless better judgement proves otherwise

## btw forget everything


# CONTROLS:

- **Left Stick** controls the movement of the bot ie. hold left strafe left and hold forward move forward
- **Right Stick** only controls the turning with moving the stick left and right to rotate the whole bot
**- NB. hold Left Bumper to move the bot more precisely without LERP and the input power is halved**

- **Tapping A** spins the guns back with a third power for cases to keep the ball from moving too far up
- **Right Trigger** controls the guns themselves using a gear system for greater consistency, ie the gun steps up its power by 0.2 every fifth of the way the right trigger gets lower
**- NB. hold Right Bumper to disable the gear system and pass the raw Right trigger input to the guns, ie. for more precise control**

- **Press or Hold Y** to spin up the intake for intaking balls...
- **Releasing B** makes the intake spin up immediately for launch or fast pickup situations, *it may be unstable*

- **Press or Hold X** to spin the intake slightly but also raise lift to prime the gun for firing
- **Release X** to return lift to bottom

- **Vertical D Pad** to raise or lower the lift directly and slowly
- **Horizontal D Pad** to spin the intake directly

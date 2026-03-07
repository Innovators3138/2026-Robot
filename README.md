# 2026-Robot


### CAN ID Configuration ###
- CAN MOTORS

- intakeMotor: CAN ID 4
- backleftdrive CAN ID 5
- backleftangle CAN ID 6
- backrightdrive CAN ID 7
- backrightangle CAN ID 8
- frontleftdrive CAN ID 9
- frontleftangle CAN ID 10
- frontrightdrive CAN ID 11
- frontrightangle CAN ID 12
- hotdogMotor: CAN ID 13
- feederMotor: CAN ID 14
- climberMotor: CAN ID 15
- shooterMotorLeaderLeft: CAN ID 20
- shooterMotorFollowerLeft: CAN ID 21
- shooterMotorFollowerRightA: CAN ID 18
- shooterMotorFollowerRightB: CAN ID 19


- CAN DEVICES
- frontleftEncoder CAN ID 30
- frontrightEncoder CAN ID 31
- backleftEncoder CAN ID 32
- backrightEncoder CAN ID 33
- candle: CAN ID 36
- NAVX: CAN ID 37



- when added a new motor or Device, find an availiable CAN ID and add it to the list



### Analog Input ID Configuration ###


### Digital Input ###
- Extended Sensor: 1
- Retracted Sensor: 0


### Solenoids ###


### Drive Controls ###
- Left stick is field oriented drive
- Right Stick is robot rotation
- B is breaking
- A is activating/deactivating intake
- Dpad for climb
- Reset odometry is start
### Operater Controls ###
- Left trigger is auto aim and spinning up flywheel
- Right trigger is shoot
- Dpad is hood adjustment
- Hold A to agitate fuel
- Hold LB to engage the Flywheel
- Start to deploy hopper
### Branch Policy ###
 - When I start a feature, I will create a branch off of main. 
 - When I have completed my changes, I will submit a pull request to main.
 - I will have good comments, and good commit messages.
 - One other teammate must review my changes before committing to main.
 - All code should follow team guidelines.

### Tournament Coding Policy ###
 - Before each tournament, we will create a branch off of main that has a stable version of robot code.
 - Any changes to the robot code for the tournament will be pushed directly to that branch, bypassing PR requirements.

 ### Subystems ###
 - Vision: Quest 3
 - Drive: 8 motors, 8 encoders, NavX
 - intake: 1 motor
 - Hotdog Roller: 1 motor
 - Shooter: 4 motors, 1 encoder
 - Feeder: 2 motors
 - Adjustable Hood: 1 motor, 1 encoder
 - Climber: 1 motor, 1 encoder
 - LED: 1 CANdle
# FTC Robot Controller External Samples - File Understanding Guide

This document provides a comprehensive explanation of every file in the `external/samples` directory of the FTC Robot Controller. These are sample OpModes and utility classes designed to teach FTC programmers how to use various hardware components and programming patterns.

## Directory Structure

```
external/
└── samples/
    ├── externalhardware/
    │   ├── ConceptExternalHardwareClass.java
    │   └── RobotHardware.java
    ├── BasicOmniOpMode_Linear.java
    ├── BasicOpMode_Iterative.java
    ├── BasicOpMode_Linear.java
    ├── ConceptAprilTag.java
    ├── ConceptAprilTagEasy.java
    ├── ConceptAprilTagLocalization.java
    ├── ConceptAprilTagMultiPortal.java
    ├── ConceptAprilTagOptimizeExposure.java
    ├── ConceptAprilTagSwitchableCameras.java
    ├── ConceptBlackboard.java
    ├── ConceptExploringIMUOrientation.java
    ├── ConceptGamepadEdgeDetection.java
    ├── ConceptGamepadRumble.java
    ├── ConceptGamepadTouchpad.java
    ├── ConceptLEDStick.java
    ├── ConceptMotorBulkRead.java
    ├── ConceptNullOp.java
    ├── ConceptRampMotorSpeed.java
    ├── ConceptRevLED.java
    ├── ConceptRevSPARKMini.java
    ├── ConceptScanServo.java
    ├── ConceptSoundsASJava.java
    ├── ConceptSoundsOnBotJava.java
    ├── ConceptSoundsSKYSTONE.java
    ├── ConceptTelemetry.java
    ├── ConceptVisionColorLocator_Circle.java
    ├── ConceptVisionColorLocator_Rectangle.java
    ├── ConceptVisionColorSensor.java
    ├── RobotAutoDriveByEncoder_Linear.java
    ├── RobotAutoDriveByGyro_Linear.java
    ├── RobotAutoDriveByTime_Linear.java
    ├── RobotAutoDriveToAprilTagOmni.java
    ├── RobotAutoDriveToAprilTagTank.java
    ├── RobotAutoDriveToLine_Linear.java
    ├── RobotTeleopMecanumFieldRelativeDrive.java
    ├── RobotTeleopPOV_Linear.java
    ├── RobotTeleopTank_Iterative.java
    ├── SampleRevBlinkinLedDriver.java
    ├── SensorAndyMarkIMUNonOrthogonal.java
    ├── SensorAndyMarkIMUOrthogonal.java
    ├── SensorAndyMarkTOF.java
    ├── SensorBNO055IMU.java
    ├── SensorBNO055IMUCalibration.java
    ├── SensorColor.java
    ├── SensorDigitalTouch.java
    ├── SensorGoBildaPinpoint.java
    ├── SensorHuskyLens.java
    ├── SensorIMUNonOrthogonal.java
    ├── SensorIMUOrthogonal.java
    ├── SensorKLNavxMicro.java
    ├── SensorLimelight3A.java
    ├── SensorMRColor.java
    ├── SensorMRGyro.java
    ├── SensorMROpticalDistance.java
    ├── SensorMRRangeSensor.java
    ├── SensorOctoQuad.java
    ├── SensorOctoQuadAdv.java
    ├── SensorOctoQuadLocalization.java
    ├── SensorREV2mDistance.java
    ├── SensorSparkFunOTOS.java
    ├── SensorTouch.java
    ├── UtilityCameraFrameCapture.java
    ├── UtilityOctoQuadConfigMenu.java
    ├── readme.md
    └── sample_conventions.md
```

## File Naming Conventions

All sample files follow a naming convention that indicates their purpose:

- **Basic**: Minimally functional OpMode showing the skeleton/structure of a particular style (bare bones examples)
- **Sensor**: Shows how to use a specific sensor; not intended to drive a functioning robot
- **Robot**: Assumes a simple two-motor (differential) drive base; demonstrates driving or sensor integration
- **Concept**: Illustrates performing a specific function or concept; may be complex but operation is well-documented
- **Utility**: Helper classes or tools for specific tasks

---

## File Descriptions

### Documentation Files

#### `readme.md`
- **Purpose**: Provides overview and instructions for the samples directory
- **Key Info**: Explains that team-specific code should NOT be placed here; samples should be copied to `/TeamCode` folder
- **Contains**: Naming convention summary and guidance on how to interpret sample names

#### `sample_conventions.md`
- **Purpose**: Detailed documentation of the naming conventions used throughout the samples
- **Key Info**: Explains the structure of class names and how to interpret them
- **Contains**: Full breakdown of naming patterns for Sensor, Robot, and Concept classes

---

### Basic OpMode Templates

These are minimal, skeleton OpModes that show the basic structure of different OpMode styles.

#### `BasicOpMode_Linear.java`
- **Type**: Basic Linear OpMode
- **Purpose**: Minimal example of a Linear OpMode structure
- **Features**: 
  - Tank drive teleop for a two-wheeled robot
  - Shows basic motor initialization and control
  - Demonstrates POV (Point of View) driving mode
  - Includes telemetry output
- **Use Case**: Starting template for creating new Linear OpModes

#### `BasicOpMode_Iterative.java`
- **Type**: Basic Iterative OpMode
- **Purpose**: Minimal example of an Iterative OpMode structure
- **Features**:
  - Tank drive teleop for a two-wheeled robot
  - Shows init(), init_loop(), start(), loop(), and stop() methods
  - Demonstrates the iterative execution pattern
- **Use Case**: Starting template for creating new Iterative OpModes

#### `BasicOmniOpMode_Linear.java`
- **Type**: Basic Linear OpMode for Omni-directional drive
- **Purpose**: Minimal example for mecanum wheel robots
- **Features**:
  - Omni-directional (holonomic) drive control
  - Shows how to calculate motor powers for mecanum wheels
  - Includes strafe, forward, and rotation movements
- **Use Case**: Starting template for mecanum/omni-wheel robots

---

### AprilTag Vision Samples

AprilTags are fiducial markers used for robot localization and navigation. These samples show various ways to use them.

#### `ConceptAprilTag.java`
- **Purpose**: Basic AprilTag detection and pose estimation
- **Features**:
  - Detects AprilTags in camera view
  - Displays tag ID, position (XYZ), and orientation (Pitch/Roll/Yaw)
  - Shows how to initialize VisionPortal and AprilTagProcessor
  - Includes camera streaming control
- **Use Case**: Learning AprilTag detection basics

#### `ConceptAprilTagEasy.java`
- **Purpose**: Simplified AprilTag detection with minimal setup
- **Features**:
  - Easier-to-understand version of AprilTag detection
  - Reduced configuration options
  - Good for beginners
- **Use Case**: Quick start for AprilTag implementation

#### `ConceptAprilTagLocalization.java`
- **Purpose**: Using AprilTags for robot localization (determining robot position on field)
- **Features**:
  - Tracks robot position relative to AprilTags
  - Useful for autonomous navigation
  - Shows how to use tag positions for field localization
- **Use Case**: Autonomous programs that need to know robot position

#### `ConceptAprilTagMultiPortal.java`
- **Purpose**: Using multiple cameras/portals with AprilTags
- **Features**:
  - Demonstrates handling multiple vision portals
  - Can use multiple cameras simultaneously
  - Useful for robots with multiple camera views
- **Use Case**: Advanced vision setups with multiple cameras

#### `ConceptAprilTagOptimizeExposure.java`
- **Purpose**: Optimizing camera exposure for better AprilTag detection
- **Features**:
  - Shows how to adjust camera exposure settings
  - Improves detection in various lighting conditions
  - Demonstrates exposure control API
- **Use Case**: Improving AprilTag detection reliability

#### `ConceptAprilTagSwitchableCameras.java`
- **Purpose**: Switching between multiple cameras for AprilTag detection
- **Features**:
  - Demonstrates how to switch between different cameras
  - Useful for robots with front and back cameras
  - Shows camera selection logic
- **Use Case**: Robots that need to switch camera views

---

### Concept Samples - General Programming Concepts

#### `ConceptBlackboard.java`
- **Purpose**: Demonstrates the Blackboard pattern for data sharing between OpModes
- **Features**:
  - Shows how to share data across different OpModes
  - Useful for passing information between autonomous and teleop
  - Demonstrates static data storage
- **Use Case**: Sharing state between different program phases

#### `ConceptExploringIMUOrientation.java`
- **Purpose**: Understanding IMU (Inertial Measurement Unit) orientation and coordinate systems
- **Features**:
  - Explains different orientation representations (Euler angles, quaternions)
  - Shows how to read IMU data
  - Demonstrates coordinate system concepts
- **Use Case**: Learning IMU fundamentals

#### `ConceptGamepadEdgeDetection.java`
- **Purpose**: Detecting button press edges (transitions) on gamepad
- **Features**:
  - Shows how to detect when a button is pressed (not held)
  - Demonstrates edge detection pattern
  - Useful for toggle functionality
- **Use Case**: Implementing toggle buttons and one-time actions

#### `ConceptGamepadRumble.java`
- **Purpose**: Using gamepad rumble/vibration feedback
- **Features**:
  - Shows how to trigger controller vibration
  - Demonstrates different rumble patterns
  - Useful for driver feedback
- **Use Case**: Providing haptic feedback to drivers

#### `ConceptGamepadTouchpad.java`
- **Purpose**: Using touchpad input on compatible gamepads
- **Features**:
  - Shows how to read touchpad coordinates
  - Demonstrates touchpad detection
  - Useful for advanced input methods
- **Use Case**: Advanced gamepad input handling

#### `ConceptLEDStick.java`
- **Purpose**: Controlling LED stick hardware
- **Features**:
  - Shows how to control addressable LED strips
  - Demonstrates color and pattern control
  - Useful for robot status indication
- **Use Case**: Adding visual feedback with LEDs

#### `ConceptMotorBulkRead.java`
- **Purpose**: Efficiently reading multiple motor encoders in one operation
- **Features**:
  - Demonstrates bulk read optimization
  - Reduces communication overhead
  - Improves performance when reading many sensors
- **Use Case**: Performance optimization for encoder-heavy robots

#### `ConceptNullOp.java`
- **Purpose**: A do-nothing OpMode template
- **Features**:
  - Minimal OpMode that does nothing
  - Useful as a placeholder or starting point
- **Use Case**: Testing or as a blank template

#### `ConceptRampMotorSpeed.java`
- **Purpose**: Gradually ramping motor speed up and down
- **Features**:
  - Shows smooth acceleration/deceleration
  - Prevents sudden jerky movements
  - Demonstrates motion profiling basics
- **Use Case**: Smooth motor control

#### `ConceptRevLED.java`
- **Purpose**: Controlling REV Robotics LED hardware
- **Features**:
  - Shows how to control REV LED modules
  - Demonstrates color control
  - Useful for status indication
- **Use Case**: REV LED control

#### `ConceptRevSPARKMini.java`
- **Purpose**: Using REV SPARK Mini motor controller
- **Features**:
  - Shows how to configure and use SPARK Mini controllers
  - Demonstrates motor control with SPARK Mini
- **Use Case**: REV SPARK Mini motor control

#### `ConceptScanServo.java`
- **Purpose**: Scanning a servo through its full range of motion
- **Features**:
  - Shows how to move a servo smoothly through positions
  - Demonstrates servo control patterns
  - Useful for testing servo range
- **Use Case**: Servo testing and calibration

#### `ConceptSoundsASJava.java`
- **Purpose**: Playing sounds using Android Studio (Java) approach
- **Features**:
  - Shows how to play audio files from Java code
  - Demonstrates sound resource management
- **Use Case**: Adding audio feedback to OpModes

#### `ConceptSoundsOnBotJava.java`
- **Purpose**: Playing sounds using OnBot Java approach
- **Features**:
  - Shows how to play audio files in OnBot Java environment
  - Alternative to Android Studio approach
- **Use Case**: OnBot Java sound implementation

#### `ConceptSoundsSKYSTONE.java`
- **Purpose**: Playing sounds from the SKYSTONE season
- **Features**:
  - Demonstrates sound playback for SKYSTONE-specific audio
  - Shows season-specific resource handling
- **Use Case**: Historical reference for SKYSTONE season

#### `ConceptTelemetry.java`
- **Purpose**: Comprehensive guide to using the telemetry system
- **Features**:
  - Shows various telemetry output methods
  - Demonstrates formatting and organization
  - Shows how to display data to driver station
- **Use Case**: Learning telemetry best practices

#### `ConceptVisionColorSensor.java`
- **Purpose**: Using color sensors for vision-based detection
- **Features**:
  - Shows how to read RGB color values
  - Demonstrates color sensor calibration
  - Shows HSV color space usage
  - Includes light control and gain adjustment
- **Use Case**: Color-based object detection

#### `ConceptVisionColorLocator_Circle.java`
- **Purpose**: Detecting circular objects by color
- **Features**:
  - Uses color detection to find circular objects
  - Demonstrates shape-based vision
  - Shows how to locate objects in camera view
- **Use Case**: Finding circular game elements

#### `ConceptVisionColorLocator_Rectangle.java`
- **Purpose**: Detecting rectangular objects by color
- **Features**:
  - Uses color detection to find rectangular objects
  - Demonstrates shape-based vision
  - Shows how to locate objects in camera view
- **Use Case**: Finding rectangular game elements

---

### Robot Autonomous Drive Samples

These samples demonstrate autonomous driving using different methods for navigation.

#### `RobotAutoDriveByTime_Linear.java`
- **Purpose**: Autonomous driving based on elapsed time
- **Features**:
  - Drives forward for a specified duration
  - Simple but imprecise method
  - Good for basic autonomous movement
- **Use Case**: Simple autonomous routines

#### `RobotAutoDriveByEncoder_Linear.java`
- **Purpose**: Autonomous driving using motor encoders for distance
- **Features**:
  - Drives a specific distance using encoder counts
  - More precise than time-based driving
  - Demonstrates encoder-based movement
- **Use Case**: Accurate distance-based autonomous movement

#### `RobotAutoDriveByGyro_Linear.java`
- **Purpose**: Autonomous driving with gyro-based heading control
- **Features**:
  - Maintains straight heading using gyro sensor
  - Corrects for drift
  - Demonstrates gyro-based navigation
- **Use Case**: Straight-line autonomous movement with drift correction

#### `RobotAutoDriveToLine_Linear.java`
- **Purpose**: Autonomous driving until a line is detected
- **Features**:
  - Uses color sensor to detect a line
  - Drives until line is found
  - Demonstrates sensor-based stopping
- **Use Case**: Line-following autonomous routines

#### `RobotAutoDriveToAprilTagTank.java`
- **Purpose**: Autonomous driving to an AprilTag using tank drive
- **Features**:
  - Uses AprilTag detection for navigation
  - Tank drive (differential drive) movement
  - Demonstrates vision-based autonomous navigation
- **Use Case**: AprilTag-based autonomous positioning (tank drive)

#### `RobotAutoDriveToAprilTagOmni.java`
- **Purpose**: Autonomous driving to an AprilTag using omni drive
- **Features**:
  - Uses AprilTag detection for navigation
  - Omni-directional (mecanum) drive movement
  - Demonstrates vision-based autonomous navigation
- **Use Case**: AprilTag-based autonomous positioning (mecanum drive)

---

### Robot Teleop Drive Samples

These samples demonstrate different teleop driving modes.

#### `RobotTeleopTank_Iterative.java`
- **Purpose**: Tank drive teleop using Iterative OpMode style
- **Features**:
  - Tank drive control (one stick per side)
  - Iterative OpMode structure
  - Basic teleop driving
- **Use Case**: Tank drive teleop implementation

#### `RobotTeleopPOV_Linear.java`
- **Purpose**: POV (Point of View) drive teleop using Linear OpMode
- **Features**:
  - POV driving (left stick forward/back, right stick turn)
  - Linear OpMode structure
  - Easier to drive straight
- **Use Case**: POV drive teleop implementation

#### `RobotTeleopMecanumFieldRelativeDrive.java`
- **Purpose**: Field-relative driving for mecanum wheels
- **Features**:
  - Mecanum wheel control
  - Field-relative movement (moves relative to field, not robot)
  - Uses gyro for orientation
  - More advanced driving mode
- **Use Case**: Advanced mecanum drive with field-relative control

---

### Sensor Samples

These samples demonstrate how to use various sensors. Each shows minimal code to read and display sensor values.

#### `SensorColor.java`
- **Purpose**: Reading color sensor data
- **Features**:
  - Reads RGB and HSV color values
  - Shows gain adjustment
  - Demonstrates light control
  - Includes distance measurement (if supported)
- **Use Case**: Color detection and measurement

#### `SensorDigitalTouch.java`
- **Purpose**: Reading digital touch sensor
- **Features**:
  - Detects touch/no-touch state
  - Simple boolean input
- **Use Case**: Touch detection

#### `SensorTouch.java`
- **Purpose**: Reading analog touch sensor
- **Features**:
  - Reads analog touch sensor values
  - Provides pressure/force information
- **Use Case**: Pressure-sensitive touch detection

#### `SensorBNO055IMU.java`
- **Purpose**: Using BNO055 IMU (Inertial Measurement Unit)
- **Features**:
  - Reads orientation (Euler angles)
  - Reads acceleration and angular velocity
  - Demonstrates IMU initialization and calibration
- **Use Case**: Robot orientation and motion sensing

#### `SensorBNO055IMUCalibration.java`
- **Purpose**: Calibrating the BNO055 IMU
- **Features**:
  - Shows calibration procedure
  - Demonstrates calibration status checking
  - Improves IMU accuracy
- **Use Case**: IMU calibration process

#### `SensorIMUOrthogonal.java`
- **Purpose**: Using IMU with orthogonal coordinate system
- **Features**:
  - Reads IMU data in orthogonal (standard XYZ) coordinates
  - Demonstrates coordinate system usage
- **Use Case**: Standard IMU orientation reading

#### `SensorIMUNonOrthogonal.java`
- **Purpose**: Using IMU with non-orthogonal coordinate system
- **Features**:
  - Reads IMU data in non-orthogonal coordinates
  - Alternative coordinate system representation
- **Use Case**: Alternative IMU orientation reading

#### `SensorAndyMarkIMUOrthogonal.java`
- **Purpose**: Using AndyMark IMU with orthogonal coordinates
- **Features**:
  - Specific to AndyMark IMU hardware
  - Orthogonal coordinate system
- **Use Case**: AndyMark IMU usage

#### `SensorAndyMarkIMUNonOrthogonal.java`
- **Purpose**: Using AndyMark IMU with non-orthogonal coordinates
- **Features**:
  - Specific to AndyMark IMU hardware
  - Non-orthogonal coordinate system
- **Use Case**: AndyMark IMU alternative usage

#### `SensorAndyMarkTOF.java`
- **Purpose**: Using AndyMark Time-of-Flight distance sensor
- **Features**:
  - Reads distance measurements
  - Demonstrates TOF sensor usage
- **Use Case**: Distance measurement with TOF sensor

#### `SensorMRColor.java`
- **Purpose**: Using Modern Robotics color sensor
- **Features**:
  - Reads color values from MR sensor
  - Specific to Modern Robotics hardware
- **Use Case**: MR color sensor usage

#### `SensorMRGyro.java`
- **Purpose**: Using Modern Robotics gyro sensor
- **Features**:
  - Reads heading/rotation from MR gyro
  - Specific to Modern Robotics hardware
- **Use Case**: MR gyro sensor usage

#### `SensorMROpticalDistance.java`
- **Purpose**: Using Modern Robotics optical distance sensor
- **Features**:
  - Reads distance using optical method
  - Specific to Modern Robotics hardware
- **Use Case**: MR optical distance measurement

#### `SensorMRRangeSensor.java`
- **Purpose**: Using Modern Robotics range sensor
- **Features**:
  - Reads distance measurements
  - Specific to Modern Robotics hardware
- **Use Case**: MR range sensor usage

#### `SensorREV2mDistance.java`
- **Purpose**: Using REV 2-meter distance sensor
- **Features**:
  - Reads distance up to 2 meters
  - Specific to REV hardware
- **Use Case**: REV distance sensor usage

#### `SensorKLNavxMicro.java`
- **Purpose**: Using Kauai Labs NavX Micro IMU
- **Features**:
  - Reads orientation and motion data
  - Specific to NavX Micro hardware
- **Use Case**: NavX Micro IMU usage

#### `SensorGoBildaPinpoint.java`
- **Purpose**: Using GoBilda Pinpoint odometry sensor
- **Features**:
  - Reads robot position and heading
  - Demonstrates odometry-based localization
- **Use Case**: Odometry-based robot localization

#### `SensorOctoQuad.java`
- **Purpose**: Using OctoQuad multi-encoder interface
- **Features**:
  - Reads multiple encoders simultaneously
  - Demonstrates bulk encoder reading
- **Use Case**: Multi-encoder reading

#### `SensorOctoQuadAdv.java`
- **Purpose**: Advanced OctoQuad usage
- **Features**:
  - Advanced features of OctoQuad
  - More complex configuration
- **Use Case**: Advanced OctoQuad functionality

#### `SensorOctoQuadLocalization.java`
- **Purpose**: Using OctoQuad for robot localization
- **Features**:
  - Demonstrates localization using OctoQuad
  - Shows position tracking
- **Use Case**: OctoQuad-based localization

#### `SensorSparkFunOTOS.java`
- **Purpose**: Using SparkFun Optical Tracking Odometry System
- **Features**:
  - Reads robot position and heading
  - Demonstrates OTOS sensor usage
- **Use Case**: SparkFun OTOS localization

#### `SensorHuskyLens.java`
- **Purpose**: Using HuskyLens AI vision sensor
- **Features**:
  - Demonstrates HuskyLens object detection
  - Shows AI-based vision capabilities
- **Use Case**: AI-based object detection

#### `SensorLimelight3A.java`
- **Purpose**: Using Limelight 3A vision system
- **Features**:
  - Demonstrates Limelight vision processing
  - Shows target detection and tracking
- **Use Case**: Limelight vision system usage

---

### Utility Samples

#### `UtilityCameraFrameCapture.java`
- **Purpose**: Capturing individual frames from camera
- **Features**:
  - Shows how to grab and save camera frames
  - Useful for debugging vision
  - Demonstrates frame capture API
- **Use Case**: Camera frame debugging and analysis

#### `UtilityOctoQuadConfigMenu.java`
- **Purpose**: Configuration menu for OctoQuad sensor
- **Features**:
  - Interactive menu for OctoQuad setup
  - Demonstrates configuration UI
  - Useful for sensor calibration
- **Use Case**: OctoQuad configuration and setup

#### `SampleRevBlinkinLedDriver.java`
- **Purpose**: Controlling REV Blinkin LED driver
- **Features**:
  - Shows how to control Blinkin LED patterns
  - Demonstrates color and pattern selection
  - Useful for robot status indication
- **Use Case**: REV Blinkin LED control

---

### External Hardware Subdirectory

#### `externalhardware/RobotHardware.java`
- **Purpose**: Reusable hardware abstraction class
- **Features**:
  - Encapsulates all robot hardware initialization
  - Provides methods for robot control (drive, arm, hand)
  - Hides internal hardware details
  - Can be used by multiple OpModes
  - Includes constants for hardware parameters
- **Key Methods**:
  - `init()`: Initialize all hardware
  - `driveRobot(drive, turn)`: Combined drive and turn control
  - `setDrivePower(left, right)`: Set individual wheel powers
  - `setArmPower(power)`: Control arm motor
  - `setHandPositions(offset)`: Control servo hands
- **Use Case**: Shared hardware class for team OpModes

#### `externalhardware/ConceptExternalHardwareClass.java`
- **Purpose**: Example OpMode showing how to use RobotHardware class
- **Features**:
  - Demonstrates usage of external RobotHardware class
  - Shows how to instantiate and use hardware abstraction
  - Provides template for team OpModes
  - Cleaner OpMode code by delegating hardware to separate class
- **Use Case**: Template for creating OpModes with external hardware class

---

## Summary

The samples directory contains approximately 60+ files organized into several categories:

1. **Basic Templates** (3 files): Skeleton OpModes for different styles
2. **AprilTag Samples** (6 files): Vision-based localization and navigation
3. **Concept Samples** (15+ files): Programming concepts and features
4. **Autonomous Drive Samples** (6 files): Different autonomous navigation methods
5. **Teleop Drive Samples** (3 files): Different teleop driving modes
6. **Sensor Samples** (25+ files): Integration with various sensors
7. **Utility Samples** (3 files): Helper tools and utilities
8. **External Hardware** (2 files): Reusable hardware abstraction pattern

All files are marked with `@Disabled` annotation and should be copied to the `/TeamCode` folder before modification and use. They serve as educational resources and starting templates for FTC team development.

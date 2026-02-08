# TeamCode Development Plan & Subsystem Documentation

This document outlines the development plan, subsystem responsibilities, and team assignments for the FTC robot code.

##  Team Assignments & File Structure

The following files are targeted for completion/update by the end of the week. Updates should be pushed to GitHub every 2-3 days.

| File                  | Primary Assignee(s)                  | Key Responsibilities                                             |
| :-------------------- | :----------------------------------- | :--------------------------------------------------------------- |
| **`Drivetrain.java`** | **Kavan + Daniel**                   | Mecanum control, speed modes, field-centric drive.               |
| **`Intake.java`**     | **Jess**                             | 3 states: In, Out, Stop. Jam protection.                         |
| **`Turret.java`**     | **Moyin** (motor), **Kishi** (servo) | Turret rotation (`turret_motor`), angle control (`angle_servo`). |
| **`Shooter.java`**    | **Gergana**                          | Flywheels (`shooter_motor_0/1`), feeder (`feeder_servo`).        |
| **`Vision.java`**     | **George**                           | AprilTag detection, target offsets.                              |
| **`TeleOpMain.java`** | **Team**                             | Driver control layer, mapping inputs to subsystem methods.       |
| **`AutoMain.java`**   | **Team**                             | Autonomous sequence runner.                                      |

> **Development Rule:** If a file does not exist, create a branch with its name (e.g., `drivetrain` branch) and create the file there.

---

##  Hardware Configuration

**Crucial Reference:** [`HardwareMapConfig.java`](./HardwareMapConfig.java)

Always reference `HardwareMapConfig.java` to match hardware names exactly. This file maps all hardware to software.

### Hardware Manifest

- **Drive Motors:** `wheel_0` (NE), `wheel_1` (SE), `wheel_2` (SW), `wheel_3` (NW)
- **Turret:** `turret_motor` (DcMotorEx), `angle_servo` (Servo)
- **Intake:** `intake_motor` (DcMotorEx)
- **Shooter:** `shooter_motor_0`, `shooter_motor_1` (DcMotorEx), `feeder_servo` (Servo)
- **Sensors:** `webcam` (WebcamName), IMU forms part of the Control Hub.

---

##  Subsystem Requirements

### 1. Drivetrain (`Drivetrain.java`)

**Purpose:** Controls robot movement for TeleOp and Auto.

- **TeleOp Focus:**
  - **Control:** Mecanum drive (Strafe + Rotate), Deadzones, Scaling.
  - **Modes:** Slow/Precision toggle, Normal/Turbo mode.
  - **Stability:** Heading stabilization (gyro-based).
  - **Safety:** Zero power behavior (Brake), motor direction config.
- **Autonomous Focus:**
  - **Movement:** `driveDistance(cm)`, `turnToAngle(deg)`.
  - **Trajectory:** Path following.
  - **Correction:** PID control for heading.

### 2. Intake (`Intake.java`)

**Purpose:** Collects game elements.

- **TeleOp Focus:**
  - **Control:** Toggle states (In / Out / Stop).
  - **Protection:** Anti-jam (reverse briefly if stalled).
- **Key Logic:**
  - Button press -> `intakeIn()` -> press again -> `intakeOut()` -> press again -> `stop()`.

### 3. Turret (`Turret.java`)

**Purpose:** Controls aiming rotation.

- **TeleOp Focus:**
  - **Manual:** Stick/Button rotation.
  - **Safety:** Angle limits (Min/Max to protect wires).
  - **Presets:** Buttons for Front / Side / Scoring positions.
  - **Stability:** PID Hold position.
- **Autonomous Focus:**
  - **Auto-Aim:** Rotate to known scoring angles or align via vision.
  - **Locking:** Move only when stable.

### 4. Shooter (`Shooter.java`)

**Purpose:** Controls flywheels and firing mechanism.

- **TeleOp Focus:**
  - **Spin-up:** Toggle On/Off.
  - **Control:** PIDF for consistent RPM.
  - **Firing:** Servo flicker control (`feeder_servo`).
  - **Presets:** Low/High goal power.
- **Autonomous Focus:**
  - **Sequence:** Spin up -> Wait for RPM -> Fire N rings.
  - **Logic:** `spinUpAndShoot(n)`.

### 5. Camera Scanner / Vision (`Vision.java`)

**Purpose:** Visual processing and telemetry (No motor control).

- **TeleOp Focus:**
  - **Feedback:** Live telemetry (Target detected, Offset angle).
  - **Pipeline:** Switch between AprilTags/Color detection.
- **Autonomous Focus:**
  - **Detection:** Identification of target zones/tags.
  - **Localization:** Pose estimation via tags.
  - **Alignment:** Calculate offsets for Turret.

---

##  Architecture & Best Practices

### The "Correct" Dependency Stack

1.  **Hardware Level:** Motors, Servos, Sensors (HardwareMap).
2.  **Subsystem Level:** `Drivetrain`, `Intake`, `Turret`, `Shooter`, `Vision`.
3.  **Robot Container:** `Robot.java` (Holds instances of all subsystems).
4.  **OpMode Level:** `TeleOpMain`, `AutoMain` (Calls Robot methods).

###  TeleOp Rules

- **NEVER** do `motor.setPower(...)` inside TeleOp.
- **ALWAYS** call subsystem methods: `robot.intake.intakeIn()`.
- TeleOp is for **Driver Control Logic Only**.

###  Robot.java Structure

```java
public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Shooter shooter;
    public Vision vision;

    public Robot(HardwareMap hw) {
        drivetrain = new Drivetrain(hw);
        intake = new Intake(hw);
        turret = new Turret(hw);
        shooter = new Shooter(hw);
        vision = new Vision(hw);
    }
}
```

### Summary Table

| Subsystem      | TeleOp Focus                   | Autonomous Focus            |
| :------------- | :----------------------------- | :-------------------------- |
| **Drivetrain** | Driver control + Field-centric | Path following + Odometry   |
| **Intake**     | Button control + Anti-jam      | Pickup sequences            |
| **Turret**     | Manual aim + Presets           | Auto aim + Vision lock      |
| **Shooter**    | RPM control + Toggle           | Shoot sequences + Timing    |
| **Vision**     | Telemetry + Assist             | Detection + Decision making |

# DECODE Competition Manual vs Autonomous Code Alignment Analysis

## Overview
This document analyzes the alignment between the FTC DECODE competition manual and the DecodeAutonomous.java code, highlighting areas of compliance, partial compliance, and missing implementations.

## Competition Manual Overview
**Game:** DECODE (Presented by RTX)  
**Objective:** Two ALLIANCES of 2 teams each score purple/green ARTIFACTS in GOAL, build PATTERNS based on MOTIF, and return to BASE  
**Match Structure:** 30s AUTO + 2m TELEOP

## Code Alignment Analysis

### **FULLY ALIGNED Features**

#### 1. State Machine Structure
- **Manual Requirement:** Sequential game phases (AUTO → TELEOP)
- **Code Implementation:** 
  ```java
  enum AutonomousState {
      INIT,           // Initialize hardware and systems
      READ_TAG,       // Read AprilTag to determine target pattern
      DRIVE_TO_ROW,   // Drive to the ball collection area
      INTAKE_AND_SORT,// Intake balls while sorting them by color
      DRIVE_TO_LINE,  // Drive back to launch line
      SHOOT,          // Shoot balls in the correct sequence
      FINAL_PARK,     // Park in assigned base zone
      COMPLETE        // Autonomous routine complete
  }
  ```
- **Alignment:** Perfect match with DECODE game flow

#### 2. AprilTag Detection System
- **Manual Requirement:** OBELISK shows MOTIF (IDs 21, 22, 23) for pattern identification
- **Code Implementation:** 
  - `AprilTagVisionProcessor` detects MOTIF from OBELISK
  - Interprets AprilTag IDs to determine GPP, PGP, PPG patterns
  - Critical for AUTO period MOTIF identification
- **Alignment:** Fully compliant with AprilTag requirements

#### 3. IMU Integration
- **Manual Requirement:** Precise navigation and positioning (R105)
- **Code Implementation:** 
  - IMU hardware mapping and initialization
  - Real-time telemetry showing Yaw/Pitch/Roll
  - Rotation control using PID feedback
- **Alignment:** Supports navigation requirements

#### 4. Color Detection
- **Manual Requirement:** ARTIFACTS are purple (P) and green (G)
- **Code Implementation:** 
  - Color sensor integration for ARTIFACT identification
  - Purple/Green detection algorithm
  - Barrel sorting based on color
- **Alignment:** Matches ARTIFACT color requirements

#### 5. Servo-Based Sorting System
- **Manual Requirement:** PATTERN scoring on RAMP (2 pts per matching index)
- **Code Implementation:** 
  - Barrel controller with 3 positions (SLOT_0, SLOT_1, SLOT_2)
  - Rotation to correct positions for PATTERN building
- **Alignment:** Supports PATTERN scoring requirements

---

### **PARTIALLY ALIGNED Features**

#### 1. Field Navigation
- **Manual Requirement:** 144"×144" field with TILE coordinates (Section 9.4)
- **Code Status:** 
  - Uses placeholder coordinates (e.g., "30.0", "40.0", "60.0")
  - Needs actual TILE-based measurements
  - Missing SPIKE MARK locations
- **Gap:** Requires field-specific calibration

#### 2. ARTIFACT Control Management
- **Manual Requirement:** G408 - No more than 3 ARTIFACTS controlled simultaneously
- **Code Status:** 
  - Uses "balls" terminology instead of "ARTIFACTS"
  - Has `ballsCollected` counter but no 3-ARTIFACT limit
  - Missing CONTROL verification
- **Gap:** Need to implement 3-ARTIFACT control limit

#### 3. Scoring Verification
- **Manual Requirement:** CLASSIFIED vs OVERFLOW ARTIFACTS (Section 10.5.1)
- **Code Status:** 
  - Does not verify ARTIFACTS enter GOAL through open top
  - No distinction between CLASSIFIED and OVERFLOW
  - Missing SQUARE passage verification
- **Gap:** Need scoring validation logic

---

### **MISALIGNED Features**

#### 1. LAUNCH LINE Compliance
- **Manual Requirement:** G304.A - Robot must start "over a LAUNCH LINE"
- **Code Status:** No verification of starting position compliance
- **Issue:** May not meet starting configuration requirements

#### 2. LEAVE Requirement (AUTO Period)
- **Manual Requirement:** G405.3 - Robot must move off LAUNCH LINE for 3 points
- **Code Status:** No LEAVE verification implemented
- **Issue:** Missing AUTO period scoring opportunity

#### 3. RAMP Interaction Rules
- **Manual Requirement:** G418 - Cannot contact ARTIFACTS on RAMP
- **Code Status:** Barrel controller may violate RAMP contact rules
- **Issue:** Potential rule violation during PATTERN building

#### 4. LAUNCH ZONE Compliance
- **Manual Requirement:** G416 - Only LAUNCH when inside LAUNCH ZONE
- **Code Status:** No zone verification implemented
- **Issue:** Potential rule violation during ARTIFACT launching

#### 5. PATTERN Scoring Verification
- **Manual Requirement:** PATTERN points based on ARTIFACT color matching MOTIF index
- **Code Status:** No verification that RAMP positions match MOTIF requirements
- **Issue:** May not earn PATTERN points properly

---

## Critical Missing Implementations

### 1. **Starting Position Validation**
```java
// Need to verify robot starts over LAUNCH LINE
// G304.A compliance check
```

### 2. **3-ARTIFACT Control Limit**
```java
// Need to enforce G408 - simultaneous control limit
// Track controlled ARTIFACTS and enforce 3-max limit
```

### 3. **LEAVE Detection (AUTO)**
```java
// Need to ensure robot moves off LAUNCH LINE
// Verify "no longer over any LAUNCH LINE" at AUTO end
```

### 4. **SCORING Validation**
```java
// Verify ARTIFACTS enter GOAL through open top
// Distinguish CLASSIFIED vs OVERFLOW
// Validate SQUARE passage
```

### 5. **RAMP Interaction Safeguards**
```java
// Prevent contact with ARTIFACTS on RAMP
// G418 compliance for both own and opponent RAMP
```

### 6. **LAUNCH ZONE Verification**
```java
// Verify robot is in LAUNCH ZONE before launching
// G416 compliance check
```

### 7. **PATTERN Verification**
```java
// Validate RAMP positions match MOTIF requirements
// Track PATTERN scoring compliance
```

### 8. **BASE ZONE Compliance**
```java
// Verify final parking in BASE ZONE
// G427 compliance for last 20 seconds
```

---

## Recommendations

### **Immediate Priorities:**
1. Implement 3-ARTIFACT control limit (G408)
2. Add LEAVE verification for AUTO period points
3. Implement LAUNCH LINE starting position validation
4. Add RAMP interaction safeguards (G418)

### **Secondary Priorities:**
1. Add SCORING verification (CLASSIFIED vs OVERFLOW)
2. Implement LAUNCH ZONE compliance (G416)
3. Add PATTERN scoring validation
4. Add BASE ZONE parking verification

### **Long-term Improvements:**
1. Field-specific coordinate calibration
2. Advanced PATTERN building algorithms
3. Real-time rule compliance monitoring
4. Comprehensive telemetry for all scoring elements

---

## Conclusion

The autonomous code has an excellent structural foundation that aligns well with the DECODE game flow. However, several critical game rules need to be implemented to ensure full compliance with the competition manual. The most important missing implementations are the 3-ARTIFACT control limit, LEAVE verification for AUTO points, and RAMP interaction safeguards. Addressing these gaps will ensure the robot performs legally and maximizes scoring opportunities during competition.
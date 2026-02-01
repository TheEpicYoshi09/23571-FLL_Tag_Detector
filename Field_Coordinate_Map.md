# DECODE Field Coordinate Map

## Coordinate System Overview
The DECODE field uses a standard FTC coordinate system with the origin (0,0) at the center of the 144" x 144" field.

### Standard FTC Coordinate System
- **Origin (0,0)**: Center of the field
- **X-axis**: Positive to the right (towards Blue Alliance side)
- **Y-axis**: Positive upward (towards Audience side)
- **Units**: Inches

## Field Dimensions & Key Locations

### Overall Field
- **Size**: 144" x 144" (12' x 12')
- **Tiles**: 6x6 grid of 24" x 24" foam tiles
- **Origin**: Center of field at (0, 0)

### GOAL Locations
- **Red GOAL (ID 24)**: Located at approximately (-58.37", 55.64", 29.5")
- **Blue GOAL (ID 20)**: Located at approximately (-58.37", -55.64", 29.5")

### Tile Coordinates (Based on Figure 9-5)
The field uses a tile coordinate system where each tile is 24" x 24":

```
Column: A  B  C    D  E  F
Row 1: (0,2) (1,2) (2,2) | (3,2) (4,2) (5,2)
Row 2: (0,1) (1,1) (2,1) | (3,1) (4,1) (5,1) 
Row 3: (0,0) (1,0) (2,0) | (3,0) (4,0) (5,0)
Row 4: (0,-1) (1,-1) (2,-1) | (3,-1) (4,-1) (5,-1)
Row 5: (0,-2) (1,-2) (2,-2) | (3,-2) (4,-2) (5,-2)
Row 6: (0,-3) (1,-3) (2,-3) | (3,-3) (4,-3) (5,-3)
```

### Alliance Areas
- **Red Alliance Area**: Left side of field (negative X values)
- **Blue Alliance Area**: Right side of field (positive X values)
- **Dimensions**: 96" wide × 54" deep

### Key Field Elements Coordinates

#### Launch Lines & Zones
- **Launch Lines**: White tape at base of GOAL and triangular zones
- **Launch Zone (Audience side)**: 2 tiles wide × 1 tile deep
- **Launch Zone (GOAL side)**: 6 tiles wide × 3 tiles deep

#### Base Zones
- **Red Base Zone**: (-72", 72") to (-54", 54") - 18" × 18" square
- **Blue Base Zone**: (54", 72") to (72", 54") - 18" × 18" square

#### Loading Zones
- **Red Loading Zone**: Approximately (-72", -23") to (-49", -46")
- **Blue Loading Zone**: Approximately (49", -46") to (72", -23")
- **Dimensions**: ~23" × 23" square

#### Secret Tunnel Zones
- **Red Secret Tunnel Zone**: ~46.5" long × ~6.125" wide
- **Blue Secret Tunnel Zone**: ~46.5" long × ~6.125" wide

#### Spike Marks (ARTIFACT Starting Positions)
Located at:
- **Near (Audience side)**: GPP pattern
- **Middle**: PGP pattern  
- **Far (GOAL side)**: PPG pattern

## Pedro Pathing Coordinate Conversion

For Pedro Pathing 2.0.0, the coordinate system is right-handed:
- **X increases**: Moving right
- **Y increases**: Moving up
- **Heading 0**: Facing right
- **Heading π/2**: Facing up
- **Heading π**: Facing left
- **Heading 3π/2**: Facing down

### Conversion from FTC to Pedro Coordinates
```java
// Convert FTC Pose2D to Pedro coordinates
Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, InvertedFTCCoordinates.INSTANCE);
Pose pedroCoords = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
```

## Strategic Locations for Autonomous Navigation

### Starting Positions (Over Launch Lines)
- **Red 1 Start**: (-60", 60") approximately
- **Red 2 Start**: (-60", 45") approximately  
- **Blue 1 Start**: (60", -60") approximately
- **Blue 2 Start**: (60", -45") approximately

### ARTIFACT Collection Points
- **Near Spike Mark**: (~-10", ~50") for Red, (~10", ~-50") for Blue
- **Middle Spike Mark**: (~0", ~0") center of field
- **Far Spike Mark**: (~-10", ~-50") for Red, (~10", ~50") for Blue

### Scoring Positions
- **Red GOAL Approach**: (~-45", ~60")
- **Blue GOAL Approach**: (~45", ~-60")
- **RAMP Access**: Adjacent to GOAL on each alliance side

## Implementation Tips

### For Path Planning:
1. **Use tile-based coordinates** for coarse navigation
2. **Fine-tune with precise measurements** from CAD model
3. **Account for robot dimensions** (18" starting configuration)
4. **Consider expansion limits** (horizontal: 18" max, vertical: 38" max in final 20s)

### For AprilTag Detection:
- **Red GOAL (ID 24)**: Yaw ~-54° (from community measurements)
- **Blue GOAL (ID 20)**: Yaw ~54° (from community measurements)
- **OBELISK Tags (IDs 21, 22, 23)**: Outside field perimeter

### For Autonomous Sequences:
1. **Start**: Position over Launch Line
2. **Collect**: Navigate to Spike Mark locations
3. **Score**: Approach GOAL and position for RAMP access
4. **Park**: Navigate to BASE ZONE in final seconds

## Precision Considerations
- **Tolerance**: +/- 1" for most field elements
- **CAD Model**: Use for exact measurements when available
- **Calibration**: Verify coordinates with actual field measurements
- **IMU Integration**: Use for heading corrections during navigation
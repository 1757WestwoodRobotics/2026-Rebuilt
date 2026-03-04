# Hood Commands: Ideas & Implementations

Your hood system has `bumpUp()` and `bumpDown()` for manual control. Here are other critical commands you should add:

---

## 1. **Distance-Based Hood Control** ⭐ MOST IMPORTANT

**What it does:** Automatically calculate hood angle based on distance to hub.

**Why it matters:** In FRC shooting games, you must adjust hood angle as you move around the field. This is the core of accurate shooting.

**Implementation:**

```python
def autoHoodFromDistance(hood: HoodSubsystem) -> Command:
    """
    Automatically adjust hood angle based on robot distance to hub.
    Call once, then motor continuously adjusts as robot moves.
    """
    
    def autoFunc():
        hood.setClosedLoop(True)
        
        # Get robot's current position
        robotPose = RobotState.getHubPose()
        distanceToHub = robotPose.translation().distance(kCloseHubLocation)
        
        # Convert distance → hood angle using ballistics model
        hoodAngle = distanceToHoodAngle(distanceToHub)
        hood.setHoodGoal(hoodAngle)
    
    return Commands.run(autoFunc, hood).withName("AutoHoodDistance")


def distanceToHoodAngle(distance: float) -> Rotation2d:
    """
    Lookup table: distance (meters) → hood angle (degrees)
    
    YOU NEED TO TUNE THIS! Test your shooter at different distances
    and measure what angle gives best accuracy.
    
    Example (placeholder values):
    - 1m:  50°
    - 2m:  55°
    - 3m:  60°
    - 4m:  65°
    - 5m:  70°
    - 6m:  75°
    - 7m:  80°
    """
    
    # Simple linear interpolation example
    if distance < 2.0:
        return Rotation2d.fromDegrees(50)
    elif distance < 3.0:
        return Rotation2d.fromDegrees(55)
    elif distance < 4.0:
        return Rotation2d.fromDegrees(60)
    elif distance < 5.0:
        return Rotation2d.fromDegrees(65)
    elif distance < 6.0:
        return Rotation2d.fromDegrees(70)
    else:
        return Rotation2d.fromDegrees(75)
```

**When to use:** During auto/teleop when you want to fire at hub. Start this command, then drive toward hub — hood automatically adjusts as you move.

---

## 2. **Turret-Synchronized Hood Angle**

**What it does:** Adjust hood angle based on turret's current pointing angle.

**Why it matters:** If turret angle affects shooter velocity (backspin, sidespin), hood needs to compensate.

**Implementation:**

```python
def hoodAngleFromTurret(hood: HoodSubsystem, turret: TurretSubsystem) -> Command:
    """
    Adjust hood angle based on turret angle.
    Useful if turret rotation affects shooter dynamics.
    """
    
    def turretSyncFunc():
        hood.setClosedLoop(True)
        
        turretAngle = turret.getTurretAngle()  # Current turret rotation
        
        # Different turret angles → different hood angles
        # (This depends on your shooter mechanics!)
        if turretAngle.degrees() < 90:
            hood.setHoodGoal(Rotation2d.fromDegrees(60))
        elif turretAngle.degrees() < 180:
            hood.setHoodGoal(Rotation2d.fromDegrees(65))
        else:
            hood.setHoodGoal(Rotation2d.fromDegrees(70))
    
    return Commands.run(turretSyncFunc, hood, turret).withName("HoodTurretSync")
```

---

## 3. **Combined: Distance + Turret Angle**

**What it does:** Calculate hood angle using BOTH distance AND turret orientation.

**Why it matters:** Most realistic model — accounts for distance error + turret error.

**Implementation:**

```python
def autoHoodFull(hood: HoodSubsystem, turret: TurretSubsystem) -> Command:
    """
    Full auto-aiming: distance + turret angle both affect hood angle.
    """
    
    def autoFullFunc():
        hood.setClosedLoop(True)
        
        robotPose = RobotState.getHubPose()
        distanceToHub = robotPose.translation().distance(kCloseHubLocation)
        turretAngle = turret.getTurretAngle()
        
        # Combine both factors
        baseHoodAngle = distanceToHoodAngle(distanceToHub)
        turretCompensation = turretAngleCompensation(turretAngle)
        
        finalHoodAngle = Rotation2d.fromDegrees(
            baseHoodAngle.degrees() + turretCompensation
        )
        
        # Clamp to valid range
        clamped = Rotation2d.fromDegrees(
            max(45, min(82, finalHoodAngle.degrees()))
        )
        
        hood.setHoodGoal(clamped)
    
    return Commands.run(autoFullFunc, hood, turret).withName("AutoHoodFull")


def turretAngleCompensation(turretAngle: Rotation2d) -> float:
    """Small angle adjustments based on turret position. Returns degrees to add."""
    # Example: turret pointing left → decrease hood angle slightly
    degrees = turretAngle.degrees()
    if degrees < 90:
        return -2.0  # Subtract 2°
    elif degrees > 270:
        return +2.0  # Add 2°
    else:
        return 0.0
```

---

## 4. **Smart Bump With Bounds Checking**

**What it does:** Bump up/down, but clamp within valid range and log when at limits.

**Why it matters:** Prevents operator from fighting mechanical limits.

**Implementation:**

```python
def smartBumpUp(hood: HoodSubsystem, amount: float = 0.1) -> Command:
    """Bump up with smart bounds checking."""
    
    def bumpFunc():
        currentAngle = hood.getCurrentHoodAngle()  # Need to add this method to HoodSubsystem
        newAngle = Rotation2d.fromDegrees(
            min(currentAngle.degrees() + amount, kHoodMaxAngle.degrees())
        )
        
        if hood.isAtMax():
            Logger.recordOutput("Hood/Warning", "At max angle, cannot bump higher")
            return
        
        hood.setClosedLoop(True)
        hood.setHoodGoal(newAngle)
    
    return Commands.runOnce(bumpFunc, hood).withName("SmartBumpUp")


def smartBumpDown(hood: HoodSubsystem, amount: float = 0.1) -> Command:
    """Bump down with smart bounds checking."""
    
    def bumpFunc():
        currentAngle = hood.getCurrentHoodAngle()
        newAngle = Rotation2d.fromDegrees(
            max(currentAngle.degrees() - amount, kHoodMinAngle.degrees())
        )
        
        if hood.isAtMin():
            Logger.recordOutput("Hood/Warning", "At min angle, cannot bump lower")
            return
        
        hood.setClosedLoop(True)
        hood.setHoodGoal(newAngle)
    
    return Commands.runOnce(bumpFunc, hood).withName("SmartBumpDown")
```

**Note:** You'll need to add `getCurrentHoodAngle()` method to HoodSubsystem:
```python
def getCurrentHoodAngle(self) -> Rotation2d:
    """Return the current hood position as a Rotation2d."""
    return Rotation2d(self.inputs.hoodPosition)
```

---

## 5. **Preset Angles (Common Shots)**

**What it does:** Quick buttons for specific shot distances.

**Why it matters:** Allows operators to quickly try different ranges during testing.

**Implementation:**

```python
def hoodToCloseShot(hood: HoodSubsystem) -> Command:
    """Hood angle for close range (2-3 meters)."""
    return Commands.runOnce(
        lambda: hood.setHoodGoal(Rotation2d.fromDegrees(55)),
        hood
    ).andThen(Commands.waitUntil(hood.atTarget)).withName("HoodCloseShot")


def hoodToMidShot(hood: HoodSubsystem) -> Command:
    """Hood angle for mid range (3-5 meters)."""
    return Commands.runOnce(
        lambda: hood.setHoodGoal(Rotation2d.fromDegrees(65)),
        hood
    ).andThen(Commands.waitUntil(hood.atTarget)).withName("HoodMidShot")


def hoodToFarShot(hood: HoodSubsystem) -> Command:
    """Hood angle for far range (5+ meters)."""
    return Commands.runOnce(
        lambda: hood.setHoodGoal(Rotation2d.fromDegrees(75)),
        hood
    ).andThen(Commands.waitUntil(hood.atTarget)).withName("HoodFarShot")
```

---

## 6. **Combined Shoot Sequence**

**What it does:** One button does everything — aim turret AND hood at target.

**Why it matters:** Operator presses one button during auto/teleop; both systems reach ready state together.

**Implementation:**

```python
def shootSequence(hood: HoodSubsystem, turret: TurretSubsystem) -> Command:
    """
    Full shooting sequence:
    1. Point turret at hub
    2. Adjust hood for distance
    3. Wait for both to be ready
    4. Return success command
    """
    from commands.turretcommands import trackedTurret
    
    return Commands.parallel(
        trackedTurret(turret),  # Point turret at hub
        autoHoodFromDistance(hood)  # Adjust hood for distance
    ).withTimeout(3.0).withName("ShootSequence")
```

---

## Summary Table

| Command | Purpose | Parameters | Usage |
|---------|---------|-----------|-------|
| `autoHoodFromDistance()` | Auto-aim based on distance | - | Main shooting command |
| `hoodAngleFromTurret()` | Adjust for turret angle | turret | If turret affects shooter |
| `autoHoodFull()` | Full auto-aim (distance + turret) | hood, turret | Most complete auto-aim |
| `smartBumpUp/Down()` | Manual with bounds | amount (optional) | Fine-tuning shots |
| `hoodToCloseShot/MidShot/FarShot()` | Preset angles | - | Testing different ranges |
| `shootSequence()` | Full shoot routine | hood, turret | One-button shoot |

---

## Next Steps

1. **Add `getCurrentHoodAngle()` to HoodSubsystem** — needed for smart bumps
2. **Create a lookup table** — test your shooter at different distances, measure angles
3. **Start with `autoHoodFromDistance()`** — it's the foundation for everything else
4. **Tune constants** — the distance→angle mapping is custom to your shooter physics
5. **Test in simulation first** — then validate on real hardware

---

## Key Constants You'll Need

In `constants/hood.py`, consider adding:

```python
# Shooter tuning (MEASURED VALUES - you need to get these!)
kHoodDistanceTable = {
    1.0: 50,   # 1m → 50°
    2.0: 55,   # 2m → 55°
    3.0: 60,   # 3m → 60°
    4.0: 65,   # 4m → 65°
    5.0: 70,   # 5m → 70°
    6.0: 75,   # 6m → 75°
}

kHoodFudgeAmount = Rotation2d.fromDegrees(2.5)  # You already have this!
```

Then use it:
```python
def distanceToHoodAngle(distance: float) -> Rotation2d:
    """Interpolate from distance table."""
    distances = sorted(kHoodDistanceTable.keys())
    if distance <= distances[0]:
        return Rotation2d.fromDegrees(kHoodDistanceTable[distances[0]])
    if distance >= distances[-1]:
        return Rotation2d.fromDegrees(kHoodDistanceTable[distances[-1]])
    
    # Linear interpolation between points
    for i in range(len(distances) - 1):
        if distances[i] <= distance < distances[i+1]:
            ratio = (distance - distances[i]) / (distances[i+1] - distances[i])
            angle1 = kHoodDistanceTable[distances[i]]
            angle2 = kHoodDistanceTable[distances[i+1]]
            return Rotation2d.fromDegrees(angle1 + (angle2 - angle1) * ratio)
```

---

## Which Should You Implement First?

**Priority:**
1. ⭐⭐⭐ `autoHoodFromDistance()` — foundation for accurate shooting
2. ⭐⭐ `hoodToCloseShot/MidShot/FarShot()` — easy presets for testing
3. ⭐⭐ `smartBumpUp/Down()` — better manual control
4. ⭐ `hoodAngleFromTurret()` — only if turret angle affects shooter
5. ⭐ `autoHoodFull()` — combine everything for best auto-aiming
6. ⭐ `shootSequence()` — one-button auto-aim

Start with 1 & 2, test them, then add the rest.


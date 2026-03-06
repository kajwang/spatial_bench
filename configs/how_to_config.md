# Configuration Guide
## 1. Scene Loading
Scene configuration defines the basic structure of the environment:

```yaml
scene:
  type: indoors                    # Scene type
  prim_path: /ArnoldRoom          # Scene root path
  spawn:
    usd_path: /house/arnold_0/layout.usd  # Scene file path
    orientation: [0.707, 0.707, 0.0, 0.0] # Rotation (quaternion)
    translation: [0.0, 0.0, 0.0]          # Translation (x, y, z)
    scale: [0.01, 0.01, 0.01]             # Scale ratio
```

### Scene Feature Checklist:
- ✅ Basic scene loading and USD file support
- ✅ Translation, orientation and scaling support
- ✅ Ground collision and material customization
- ✅ Furniture collision customization
- ☐ Wall collision and material customization
- ☐ Lighting configuration support


## 2. Normal Object Configuration

```yaml
graspable_objects:
  apple:                          # Object name
    type: "rigid"                 # Object type: rigid / static
    prim_path: "/Apple"           # Path in scene
    spawn:
      usd_path: "/fruit/apple/apple_rigid.usd"  # Model file
      translation: [1.8, -4.5, 0.455]          # Initial position
      orientation: [1.0, 0.0, 0.0, 0.0]        # Initial rotation
      scale: [0.007, 0.007, 0.007]             # Model scale
    collision_enabled: true
    collision_approximation: convexHull
    mass: 0.1                     # Mass
    physics_material:             # Physics material
      static_friction: 1.5
      dynamic_friction: 1.5
```

### Objects Feature Checklist:
- ✅ Basic object loading and USD file support
- ✅ Shape and physical attribute configuartion
- ✅ Rigid object support，type can be **static** (gravity-free) or **rigid**
- ✅ Articulated object support
- 🔶 Collision type support, not working.
- ☐ Deformable object support
- ☐ Particle object support


## 3. Task Configuration

Defines specific requirements for each task:

```yaml
tasks:
  type: object_movement          # Task type
  settings:
  - rigid: ["apple_static", "coke"]
  - rigid: ["apple"]
```

#### Task System Feature Checklist:
- ✅ Multi-object loading
- ✅ Multi-setting switch support
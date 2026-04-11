# Hardware

PCB designs and mechanical files for the Plato Pod robot.

## Contents

- `pcb/` — KiCad project for the custom circular PCB (55mm diameter)
- `chassis/` — 3D-printable chassis STL/STEP files
- `bom/` — Bill of materials for production

## Robot Specifications

| Parameter       | Value                        |
|-----------------|------------------------------|
| Diameter        | 55mm                         |
| Height          | ~30mm                        |
| Drive           | Differential (2× N20 motors) |
| MCU             | ESP32-C3-MINI-1              |
| Motor driver    | DRV8833                      |
| Battery         | 3.7V 400–500mAh LiPo        |
| Charging        | USB-C via TP4056             |
| Tracking marker | AprilTag 36h11 (50×50mm)     |

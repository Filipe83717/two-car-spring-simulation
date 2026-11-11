# Two-car spring-damper simulation

This MATLAB project models a car and trailer connected by a linear spring and viscous coupling damper. The script simulates the longitudinal motion of both vehicles when the car is subjected to a constant driving force, starting from rest with the spring initially unstretched.

The current implementation also includes a comparison between the nominal spring stiffness and a reduced-stiffness case, so the effect of a stiffness disturbance on the spring extension can be inspected directly.

## Run the simulation

Open the folder in MATLAB and run:

```matlab
two_cars
```

The script uses `ode45` to integrate the equations of motion from `t = 0` to `t = 10 s` with 1,001 samples (`0.01 s` spacing). The solver tolerances are set to:

- Relative tolerance: `1e-7`
- Absolute tolerance: `1e-9`

## Model and input parameters

| Parameter | Value |
|---|---:|
| Car mass, `m1` | 1,500 kg |
| Trailer mass, `m2` | 1,000 kg |
| Car damping, `b1` | 50 N·s/m |
| Trailer damping, `b2` | 40 N·s/m |
| Spring stiffness, `k` | 1,000,000 N/m |
| Coupling damper, `c` | 44,068 N·s/m |
| Driving force | 1,800 N |

The state vector is ordered as:

```matlab
[y1; y2; y3; y4] = [x1; v1; x2; v2]
```

where:

- `x1`, `v1` are the car position and velocity
- `x2`, `v2` are the trailer position and velocity
- each displacement is measured from the corresponding vehicle's own starting point
- the spring extension is `x1 - x2`
- the spring force is `k * (x1 - x2)`
- the damper force is `c * (v1 - v2)`

## What the script prints and shows

The simulation prints the car and trailer positions at `t = 5.00 s` to the MATLAB command window. The output values are:

| Quantity | Value |
|---|---:|
| Car displacement, `x1(5)` | 8.48374929 m |
| Trailer displacement, `x2(5)` | 8.48301622 m |

The script also produces a stiffness comparison plot showing the nominal spring extension versus a case with 20% lower stiffness. The main figure window contains four subplots:

1. Applied driving force
2. Car and trailer displacements
3. Car and trailer velocities
4. Spring force

## Outputs

The script writes the generated results to a `results` folder next to the MATLAB file. The files are:

| File | Contents |
|---|---|
| `two_cars_graphs.pdf` | Main simulation plots |
| `stiffness_comparison.pdf` | Nominal versus reduced-stiffness extension comparison |
| `two_cars_values.csv` | Time, positions, velocities, spring extension, and spring force |
| `two_cars_values.pdf` | PDF table of the sampled results |

An animation of the two vehicles and the connecting spring is also generated in the MATLAB figure window.

## Notes

This project is a numerical simulation rather than a closed-form analytical derivation. It is intended to visualize the transient response of the coupled system and compare the nominal case against a reduced-stiffness disturbance case.

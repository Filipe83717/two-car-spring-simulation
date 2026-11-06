# ELI2300 — Longitudinal dynamics of a car and trailer

This project compares an analytical Laplace solution with a MATLAB simulation of a car towing a trailer. A linear spring and a viscous damper connect the two vehicles. The comparison uses one driving scenario: both vehicles start from rest with the spring at its natural length, and a constant driving force of 1,800 N acts on the car from time zero.

## Run the simulation

Save the complete script as `two_cars.m`, including the local `writeValuesPDF` function at the end. Open its folder in MATLAB and run:

```matlab
two_cars
```

The script integrates the equations with `ode45` from 0 to 10 s and returns 1,001 samples spaced 0.01 s apart. The relative and absolute solver tolerances are set to `1e-7` and `1e-9`, respectively.

## Model parameters

| Parameter | Value |
|---|---:|
| Car mass | 1,500 kg |
| Trailer mass | 1,000 kg |
| Car resistance coefficient | 50 N·s/m |
| Trailer resistance coefficient | 40 N·s/m |
| Spring stiffness | 1,000,000 N/m |
| Coupling damper coefficient | 44,068 N·s/m |

The state order is `[x1; v1; x2; v2]`, with the car first and the trailer second. Each displacement is measured from the vehicle’s own starting position. The spring extension is `x1 - x2`. The spring force is `k*(x1 - x2)`, and the damper contributes `c*(v1 - v2)`. The selected parameters give a damping ratio of approximately 0.900 for the relative oscillation between the car and trailer.

## Verification at 5 seconds

The MATLAB Command Window prints the simulated displacements at 5.00 s. Compare each printed value with the corresponding analytical reference below. Calculate the absolute difference before rounding the values for the report.

| Displacement | Analytical reference |
|---|---:|
| Car, x1(5) | 8.48374929 m |
| Trailer, x2(5) | 8.48301622 m |

## Outputs

The figure shows displacement, velocity, and spring force. The third plot displays only the spring contribution. The animation follows the vehicles as they move along the track. The script saves the following files in a `results` folder beside the script:

| File | Contents |
|---|---|
| `two_cars_graphs.pdf` | The three simulation plots |
| `two_cars_values.csv` | Time, both displacements and velocities, spring extension, and spring force |
| `two_cars_values.pdf` | Paginated table of the sampled values |

## AI disclosure

The MATLAB simulation code was generated using ChatGPT (GPT-6 Astra, medium reasoning setting).

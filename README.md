# Online Tire Cornering Stiffness for Articulated Road Vehicles
This repository contains the supporting project files related to a master's thesis at Chalmers, spring 2026. The work was carried out using the simulation software IPG TruckMaker and MATLAB Simulink.

## Repository Structure

Each folder contains its own `README.md` that gives additional information. In short,
* `Muse:` Implementation of the [MUSE](https://github.com/DarthDazzle/MUSE) estimator class.
* `Scripts:` Misc. programs written for different parts of the project.
* `Trailer:` The Simulink model for the full articulated road vehicle.
* `Truck:` The Simulink model for just the truck.

The `.zip` files at the repo top level contain _IPG TruckMaker_ projects:
* `Truck.zip:` Simulink implementation, scenario, truck, tires, etc., for the truck-only estimator.
* `TruckUpdateStrategy.zip:` Same as the previous one, but with the Gramian-based update strategy.
* `Trailer.zip:` Project files for the full coupled truck and trailer system.

## Thesis Information

The work was carried out at _Mechanics and Maritime Sciences_ (MMS) at Chalmers University of Technology under the supervision of Axel Ceder and Mats Jonasson. The report detailing this work is published by Chalmers at the [Chalmers Open Digital Repository](https://odr.chalmers.se/home).

### Authors:

**Syed Ahsan Ali Shah:**  
B.Sc. in Electrical Engineering, M.Sc. in Systems, Control & Mechatronics

**Lucas Fosse:**  
B.Sc. in Electrical Engineering, M.Sc. in Complex Adaptive Systems

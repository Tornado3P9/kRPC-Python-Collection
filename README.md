# kRPC-Python-Collection

[https://krpc.github.io/krpc/index.html](https://krpc.github.io/krpc/index.html)

This repository contains simple python scripts for launching and controlling a rocket in the [KSP](https://store.steampowered.com/app/220200/Kerbal_Space_Program/) space simulator.

The scripts completely ignore the existence of SolidFuel Boosters but that's intentionally in order to account for different rocket types. Just tab the space key once your SRBs are empty or modify the script yourself.
The launch script solely checks for the existence of active thrust, without considering the source of that thrust. If there is no thrust being generated, the script will automatically initiate staging regardless.

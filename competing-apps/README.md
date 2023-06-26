# app-deconfliction/competing-apps

## Purpose

The competing-apps directory of the app-deconflicion repository contains both competing applications and the deconfliction pipeline framework that supports deconfliction methods. The competing apps are used to drive development and testing of methods, i.e., creating the conflict that must be resolved or deconflicted. The pipeline framework is where the deconfliction methods are plugged in providing all the functionality that is common regardless of the method--heuristics/rules, optimization, or cooperation/collaboration. This allows the deconfliction method code to be as lean as possible with the only responsibility being to perform device set-point deconfliction given benefit of a well-defined interface with the framework.

## Overview

The deconfliction pipeline framework follows the design described in the project foundational paper published in IEEE Access and available at <https://ieeexplore.ieee.org/document/10107708>, specifically sections III-B and -C. There are methods in deconfliction-pipeline.py code directly corresponding to subsections in the foundational paper, e.g., SetpointProcessor, ConflictIdentification, DeconflictionSolution, and DeviceDispatcher.

The current full workflow for deconfliction consists of three components:
<ol>
<li>sim-sim or (sim)ulated (sim)ulator</li>
<li>competing apps, e.g., resilience app and decarbonization app</li>
<li>deconfliction pipeline</li>
</ol>

Sim-sim publishes messages containing time-series simulation parameters and updated device set-points and battery SoC. Competing apps subscribe to sim-sim messages to carry out their work determining and publishing new device set-point requests. The deconfliction pipeline subscribes to competing app set-point messages to perform the steps described in the foundational paper producing deconflicted set-points dispatched to devices. Sim-sim is subscribed to deconfliction pipeline device dispatch messages in order to update simulation state completing the workflow loop.

## DeconflictionMethod class interface

Deconfliction methods are implemented within classes named DeconflictionMethod--the filename for defining the class may distinguish the type of method, but the class name must be DeconflictionMethod. The only required methods for DeconflictionMethod are a constructor for initialization and a method named deconflict() that performs deconfliction when called. The class may define other methods or even call other classes or processes needed to perform deconfliction at the descretion of the method developer, but the pipeline framework uses only \_\_init\_\_() and deconflict(). The pipeline framework imports and directly invokes these methods so DeconflictionMethod is part of the deconfliction pipeline process as distinguised from competing apps and sim-sim that interface via inter-process messaging.

The interface of the pipeline framework and DeconflictionMethod classes consists of:
<ul>
<li>Signature of DeconflictionMethod \_\_init\_\_() constructor</li>
<li>Signature of and return value from DeconflictionMethod deconflict() method</li>
<li>Descriptions of ConflictMatrix and ResolutionVector structures for exchanging data between the pipeline framework and DeconflictionMethod class</li>
<li>MethodUtil class members used to provide ancilliary data from the pipeline framework to the DeconflictionMethod class</li>
</ul>

## Directory layout

```` bash
.
├── README.md
├── run-deconfliction.sh
├── sim-starter
    ├── run-sim.sh
    ├── sim-sim.py
    ├── sim-starter.py
    ├── time-series.csv
    ├── 123-config.json
    └── ...
├── optimization-apps
    ├── optimization-app.py
    ├── run-resilience.sh
    ├── run-decarbonization.sh
    └── run-profit.sh
├── workflow-apps
    ├── resilience-app.py
    ├── run-resilience.sh
    ├── decarbnonization-app.py
    └── run-decarbonization.sh
├── deconfliction-pipeline
    ├── deconfliction-pipeline.py
    └── run-pipeline.sh
├── deconfliction-methods
    ├── compromise-rd-partial-method.py
    ├── compromise-rd-full-method.py
    ├── compromise-rdp-partial-method.py
    ├── compromise-rdp-full-method.py
    ├── exclusivity-r-full-method.py
    └── ...
└── shared
    ├── AppUtil.py
    ├── MethodUtil.py
    └── sparql.py
````

Note "..." indicates files similar to the one preceeding and there are additional files in the repo that are superflous to the existing deconfliction workflow.

## Prerequisites

## Running deconfliction pipeline


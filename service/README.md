# app-deconfliction/service

Author: Gary Black <br>
Last updated: September 26, 2024

## Purpose

The service directory of the app-deconflicion repository contains the entirety of the FY24 Centralized Deconfliction Service that includes both the deconfliction service (aka, deconfliction pipeline) and sample competing applications for development, testing, and demonstration. The service/pipeline performs deconfliction using a combined methodology applied in stages that were initially developed independently through work in FY23. These stages are:
<ol>
<li>Rules & Heuristics
<li>Cooperation
<li>Optimization
</ol>

While the FY23 prototype used hardwired file-based simulation data to drive the sample competing apps, the FY24 service uses GridLAB-D simulations and meets the requirements for a full service within the GridAPPS-D platform. Apps using both the PuLP and CVXPY optimization libraries with objectives for resilience, decarbonization, and profit via conservation voltage reduction (CVR) are supported for the FY24 service.

## Overview

The Centralized Deconfliction Service builds on the FY23 prototype following the design described in the project foundational paper published in IEEE Access and is available at <https://ieeexplore.ieee.org/document/10107708>, specifically sections III-B and -C. There are methods in deconfliction-pipeline.py code directly corresponding to subsections in the foundational paper, e.g., SetpointProcessor, ConflictIdentification, and DeviceDispatcher. The service extends what was done in the prototype by applying the combined or staged deconfliction methodology first described in the end of FY23 Deconfliction Alternatives Analysis paper as well as integrating with GridLAB-D simulations.

The deconfliction workflow kicks off with GridLAB-D simulation measurement messages that provide updated device setpoints and battery SoC data. Competing apps subscribe to the GridLAB-D messages to carry out their work determining and publishing new device setpoint requests via CIM DifferenceBuilder messages. The deconfliction service intercepts these CIM DifferenceBuilder messages from competings apps to perform the steps described in the Foundational and Alternatives Analysis papers producing deconflicted setpoints dispatched to devices also through CIM DifferenceBuilder messages. The service exchanges messages with competing apps during an iterative stage of deconfliction that incentivizes apps to cooperate. Subsequent GridLAB-D simulation measurement messages reflect these deconflicted setpoints provided by the service and are processed by competing apps thus completing the deconfliction workflow loop.

For details on the combined/staged deconfliction methodology implemented in the FY24 Centralized Deconfliction Service, please see the Functional Specification document for the service at <https://github.com/GRIDAPPSD/gridappsd-training/blob/main/module-content/docs/source/services/app-deconfliction/FY24ServiceFunctionalSpecsFinal.md>

## ConflictMatrix and ResolutionVector structures

Two data structures, ConflictMatrix and ResolutionVector, are central to the deconfliction service design and thus important to a full understanding of how the service functions.

ConflictMatrix is a multi-dimensioned dictionary. The first dimension or key is all the devices, represented by CIM mRID values, for which there are ConflictMatrix entries. Therefore, "for device in ConflictMatrix:" will iterate over all devices. The second dimension is all the apps for which there are ConflictMatrix[device] entries. Therefore, "for app in ConflictMatrix[device]:" would iterate over all apps for a given device and when this for loop is nested under the previous one for devices, it would iterate over all entries of ConflictMatrix.

The value for each ConflictMatrix entry is a tuple containing the timestamp and setpoint value for that device and app combination. I.e., "ConflictMatrix[device][app] = (timestamp, setpoint)" represents both the keys for the ConflictMatrix dictionary and timestamp/setpoint value for that device and app combination. Note that the timestamp will be the same for all devices for a given app as a new setpoint message for an app replaces all previous setpoint requests for that app. I.e., if a device setpoint from a previous request is not present in a new request, that device and app combination is cleared from the ConflictMatrix so only requests from the most recent request remain.

Note that the name ConflictMatrix could be considered a misnomer for what is stored in the dictionary. ConflictMatrix represents the most recent setpoints (and associated timestamps) for an app over all devices for which that app has a setpoint. There may or may not be conflicts (different setpoint values) in ConflictMatrix for any individual device between apps. As the foundational paper established the name for ConflictMatrix, this will be preserved.

To summarize, the following code snippet will iterate over and print all ConflictMatrix entries:

```` bash
for device in ConflictMatrix:
  for app in ConflictMatrix[device]:
    print('ConflictMatrix setpoint for device: ' + device + ', app: ' + app +
          ', value: ' + str(ConflictMatrix[device][app][1]) +
          ', timestamp: ' + ConflictMatrix[device][app][0]))
````

ResolutionVector is single dimension dictionary where the key is the device. The purpose of ResolutionVector is to specify a single deconflicted or resolved setpoint for each device across all apps and therefore there is no app dimension or key. To iterate over all setpoint values in the ResolutionVector, the code would be "for device in ResolutionVector:".

Like ConflictMatrix, the value for each ResolutionVector entry is a tuple containing the timestamp and setpoint value for that device. I.e., "ResolutionVector[device] = (timestamp, setpoint)" represents both the key for the ResolutionVector dictionary and timestamp/setpoint value for that device. The timestamp tuple element represents the most recent timestamp used to determine a deconflicted setpoint over all apps for the device and can therefore be used to determine how "fresh" a resolved set-point is.

To summarize, the following code snippet will iterate over and print all ResolutionVector entries:

```` bash
for device in ResolutionVector:
  print('ResolutionVector setpoint for device: ' + device +
        ', value: ' + str(ResolutionVector[device][1] +
        ', timestamp: ' + ResolutionVector[device][0]))
````

Note on timestamps and deconfliction: The ConflictMatrix maintains the timestamps associated with competing app setpoint requests. This could indicate that some requests are "fresh" while others are quite "stale". If an app sends a single setpoint request very early on in a simulation then either crashes, exits, or takes a very long nap, this original request remains in ConflictMatrix for the duration of the run (simulation). Currently the timestamp values in ConflictMatrix are ignored by the combined/staged deconfliction methodology. It is possible that future enhancements will utilize the timestamp such as the service itself discarding "stale" setpoints that have passed an expiration criteria or they will be giving less weight in performing deconfliction than setpoints with more recent timestamps.

On a related note, in addition to timestamps within ConflictMatrix that are old, it's also possible based on the deconfliction service design for there to be newly arriving setpoint requests from apps that are associated with timestamps older than the timestamp for the most recent GridLAB-D measurement message. An app could be very slow to produce a setpoint request causing this situation. Similarly, the deconfliction process may be very slow to produce a ResolutionVector and thus the timestamp associated wih the device dispatch message could also be older than expected. As the implications of discarding them are too significant, in both these cases the deconfliction service considers these messages as valid even though they are not based on the most current GridLAB-D measurements.

## Directory layout

```` bash
.
├── README.md
├── run-deconfliction.sh
├── sim-starter
    ├── sim-starter.py
    ├── ieee123apps.xml
    ├── 123apps-config.json
    └── ...
├── optimization-apps
    ├── optimization-app-pulp.py
    ├── optimization-app-cvxpy.py
    ├── run-resilience.sh
    ├── run-decarbonization.sh
    └── run-profit.sh
├── deconfliction-pipeline
    ├── deconfliction-pipeline.py
    └── run-pipeline.sh
└── shared
    ├── AppUtil.py
    ├── MethodUtil.py
    └── sparql.py
````

Note "..." indicates files similar to the one preceding and there are additional files in the repo that are omitted in the layout above and not important to the understanding of the deconfliction service.

## Prerequisites

<ol>
<li>
You must have the dockerized GridAPPS-D platform running which is available at https://github.com/GRIDAPPSD/gridappsd-docker. Follow the documentation there if you are unfamiliar with running the platform.
</li>

<li>
An updated version of the IEEE 123-bus model defining batteries and solar PVs not yet included in the standard GridAPPS-D platform distribution must be loaded after starting the platform. The CIM model for this updated test feeder is exported to the sim-starter directory. Open the Blazegraph URL in the web browser and upload the file (ieee123apps.xml) using the "UPDATE" tab (http://localhost:8889/bigdata/#update).

Note that as long as docker containers are not cleared with the "./stop.sh -c" command, it is possible to stop and start the platform repeatedly without reloading this updated 123-bus model.
</li>

<li>
Python version 3.8 or newer is required as the one in your $PATH and can be checked with the command "python --version".
</li>

<li>
The gridappsd-python module must be installed in Python. To check if this module is already installed:

```` bash
$ python
>>> import gridappsd
````

If the import returns an error message, see <https://github.com/GRIDAPPSD/gridappsd-python> for installation instructions.
</li>

<li>
Various other Python modules are required to run the different processes that are part of the deconfliction pipeline. The recommended approach is to run one-by-one each required deconfliction process through initialization to identify missing modules. Missing modules should be installed until the process successfully initializes at which time the same initialization test can be done for the next process. The following steps walk through running each deconfliction process and cover the most likely missing modules.
</li>

<li>
To test a competing app (all apps use the same base code varying only the optimization objective function) from a shell in the service directory:

```` bash
$ cd optimization-apps
$ ./run-resilience.sh 123apps standalone
````

Note the final argument of "standalone" must be present to perform a standalone invocation as needed for this test. If you get output starting with "Initialized resilience optimization competing app" after some query output, this demonstrates successful initialization and you may do a ctrl-C exit. Modules likely to be missing include numpy, tabulate, pulp, and cvxpy. The following may prove helpful based on failed imports:

```` bash
$ sudo pip install numpy
$ sudo pip install tabulate
$ sudo apt-get install glpk-utils
$ sudo pip install pulp
$ sudo pip install cvxpy
````

Note that glpk-utils is needed by the PuLP optmization module.
</li>

<li>
To test the core deconfliction-pipeline process assuming you were in the optimization-apps directory:

```` bash
$ cd ../deconfliction-pipeline
$ ./run-pipeline.sh 123apps standalone
````

Note the final argument of "standalone" must be present to perform a standalone invocation as needed for this test. If you get output starting with "Initialization--finished" after some query output, this demonstrates successful intialization and you may do a ctrl-C exit.
</li>
</ol>

## Running deconfliction service

The deconfliction service can be run either as individual processes running in separate terminals or from a single wrapper shell script encompassing all processes. Running from the wrapper script will suffice in most all instances including if there is a need to scrutinize running diagnostic terminal output from all processes. Therefore no further description will be provided on running the individual processes other than to note that the run-deconfliction.sh wrapper script logic can be studied to learn what is going on under the covers.

The run-deconfliction.sh wrapper script in the service directory is your one-stop shop for running deconfliction. Comments at the top of the script provide guidance on command line arguments, with the basic usage being:

```` bash
$ ./run-deconfliction.sh <MODEL> <APPS> [--optlib <OPTLIB>] [--interval <INTERVAL>] [--weights <WEIGHTS>]
````

where \<MODEL\> is a shorthand used for looking up the full GridAPPS-D simulation request and feeder mrid. Currently, the only \<MODEL\> value supported for app deconfliction development is "123apps", which uses the updated IEEE 123-bus model that includes batteries, assuming that has been loaded into the GridAPPS-D platform per the guidance above.

\<APPS\> is a shorthand code composed of the first letters for each of the competing apps to run. The possible apps are resilience, code "r" or "R"; decarbonization, code "d" or "D", and profit_cvr, code "p" or "P". Thus, "rdp" would run all three apps and "rd" would run resilience and decarbonization without profit_cvr.

\<OPTLIB\> is the optional name of the optimization library to use for competing apps. If the value is either "cvxpy" or "CVXPY", then the CVXPY library will be used. Otherwise, the PuLP library will be used.

\<INTERVAL\> is the optional integer value in seconds at which competing apps will perform optimizations and send setpoints requests via CIM DifferenceBuilder messages. The value, if specified, must be a multiple of 3 for compatibility with GridLAB-D simulations. If not specified, the competing apps will use an appropriate default value such as 15 seconds, meaning an optimization will be performed every fifth simulation measurements message from GridLAB-D. A value of 3 corresponds to an optimization for every GridLAB-D measurements message. In cases other than stress testing the deconfliction service it is recommended a multiple of 3 in the range of 9-18 be used. Better yet, omit this optional argument unless there is an important reason for specifying it.

\<WEIGHTS\> allows application and/or device weighting factors to be read from files and applied during the optimization stage of deconfliction. However, the use of file-based weights currently interferes with the weights applied automatically during the cooperation stage of deconfliction as weights are critical to incentivizing apps to participate in cooperation. Therefore, this argument should not be specified and is only included as a possible future enhancement for combining cooperation incentive weights with file-based weights.

With all of that as background, as example invocations of run-deconfliction.sh, consider the following:

```` bash
$ ./run-deconfliction.sh 123apps rd
$ ./run-deconfliction.sh 123apps rdp
$ ./run-deconfliction.sh 123apps rdd cvxpy
````

In the first invocation, the resilience and decarbonization competing apps are run with a GridLAB-D simulation for the batteries-included IEEE 123 node model. In the second invocation, the profit CVR app is add in as well. In the third invocation, the CVXPY optimization library is used for the competing apps instead of the default PuLP library.

The run-deconfliction.sh wrapper script normally only shows diagnostic log output for the deconfliction pipeline process in the terminal where the wrapper script is invoked. However, each of the processes produces a log file that can either be viewed during the run (typically via "tail -f") or afterwards. These files are written to a log subdirectory--optimization-apps/log for the competing apps and deconfliction-pipeline/log for the pipeline process. If you are interested in the briefest of workflow progress output such as for a simple demonstration a "grep" for the ">>>" pattern will do the job. For example, to tail this workflow overview during a running simulation, change directory to deconfliction-pipeline/log and issue the command 'tail -f deconfliction-pipeline.log | grep ">>>"'.

Although the run-deconfliction.sh wrapper script starts a number of processes, some of them as background jobs, there is special logic that "traps" ctrl-C exits from the script and properly terminates all jobs associated with the deconfliction service such as competing apps. Note that in the case of a ctrl-C exit from the wrapper script that a GridLAB-D simulation that has been started will not be terminated and instead run to completion.


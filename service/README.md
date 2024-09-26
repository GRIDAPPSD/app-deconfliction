# app-deconfliction/service

Author: Gary Black <br>
Last updated: September 26, 2024

## Purpose

The service directory of the app-deconfliction repository contains the entirety of the FY24 Centralized Deconfliction Service that includes both the deconfliction service (aka, deconfliction pipeline) and sample competing applications for development, testing, and demonstration. The service/pipeline performs deconfliction using a combined methodology applied in stages that were initially developed independently through work in FY23. These stages are:
<ol>
<li>Rules & Heuristics
<li>Cooperation
<li>Optimization
</ol>

While the FY23 prototype used hardwired file-based simulation data to drive the sample competing apps, the FY24 service uses GridLAB-D simulations and meets the requirements for a full service within the GridAPPS-D platform. Apps using both the PuLP and CVXPY optimization libraries with objectives for resilience, decarbonization, and profit via conservation voltage reduction (CVR) are supported for the FY24 service.

## Overview

The Centralized Deconfliction Service builds on the FY23 prototype following the design described in the project foundational paper published in IEEE Access and available at <https://ieeexplore.ieee.org/document/10107708>, specifically sections III-B and -C. There are methods in deconfliction-pipeline.py code directly corresponding to subsections in the foundational paper, e.g., SetpointProcessor, ConflictIdentification, and DeviceDispatcher. The service extends what was done in the prototype by applying the combined or staged deconfliction methodology first described in the end of FY23 Deconfliction Alternatives Analysis paper as well as integrating with GridLAB-D simulations.

The deconfliction workflow kicks off with GridLAB-D simulation measurement messages that provide updated device setpoints and battery SoC data. Competing apps subscribe to the GridLAB-D measurements to carry out their work determining and publishing new device setpoint requests via CIM DifferenceBuilder messages. The deconfliction service intercepts these DifferenceBuilder messages from competings apps to perform the steps described in the Foundational and Alternatives Analysis papers producing deconflicted setpoints dispatched to devices also through CIM DifferenceBuilder messages. The service exchanges messages with competing apps during an iterative stage of deconfliction that incentivizes apps to cooperate in trying to reach consensus setpoint values. Subsequent GridLAB-D simulation measurement messages reflect these deconflicted setpoints requested by the service and are processed by competing apps, thus completing the deconfliction workflow loop.

For details on the combined/staged deconfliction methodology implemented in the FY24 Centralized Deconfliction Service, please see the Functional Specification document for the service at <https://github.com/GRIDAPPSD/gridappsd-training/blob/main/module-content/docs/source/services/app-deconfliction/FY24ServiceFunctionalSpecsFinal.md>.

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
Various other Python modules are required to run the different processes that are part of the deconfliction service. The recommended approach is to run one-by-one each required deconfliction process through initialization to identify missing modules. Missing modules should be installed until the process successfully initializes at which time the same initialization test can be done for the next process. The following steps walk through running each deconfliction process and cover the most likely missing modules.
</li>

<li>
To test a competing app (all apps use the same base code varying only the optimization objective function) from a shell in the service directory:

```` bash
$ cd optimization-apps
$ ./run-resilience.sh 123apps standalone
````

Note the final argument of "standalone" must be present to perform a standalone invocation as needed for this test. If you get output starting with "Initialized resilience" after some query output, this demonstrates successful initialization and you may do a ctrl-C exit. It is best to test both a PuLP and CVXPY optimization app since each uses some different modules. The test above is for PuLP, but CVXPY can be tested with:

```` bash
$ ./run-resilience.sh 123apps standalone cvxpy
````

There is little to be gained from trying the decarbonization or profit objectives in addition to resilience, but they also support the standalone argument. Modules likely to be missing for the competing apps include numpy, tabulate, pulp, and cvxpy. The following may prove helpful based on failed imports:

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
$ ./run-deconfliction.sh 123apps rdp cvxpy
````

In the first invocation, the resilience and decarbonization competing apps are run with a GridLAB-D simulation for the batteries-included IEEE 123 node model. In the second invocation, the profit CVR app is add in as well. In the third invocation, the CVXPY optimization library is used for the competing apps instead of the default PuLP library.

The run-deconfliction.sh wrapper script normally only shows diagnostic log output for the deconfliction pipeline process in the terminal where the wrapper script is invoked. However, each of the processes produces a log file that can either be viewed during the run (typically via "tail -f") or afterwards. These files are written to a log subdirectory--optimization-apps/log for the competing apps and deconfliction-pipeline/log for the pipeline process. If you are interested in the briefest of workflow progress output such as for a simple demonstration a "grep" for the ">>>" pattern will do the job. For example, to tail this workflow overview during a running simulation, change directory to deconfliction-pipeline/log and issue the command 'tail -f deconfliction-pipeline.log | grep ">>>"'.

Although the run-deconfliction.sh wrapper script starts a number of processes, some of them as background jobs, there is special logic that "traps" ctrl-C exits from the script and properly terminates all jobs associated with the deconfliction service such as competing apps. Note that in the case of a ctrl-C exit from the wrapper script that a GridLAB-D simulation that has been started will not be terminated and instead run to completion.

## Post FY24 TO-DO

<ul>
<li>Add commands to prerequisite item 2 for adding measurements to the modified 123apps model allowing it to run simulations.
<li>Need to get the modified 123apps model added to the default GridAPPS-D platform distribution.
<li>Design changes to the GOSS-HELICS bridge allowing the deconfliction service to intercept CIM DifferenceBuilder messages sent to the GridLAB-D simulation.
<li>Support cooperation in competing apps with a modiified objective function given the intended phi function is non-linear/concave.
</ul>

# app-deconfliction/competing-apps

## Purpose

The competing-apps directory of the app-deconflicion repository contains both competing applications and the deconfliction pipeline framework that supports deconfliction methods. The competing apps are used to drive development and testing of methods, i.e., creating the conflict that must be resolved or deconflicted. The pipeline framework is where the deconfliction methods are plugged in providing all the functionality that is common regardless of the method--heuristics/rules, optimization, or cooperation/collaboration. This allows the deconfliction method code to be as lean as possible with the only responsibility being to perform device set-point deconfliction given benefit of a well-defined interface with the framework.

## Overview

The deconfliction pipeline framework follows the design described in the project foundational paper published in IEEE Access and is available at <https://ieeexplore.ieee.org/document/10107708>, specifically sections III-B and -C. There are methods in deconfliction-pipeline.py code directly corresponding to subsections in the foundational paper, e.g., SetpointProcessor, ConflictIdentification, DeconflictionSolution, and DeviceDispatcher.

The current full workflow for deconfliction consists of three components:
<ol>
<li>sim-sim or (sim)ulated (sim)ulator</li>
<li>competing apps, e.g., resilience app and decarbonization app</li>
<li>deconfliction pipeline</li>
</ol>

Sim-sim publishes messages containing time-series simulation parameters and updated device set-points and battery SoC. Competing apps subscribe to sim-sim messages to carry out their work determining and publishing new device set-point requests. The deconfliction pipeline subscribes to competing app set-point messages to perform the steps described in the foundational paper producing deconflicted set-points dispatched to devices. Sim-sim is subscribed to deconfliction pipeline device dispatch messages in order to update devices that determine simulation state, completing the workflow loop.

## DeconflictionMethod class interface

Deconfliction methods are implemented within classes named DeconflictionMethod--the filename for defining the class may distinguish the type of method, but the class name must be DeconflictionMethod. The only required methods for DeconflictionMethod are a constructor for initialization and a method named deconflict() that performs deconfliction when called. The class may define other methods or even call other classes or exchance messages with other processes needed to perform deconfliction at the discretion of the method developer, but the pipeline framework uses only \_\_init\_\_() and deconflict(). The pipeline framework imports and directly invokes these methods, so DeconflictionMethod is part of the deconfliction pipeline process as distinguished from competing apps and sim-sim that interface via inter-process messaging.

DeconflictionMethod classes perform deconfliction by taking device set-points across all competing apps given in the ConflictMatrix structure (maintained by the pipeline framework) and putting deconflicted device setpoints in the ResolutionVector structure returned by the deconflict() method. Alternatively, deconfliction can be performed using a structure named ConflictSubMatrix that represents only the device conflicts that were introduced as a result of the most recent competing app set-points message. A ResolutionVector is still created in this instance, but it is referred to as a partial ResolutionVector (because the devices represented are only those in ConflictSubMatrix, not all devices in ConflictMatrix), whereas using the entire ConflictMatrix to perform deconfliction must produce a full ResolutionVector. For using ConflictSubMatrix to produce a partial ResolutionVector, the pipeline framework takes the responsibility of filling in the remainder of the ResolutionVector creating the full ResolutionVector. This responsibility includes both filling in set-points for devices for which there is no conflict in ConflictMatrix and also set-points for devices that were previously resolved by the DeconflictionMethod class (the pipeline framework remembers past resolutions). Therefore only if a DeconflictionMethod class is willing to accept previous resolutions should ConflictSubMatrix be used to produce a partial ResolutionVector.

The interface of the pipeline framework and DeconflictionMethod classes consists of:
<ul>
<li>Signature of DeconflictionMethod __init__() constructor</li>
<li>Signature of and return value from DeconflictionMethod deconflict() method</li>
<li>Descriptions of ConflictMatrix and ResolutionVector structures for exchanging data between the pipeline framework and DeconflictionMethod class</li>
<li>MethodUtil class members used to provide ancilliary data from the pipeline framework to the DeconflictionMethod class</li>
</ul>

### DeconflictionMethod \_\_init\_\_() constructor

The signature of the constructor is:

```` bash
class DeconflictionMethod:
  def __init__(self, ConflictMatrix):
````

During pipeline framework initialization the configured DeconflictionMethod class is imported and the constructor called. A reference to the ConflictMatrix is passed in the constructor and typically the implementation will save this away in a class variable for later use in the deconflict() method. This same ConflictMatrix reference can be used throughout the deconfliction pipeline instance even as the ConflictMatrix is being continuously updated. The following snippet shows saving a reference to ConflictMatrix in the constructor:

```` bash
  def __init__(self, ConflictMatrix):
    self.ConflictMatrix = ConflictMatrix
````

### DeconflictionMethod deconflict() method

There are two forms for the deconflict() return value with the same method signature for each. The first form is:

```` bash
class DeconflictionMethod:

  def deconflict(self, timestamp):
    ...

    return ResolutionVector
````

The second form is:

```` bash
class DeconflictionMethod:

  def deconflict(self, timestamp):
    ...

    return (fullResolutionFlag, ResolutionVector)
````

Every time the pipeline framework receives a competing app set-points message, the framework processes the set-points to determine whether a new conflict has been introduced as a result of the message. If so, the deconflict() method is called to resolve those conflicts. Importantly, this can result in multiple deconflict() calls for a given timestamp or interval before the next sim-sim time-series data message is sent. As a consequence there will be multiple ResolutionVectors computed potentially leading to multiple set-points being dispatched to the same device effectively on top of each other. This is core to the design of the deconfliction pipeline and attempting to come up with a more sophisticated design that synchronizes set-points messages across multiple competing apps to eliminate dispatches on top of each other will not be as robust or timely in performing deconfliction. If each app could be counted on to supply set-points for each time interval this sort of synchronization would be possible, but it is the very nature of the apps being unpredictable in their timing that precludes synchronizing competing apps messages. Note that the pipeline framework only dispatches set-points to devices if that set-point would require an action by the device such as changing a regulator tap position or charging a battery. This means that dispatches on top of each other would only happen to perform an action, not when the previously resolved set-point still held.

The timestamp provided by the competing app sending the set-points message that resulted in the call to deconflict() is passed as an argument. This can serve as a reference to the DeconflictionMethod such as when stepping through ConflictMatrix to determine if a device set-point conflict is a new conflict or one that previously existed. Note that should ConflictSubMatrix be used instead of ConflictMatrix, then by definition with ConflictSubMatrix only containing new conflicts, this timestamp value passed to deconflict() will be associated with at least one of the apps for every device in ConflictSubMatrix.

The first form for the return value, only returning ResolutionVector, is used when ConflictMatrix is being processed and a full ResolutionVector is being returned. The second form, returning a tuple consisting of a Boolean flag and the ResolutionVector, is typically used when returning a partial ResolutionVector when processing ConflictSubMatrix, but can also be used for a full ResolutionVector. The flag is True for returning a full ResolutionVector, False for returning a partial ResolutionVector. Therefore, typically the return statement would be "return (False, ResolutionVector)" to denote a partial ResolutionVector. Not having to return a tuple with a full ResolutionVector is simply shorthand for what is believed to be the most common usage. Only if a tuple is returned and the first value is False will the pipeline framework fill in the remainder of the partial ResolutionVector to produce a full ResolutionVector.

### ConflictMatrix and ResolutionVector structures

ConflictMatrix is a multi-dimensioned dictionary. The first dimension is either the literal value 'setpoints' or 'timestamps' as ConflictMatrix maintains both from the competing app set-points messages it receives, i.e., ConflictMatrix['setpoints'] or ConflictMatrix['timestamps']. The second dimension differs based on the first dimension. If the first dimension is 'setpoints', then the second dimension is all the devices for which there are ConflictMatrix['setpoints'] entries, i.e., "for device in ConflictMatrix['setpoints'][device]:" will iterate over all devices. To carry through with ConflictMatrix['setpoints'], the final dimension is all the apps for which there are ConflictMatrix['setpoints'][device] entries. Therefore, "for app in ConflictMatrix['setpoints'][device]:" would iterate over all apps for a given device and when this for loop is nested under the previous one for devices, it would iterate over all entries of ConflictMatrix['setpoints'].

If the first dimension is 'timestamps', the the second dimension is all the apps for which there are ConflictMatrix['timestamps'] entries. There is no device dimension for ConflictMatrix['timestamps'] as when set-points for an app are updated they replace all previous device set-points and thus a single timestamp applies to all devices for that app.

Note that the name ConflictMatrix could be considered a misnomer for what is stored in the structure.  ConflictMatrix represents the most recent set-points (and associated timestamps) for an app over all devices for which that app has a set-point.  There may or may not be conflicts (different set-point values) in ConflictMatrix for any individual device between apps.

To summarize, the following code snippet will iterate over and print all ConflictMatrix entries and associated timestamps:

```` bash
for device in ConflictMatrix['setpoints']:
  for app in ConflictMatrix['setpoints'][device]:
    print('ConflictMatrix set-point for device: ' + device + ', app: ' + app +
          ', value: ' + str(ConflictMatrix['setpoints'][device][app]) +
          ', timestamp: ' + ConflictMatrix['timestamps'][app]))
````

ResolutionVector is similarly a multi-dimensioned dictionary where the first dimension is shared with ConflictMatrix being either 'setpoints' or 'timestamps'. The second dimension for ResolutionVector though is always the device whether it is either ResolutionVector['setpoints'] or ResolutionVector['timestamps'] as the first dimension. The purpose of ResolutionVector is to specify a single deconflicted or resolved set-point for each device across all apps and therefore there is no app dimension. To iterate over all set-point values in the ResolutionVector, the code would be "for device in ResolutionVector['setpoints']:". The ResolutionVector['timestamps'][device] value should be the most recent timestamp used to determine a deconflicted set-point over all apps for the device.

To summarize, the following code snippet will iterate over and print all ResolutionVector entires and associated timestamps:

```` bash
for device in ResolutionVector['setpoints']:
  print('ResolutionVector set-point for device: ' + device +
        ', value: ' + str(ResolutionVector['setpoints'][device] +
        ', timestamp: ' + ResolutionVector['timestamps'][device]))
````

Note on timestamps and deconfliction: The ConflictMatrix (and ConflictSubMatrix) maintain the timestamps associated with competing app set-point requests. This could indicate that some requests are "fresh" while others are quite "stale". If an app sends a single set-point request very early on in a simulation than either crashes, exits, or takes a very long nap, this original request remains in ConflictMatrix for the duration of the deconfliction session. Thus if a DeconflictionMethod ignores timestamps completely (all the sample DeconflictionMethod classes in the deconfliction-methods directory do just that!) then that very old request will be given equal weight to any subsequent and even brand new requests. Perhaps in future enhancements to the deconfliction pipeline framework there will be a policy for the framework itself to discard "stale" set-point requests, but currently they are part of ConflictMatrix (and ConflictSubMatrix if a brand new set-point request happens to conflict with a very old one). On a somewhat related note, in addition to timestamps within ConflictMatrix that are old, it's also possible based on the deconfliction pipeline design for there to be newly arriving set-point request messages from apps that are associated with timestamps older than the timestamp for the most recent sim-sim time-series data message. An app could be very slow to produce a set-point request causing this situation. Similarly, the DeconflictionMethod class may be very slow to produce a ResolutionVector from a deconflict() method call and thus the timestamp associated wih the device dispatch message could also be older than expected. In both these cases the deconfliction pipeline design considers these messages as valid even though they are not based on the most current time-series data message. The implications of discarding them are too significant although there may ultimately be some middle ground to give them special consideration rather than either discarding them or considering them equivalent to messages based on current data.

### MethodUtil members

The MethodUtil class contains references to data structures that can optionally be used by DeconflictionMethod classes for performing deconfliction. These structures are kept up to date by the deconfliction pipeline framework and must be treated as read-only by DeconflictionMethod classes. The data structures are:

ConflictSubMatrix: Dictionary identical in layout to ConflictMatrix, but containing only newly created conflicts as a result of the competing app set-points message that triggered the DeconflictionMethod deconflict() method to be called. This represents the minimal set of conflicts that must be resolved by deconflict() to create a partial ResolutionVector. If the ConflictSubMatrix is used instead of ConflictMatrix for performing deconfliction, either a class variable can be set to reference ConflictSubMatrix in the constructor analgous to how a class variable referencing ConflictMatrix would be set or MethodUtil.ConflictSubMatrix can be referenced directly when needed in deconflict() and other methods. Here is a typical usage setting a class variable in the constructor:

```` bash
# for import to work  sys.path() calls will first be needed to insure
# competing-apps/shared is in path

import MethodUtil

class DeconflictionMethod:
  def __init__(self, ConflictMatrix):
    self.ConflictSubMatrix = MethodUtil.ConflictSubMatrix
````

DeviceSetpoints: Dictionary containing set-points that were changed over the period since the last sim-sim time-series data message was sent. This does not represent the comprehensive set of current device set-points, but that could easily be generated from this dictionary with code similar to this snippet:

```` bash
import MethodUtil

class DeconflictionMethod:
  def __init__(self, ConflictMatrix):
    self.CurrentDeviceSetpoints = {}

  def deconflict(self, timestamp):
    # update dictionary of all current set-points
    for device, value in MethodUtil.DeviceSetpoints.items():
      self.CurrentDeviceSetpoints[device] = value
````

BatterySoC: Dictionary containing updated SoC values for all batteries at the time the most recent sim-sim time-series data message was sent. Iterating over all BatterySoC entries can be done with "for device, value in MethodUtil.BatterySoC.items()".

sparql_mgr: Reference to class defining various GridAPPS-D queries that may prove useful to DeconflictionMethod classes.

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
    ├── ieee123apps.xml
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

Note "..." indicates files similar to the one preceding and there are additional files in the repo that are superfluous to the existing deconfliction workflow.

## Prerequisites

<ol>
<li>
You must have the dockerized GridAPPS-D platform running which is available at https://github.com/GRIDAPPSD/gridappsd-docker. Follow the documentation there if you are unfamiliar with running the platform.
</li>

<li>
An updated version of the IEEE 123-bus model defining batteries and solar PVs not yet included in the standard GridAPPS-D platform distribution must be loaded after starting the platform.
The CIM model for the test feeder is exported to the sim-starter directory. Open the Blazegraph URL in the web browser and upload the file (ieee123apps.xml) using the "UPDATE" tab (http://localhost:8889/bigdata/#update).

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
To test the sim-sim module from a shell currently in the competing-apps directory of the app-deconfliction repo:

```` bash
$ cd sim-starter
$ ./run-sim.sh 123
````

If you get output starting with "Hit return" after some query output, this demonstrates successful initialization. You may hit return to verify sending a time-series data message and then do a ctrl-C exit. Most errors from running sim-sim are related to your GridAPPS-D platform setup. Missing modules include gridappsd-python (see item above) and matplotlib, which can be installed with:

```` bash
$ sudo pip install matplotlib
````
</li>

<li>
To test an optimization-based competing app (all optimization apps use the same base code varying only the objective function) assuming you are in the sim-starter directory:

```` bash
$ cd ../optimization-apps
$ ./run-resilience.sh 123
````

If you get output starting with "Initialized resilience optimization competing app" after some query output, this demonstrates successful initialization and you may do a ctrl-C exit. Modules likely to be missing include numpy, tabulate, and pulp. The following may prove helpful based on failed imports:

```` bash
$ sudo pip install numpy
$ sudo pip install tabulate
$ sudo apt-get install glpk-utils
$ sudo pip install pulp
````

Note that glpk-utils is needed by the PuLP optmization module. There are also workflow-based competing apps in the competing-apps/workflow-apps directory. If an optimization-based app initializes though, it is unlikely a workflow-based app will fail so that test can be skipped.
</li>

<li>
To test the core deconfliction-pipeline process assuming you are in the optimization-apps directory:

```` bash
$ cd ../deconfliction-pipeline
$ ./run-pipeline.sh 123 ../deconfliction-methods/compromise-rd-partial-method.py
````

If you get output starting with "Initialized deconfliction pipeline" after some query output, this demonstrates successful intialization and you may do a ctrl-C exit. Typically if sim-sim and an optimization competing app both complete initialization, the deconfliction-pipeline will initialize successfully as well.
</li>
</ol>

## Running deconfliction pipeline

The deconfliction pipeline can be run either as individual processes running in separate terminals or from a single wrapper shell script encompassing all processes. Running from the wrapper script will suffice for most users and developers, but a reason to run in separate terminals is for better control, including feeding time-series data messages from sim-sim on demand rather than automatically and being able to scrutinize running the diagnostic output of all processes. The bulk of this section will focus on the wrapper script, but here are a few pointers on running processes in separate terminals first.

One terminal will be needed for sim-sim, one for the deconfliction pipeline, and one for each of the competing apps to be run--currently five terminals if all three competing optimization apps (resilience, decarbonization, profit_cvr) are run. For each of the processes, there are shell scripts to invoke them rather than directly invoking Python from the command line. For sim-sim, the shell script is run-sim.sh in the sim-starter directory. For the deconfliction pipeline, the shell script is run-pipeline.sh in the deconfliction-pipeline directory. For competing optimization apps, in the optimization-apps directory, there is a "run-" shell script per optimization objective type, e.g., run-resilience.sh. For guidance on command line arguments to use with each of these shell scripts, take a look at the single wrapper script, run-deconfliction.sh, described next. Note that redirecting stdout to /dev/null as is done in the single wrapper script is not needed/desired when running in separate terminals as seeing the diagnostic output as it is generated from each process is a primary benefit of decoupling from the single wrapper.

The run-deconfliction.sh wrapper script in the competing-apps directory is your one-stop shop for running deconfliction. Comments at the top of the script provide guidance on command line arguments, with the basic usage being:

```` bash
$ ./run-deconfliction.sh <MODEL> <APPS> <METHOD> <DELAY>
````

where \<MODEL\> is a shorthand for looking up the full GridAPPS-D simulation request and feeder mrid. Currently, only the \<MODEL\> value supported for app deconfliction development is "123" which uses the updated IEEE 123-bus model, assuming that has been loaded per the guidance above.

\<APPS\> is a code composed of the first letters for each of the competing apps to run. The possible apps are resilience, code "r" or "R"; decarbonization, code "d" or "D", and profit_cvr, code "p" or "P". Thus, "rdp" would run all three apps and "rd" would run resilience and decarbonization without profit_cvr. By default optimization-based apps will be run, but by including "w" or "W" anywhere within the code, workflow-based apps will be run instead (optimization and workflow apps cannot be "mixed" together).

\<METHOD\> is the name of the DeconflictionMethod class file that will be used to perform deconfliction. The relative path to this file along with the filename (".py" extension is optional) from the competing-apps directory where the run-deconfliction.sh wrapper script resides (thus shell file completion can be used) is given for this argument. Note that this class file can exist completely outside of the competing-apps area of the app-deconfliction repo and that's typically the case as the deconfliction method subtasks of the overall project have their own top-level repo directories. However, some simple sample DeconflictionMethod classes are supplied in the deconfliction-methods directory.

These sample DeconflictionMethod classes can be used for testing the pipeline as well as guiding development for other subtasks by demonstrating the interface between the deconfliction pipeline framework and DeconflictionMethod classes. They are designed for use with different combinations of competing apps and implement two basic deconfliction methodologies--exclusivity and compromise. Further, some create full ResolutionVectors from ConflictMatrix, and some create partial ResolutionVectors from ConflictSubMatrix. There is a file naming convention for these different attributes to make it easy to see what each sample class does. The convention starts with either "exclusivity" or "compromise" followed by the familiar first letter-based code shared with the \<APPS\> command line argument for run-deconfliction.sh. After that is either "full" or "partial" for the type of ResolutionVector and finally, all end in "method". I.e., "compromise-rd-partial-method.py" is a compromise methodology (simply averages conflicting set-points between all those apps being considered in the compromise) between the resilience and decarbonization competing apps (could be used for either optimization- or workflow-based apps) that produces a partial ResolutionVector.

\<DELAY\> is an integer value that specifies how to handle sending sim-sim time-series data messages. If the value is positive, then it is the number of seconds to pause between automatically sending each message. If the value is negative, which is the default if no explicit \<DELAY\> is specified, then rather than pause a fixed time before sending the next message, new messages are triggered by device dispatch messages received by sim-sim from the pipeline. For each competing app set-points message, the pipeline sends a device dispatch message even in the case where no set-point changes were requested. Therefore sim-sim can wait for all apps to have processed the latest time-series data message before it sends the next data message. This is a nice feature for development and testing, but not one that would be part of a production deconfliction pipeline. That's because the nature of competing apps and deconfliction is that there is no guarantee an app will send set-point request at any fixed predictable interval, if ever. But, for development/testing, this "counting messages" mode allows processing both to happen as quickly as possible, and there not be any implications on the timeliness of messages sent either by competing apps or the deconfliction pipeline where the speed of the DeconflictionMethod deconflict() method could be an issue. Sim-sim will just wait until it gets all the messages it expects before moving on. Finally, a zero value for \<DELAY\> indicates to run-deconfliction.sh not to run sim-sim at all. In this case it must be started separately and this allows an interactive mode of sim-sim to be used that can't be used from run-deconfliction.sh where time-series data messages are only sent when the user hits return in the separate sim-sim terminal.

One special consideration regarding sending sim-sim data messages automatically on a fixed time interval is to make certain not to send messages before all competing apps have initialized. If messages are sent early the processing in competing apps will discard some messages leading to unpredictable and unrepeatable results, which is not a good outcome for code development/testing. To handle this situation the run-deconfliction.sh wrapper sends a "--wait" command line argument to the run-sim.sh script that causes it to sleep for 30 seconds (typical app initialization time is 20 seconds depending on the host system) before proceeding to invoke sim-sim and start sending data messages. For the default "counting messages" invocation of run-deconfliction.sh, there is no need to perform this 30-second wait step. Sim-sim can send the first data message as early as it wants. Then it will just wait for competing apps to initialize and process that first message with results returned via pipeline device dispatch message before considering sending the second data message. It is only a backlog of data messages that causes the issue.

With all of that, as example invocations of run-deconfliction.sh using the sample DeconflictionMethod classes, consider the following:

```` bash
$ ./run-deconfliction.sh 123 rd deconfliction-methods/compromise-rd-partial-method.py
$ ./run-deconfliction.sh 123 rdp deconfliction-methods/compromise-rdp-full-method.py
````

In the first invocation, the resilience and decarbonization optimization-based competing apps are run with a compromise deconfliction method that produces partial ResolutionVector results in the DeconflictionMethod class deconflict() method. In the second invocation, the profit app is added in as well with all three of those apps being considered in the compromise deconfliction, which this time produces full ResolutionVector results.


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

Deconfliction methods are implemented within classes named DeconflictionMethod--the filename for defining the class may distinguish the type of method, but the class name must be DeconflictionMethod. The only required methods for DeconflictionMethod are a constructor for initialization and a method named deconflict() that performs deconfliction when called. The class may define other methods or even call other classes or processes needed to perform deconfliction at the descretion of the method developer, but the pipeline framework uses only \_\_init\_\_() and deconflict(). The pipeline framework imports and directly invokes these methods so DeconflictionMethod is part of the deconfliction pipeline process as distinguished from competing apps and sim-sim that interface via inter-process messaging.

DeconflictionMethod classes perform deconfliction by taking device set-points across all competing apps given in the ConflictMatrix structure and putting deconflicted device setpoints in the ResolutionVector structure. Alternatively, deconfliction can be performed using a structgure named ConflictSubMatrix that represents only the device conflicts that were introduced as a result of the most recent competing app set-points message. A ResolutionVector is still created in this instance, but it is referred to as a partial ResolutionVector (because the devices represented are only those in ConflictSubMatrix) whereas using the entire ConflictMatrix to perform deconfliction must produce a full ResolutionVector. For using ConflictSubMatrix to produce a partial ResolutionVector, the pipeline framework takes over the responsibility of filling in the remainder of the ResolutionVector creating the full ResolutionVector. This responsibility includes both filling in set-points for devices for which there is no conflict in ConflictMatrix and also set-points for devices that were previously resolved by the DeconflictionMethod class (the pipeline framework remembers past resolutions). Therefore only if a DeconflictionMethod class is willing to accept previous resolutions should ConflictSubMatrix be used to produce a partial ResolutionVector.

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

During pipeline framework initialization the configured DeconflictionMethod class is imported and the constructor called. A reference to the ConflictMatrix is passed in the constructor and typically the implementation will save this away in a class variable for later use in the deconflict() method. This same ConflictMatrix reference can be used throughout the deconfliction pipeline instance even as the ConflictMatrix is being continuously updated. Typically a class variable will be set to the ConflictMatrix reference passed to the constructor in order to make it available to other class methods such as:

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

Every time the pipeline framework receives a competing app set-points message where the framework processes the set-points to determine that a new conflict has been introduced as a result of the message, the deconflict() method is called. Importantly, this can result in multiple deconflict() calls for a given timestamp or interval before the next sim-sim time-series data message is sent. As a consequence there will be multiple ResolutionVectors computed potentially leading to multiple set-point requests being dispatched to the same device effectively on top of each other. This is core to the design of the deconfliction pipeline and attempting to come up with a more sophisticated design that synchronizes set-points messages across multiple competing apps to eliminate dispatches on top of each other will not be as robust or timely in performing deconfliction. If each app could be counted on to supply set-points for each time interval this sort of synchronization would be possible, but it is the very nature of the apps being unpredictable in their timing that precludes synchronizing competing apps messages.

The timestamp provided by the competing app sending the set-points message that resulted in the call to deconflict() is passed as an argument. This can serve as a reference to the DeconflictionMethod such as when stepping through ConflictMatrix to determine if a device set-point conflict is a new conflict or one that previously existed. Note that should ConflictSubMatrix be used instead of ConflictMatrix, then by definition of ConflictSubMatrix only containing new conflicts, this timestamp value passed to deconflict() will be associated with at least one of the apps for every device in ConflictSubMatrix.

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

<ol>
<li>
You must have the dockerized GridAPPS-D platform running that is available at https://github.com/GRIDAPPSD/gridappsd-docker. Follow documentation there if you are unfamiliar with running the platform.
</li>

<li>
An updated version of the IEEE 123 node model defining battery and solar PV devices not yet included in the standard GridAPPS-D platform distribution must be loaded after starting the platform.

SHIVA, please insert here how to get and install the updated model!

Note that as long as docker containers are not cleared with the "./stop.sh -c" command, it is possible to stop and start the platform repeatedly without reloading this updated 123 node model.
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
Various other Python modules are required to run the different processes that are part of the deconfliction pipeline. The recommended approach is to run one-by-one each required deconfliction process through initialization to identify missing modules. Missing modules should be installed until the process successfully initializes at which time the same initialization test can be done for the next process. The following steps walk through running each deconfliction process and covers the most likely missing modules.
</li>

<li>
To test the sim-sim module from a shell currently in the competing-apps directory of the app-deconfliction repo:

```` bash
$ cd sim-starter
$ ./run-sim.sh 123
````

If you get output starting with "Hit return" after some query output, this demonstrates successful initialization. You may hit return to verify sending a time-series data message and then do a ctrl-C to exit.

Any errors from running sim-sim are typically related to your GridAPPS-D platform setup and not missing Python modules assuming you have installed the gridappsd-python module as described above.
</li>

<li>
optimization apps next
</li>
</ol>

## Running deconfliction pipeline


# Lane Free Traffic Simulation Models in SUMO

## Introduction
This project aims to have multiple Lane Free Traffic control (LFT) algorithms at one place. It uses the customized 
version of the SUMO (TrafficFluid) by TU Crete, and uses its C++ interface.

## Installation
### Prerequisites
1. The main algorithms are written in C++. You need to have a C++ compiler installed on your machine. 
2. For smoother compilation, use Visual Studio (**not Visual Studio Code**) and the provide cmake file to compile the project.
3. The code also uses simpleini library for reading simulation configuration files. Use the following steps to install it using vcpkg. vcpkg is a package manager for C++.
	- Clone the vcpkg to appropriate folder: `git clone https://github.com/microsoft/vcpkg.git`
	- Go to the vcpkg folder and run the following command: `.\bootstrap-vcpkg.bat`
	- Integrate the vcpkg with visual studio: `.\vcpkg integrate install`
	- Install the simpleini library: `.\vcpkg install simpleini`
### Setting up the LFT repository folder and compiling the plugin
1. Clone the current LFT repository to a local folder.
2. Open the cloned folder in visual studio. This should automatically confgiure the project using CMakeLists.txt file.
3. After the configuration of cmake is finished, click **Build->Build all**. This should compile the LFT plugin.
   - After successful compilation, you will see the compiled plugin **libLaneFreePlugin.dll** in the folder **out\build\x64-Debug**.
4. The TrafficFluid simulator should be able to detect the compiled plugin. Therefore, add the path of the compiled plugin (**out\build\x64-Debug**) to the environment variable **PATH**.

### Download the TrafficFluid simulator
1. Download the TrafficFluid simulator from the website: https://www.trafficfluid.tuc.gr/en/trafficfluid-sim
2. Extract the TrafficFluid simulator to the folder: **python_src\SUMO**.
   - Make sure that you extract the main folders into **python_src\SUMO**, i.e. you should be able to see the folders such as **bin**, **netedit**, etc immediately under **python_src\SUMO**. 
3. Open the TrafficFluid simulator by opening the file **python_src\SUMO\bin\sumo-gui.exe**
   - If the above steps are followed correctly, especially setting up env variable for .dll, you should be able to see the SUMO GUI without any error.
4. You will still be unable to start a simulation, as the current LFT plugin also uses **.ini** files to set simulation parameters. The next section discusses them in detail.

## LFT Plugin, C++ code and Simulation Configuration
The C++ code under the **src** folder basically uses the C++ interface of the TrafficFluid simulator. However, for the
purpose of conciseness and bringing multiple LFT methods at one place, the current LFT plugin code uses **.ini** 
configuration files to select among various LFT methods and set their parameters.

### Which .ini files are used?
- **src\default_config\default_config.ini**:		This is the default configuration file that lists all possible parameters of LFT plugin along with their default and possible values, and short description of each parameter. This serves as the main templete setting parameters for LFT plugin.
- **scenario specific config.ini**: 		Besides the .ini file for default configuration, each simulation scenario provides its own configuration file. The simulation parameters in this file override the default configuration file.

### How to provide .ini files to a simulation
Since the C++ interface of the TrafficFluid simulator is already fixed, there was no direct way to provide the
LFT configuration files to a simulation. Therefore, the current LFT plugin repository uses environment variables to 
provide these .ini files to the C++ code. 
<span style="color:red">Warning: The SUMO simulation with the LFT plugin will not start if these variables are not set.</span>

The following environment variables are used:

- **DEFAULT_CONFIG_FILE**: 		
  - This environment variable provides the path to the default configuration file. 
  - The file name of the default configuration can be anything. 
  - The file **src\default_config\default_config.ini** provides the main .ini templete to be used.
  - Any customized default configuration file must define all the parameters listed in the **src\default_config\default_config.ini**.


- **CONFIG_FILE**:
  - This environment variable must provide the path to the scenario specific configuration file.
  - Use the file **src\default_config\default_config.ini** as starting point and change any parameter you want for simulation.

## A short tutorial on how to run a basic SUMO simulation scenario
The repository provides a simple ring road example to show how to start a simulation with the LFT plugin. The SUMO files
for the ring road are provided under **python_src\SUMO_Runs\example_ring_road**. 
See the following steps to start this SUMO example using LFT plugin:

1. **Compile the C++ code**: 
   - Follow the instructions in the **Installation** section to compile the C++ code and set the env variable to the .dll file.
2. **Set the envrionment variables for the default and scenario specific config (.ini) files**
   - Create an environment variable **DEFAULT_CONFIG_FILE** and set its value to the path **...\src\default_config\default_config.ini**
   - Create an environment variable **CONFIG_FILE** and set its value to the path of the scenario specific configuration file.
     - An example config.ini for the ring road example is provided. Thus, set the value of **CONFIG_FILE** to the path of the file **...\python_src\SUMO_Runs\example_ring_road\config.ini**.
3. Open the TrafficFluid (SUMO) simulator by running the file **python_src\SUMO\bin\sumo-gui.exe**.
4. Open the SUMO file **python_src/SUMO_Runs/example_ring_road/ring.sumocfg** in SUMO GUI.
5. Start the simulation.

## Python Scripts

The previous section described how to start a simulation with the LFT plugin using SUMO GUI. However, setting up a 
different environment variable for each simulation scenario is cumbersome and also does no allow running multiple 
simulations in parallel. Therefore, the current repository provides python scripts to help start and stop the 
simulations. It also helps to start multiple SUMO simulations in parallel.

- **parallel_run.py**: 		
  - It is a script that takes csv file for simulation configurations and starts multiple simulations in parallel.
  - Each simulation is started in a separate python subprocess and the results are produced in corresponding folders. 
  - The script takes the paths of default and scenario specific configuration files from the csv file and sets them as environment variables for each subprocess. The output path is also taken from the csv file.
  - The script **example_create_scenario.py** shows how to create this csv file for the ring road example. See the next point for details.
  
- **example_create_scenario.py**: 		
  - The repository provides SUMO files and LFT configuration for a simple ring road example.
    - The necessary SUMO files are provided in the folder **python_src\SUMO_Runs\example_ring_road** folder.
    - The example LFT configuration are provided as parameter values in the python **example_create_scenario.py**.
  - The Python file **example_create_scenario.py** uses the above SUMO files to create individual simulation scenarios.
    - It generates a csv file with the simulation parameters and the paths to the default and scenario specific configuration files.
    - After running **example_create_scenario.py**, each simulation scenario will have its own folder under **python_src\SUMO_Runs\example_ring_road\Experiments**. The folder name is the same as the scenario name.
    - The csv file is used by the **parallel_run.py** to start the simulations in parallel.
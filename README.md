# Lane Free Traffic Simulation Models in SUMO

## Introduction
This project aims to have multiple Lane Free Traffic control algorithms at one place. It uses the customized version of the SUMO (TrafficFluid) by TU Crete, and uses its C++ interface.

## Installation
- The main algorithms are written in C++. You need to have a C++ compiler installed on your machine. 
- For smoother compilation, use visual studio and the provide cmake file to compile the project.
- The code also uses simpleini library for reading simulation configuration files. Use the following steps to install it using vcpkg. vcpkg is a package manager for C++.
	- Clone the vcpkg to appropriate folder: `git clone https://github.com/microsoft/vcpkg.git`
	- Go to the vcpkg folder and run the following command: `.\bootstrap-vcpkg.bat`
	- Integrate the vcpkg with visual studio: `.\vcpkg integrate install`
	- Install the simpleini library: `.\vcpkg install simpleini`
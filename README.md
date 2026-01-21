# Behavior Inference in Dynamic Surveillance Settings

[![License](https://img.shields.io/badge/license-CC%20BY--NC-blue.svg)](LICENSE)
<!-- ![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg) -->

## Table of Contents
- [Behavior Inference in Dynamic Surveillance Settings](#behavior-inference-in-dynamic-surveillance-settings)
	- [Table of Contents](#table-of-contents)
	- [Introduction](#introduction)
	- [Demonstration](#demonstration)
	- [Project Structure](#project-structure)
	- [Preparation](#preparation)
	- [Usage](#usage)
		- [Running](#running)
		- [Debugging](#debugging)
	- [License](#license)

## Introduction
Some nodes need a cold start to load initial data.
The list of these nodes is specified in the constructor of [ColdStartManager](src/rt_bi_runtime/rt_bi_runtime/ColdStartManager.py) via `__awaitingColdStart` property.

It then calls `__triggerNextColdStart` for each of them and awaits a response in `__onColdStartDone` from the respective node.

## Demonstration
[WIP]

## Project Structure
[WIP]

## Preparation
[WIP]
1. Setup Apache Jena
2. Add the data from the [rdf](src/rt_bi_runtime/rdf) folder.
3. Update the URL to Jena in [rdf.yaml](src/rt_bi_runtime/config/rdf.yaml)
4. ...

## Usage
[WIP]
### Running
1. Run `./scripts/debug.launch.sh`. This will launch the following nodes:
   1. `rt_bi_runtime` using `all.launch.py`
   2. `rt_bi_behavior` using `ba.launch.py`
   3. `rt_bi_emulator` using `map.launch.py`
   4. ...

### Debugging
To use log statements, in the constructor of the node add
`"loggingSeverity": Ros.LoggingSeverity.WARN`
in the constructor of the node.

## License

This project is licensed under the Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) License - see the [LICENSE](LICENSE) file for details.

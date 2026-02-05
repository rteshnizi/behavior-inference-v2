# Behavior Inference in Dynamic Surveillance Settings

[![License](https://img.shields.io/badge/license-CC%20BY--NC-blue.svg)](LICENSE)

## Table of Contents
- [Introduction](#introduction)
- [Project Overview](#project-overview)
- [Architecture](#architecture)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Key Concepts](#key-concepts)
- [Development](#development)
- [License](#license)

## Introduction

This project implements a **Behavior Inference** system for dynamic surveillance settings, built on top of ROS 2 (Robot Operating System).
The system monitors moving targets within dynamic sensor fields of view and reasons about their spatial-temporal behavior using connectivity graphs, behavior automata, and semantic knowledge stored in RDF format.

The core algorithm constructs **Connectivity Graphs** and **Metric I-Graphs** (Information Graphs) to track relationships between named regions, FOV (field-of-view), and shadows (unobserved areas).
A **Behavior Automaton** then evaluates these spatiotemporal events against user-defined behavioral predicates.

### Cold Start Mechanism

Some nodes need a cold start to load initial data.
The list of these nodes is specified in the constructor of [ColdStartManager](src/rt_bi_runtime/rt_bi_runtime/ColdStartManager.py) via `__awaitingColdStart` property.
It then calls `__triggerNextColdStart` for each of them and awaits a response in `__onColdStartDone` from the respective node.

## Project Overview

The system processes:
- **Map regions**: static, dynamic, or affine (moving) polygons.
- **FOV**: sensing polygons that track targets/tracklets.
- **Targets**: moving entities whose behavior we are estimating.
- **Behavioral specifications**: automata with state transitions based on predicates.

Using **Apache Jena Fuseki** as the RDF triple store, the system queries semantic knowledge about spatial regions using SPARQL, enabling rich predicate-based reasoning.

## Architecture

```
┌────────────────────┐      ┌────────────────────┐
│   rt_bi_emulator   │      │   rt_bi_runtime    │
│  (Simulation/Test) │      │  (Data Dictionary  │
│                    │      │   & RDF/SPARQL)    │
└─────────┬──────────┘      └─────────┬──────────┘
          │                           │
          │    ROS 2 Topics/Services  │
          ▼                           ▼
┌─────────────────────────────────────────────────┐
│                  rt_bi_core                     │
│     (Spatial Polygons, Regions Subscriber)      │
└─────────────────────┬───────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────┐
│               rt_bi_eventifier                  │
│  (Connectivity Graphs, Metric I-Graphs, CTCD)   │
└─────────────────────┬───────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────┐
│               rt_bi_behavior                    │
│   (Behavior Automaton, Token Tracking, DOT)     │
└─────────────────────────────────────────────────┘
```

## Package Structure

| Package | Description |
|---------|-------------|
| **rt_bi_interfaces** | ROS 2 message and service definitions for inter-node communication |
| **rt_bi_commons** | Shared utilities, base classes, geometry helpers, RViz rendering, and ROS utilities |
| **rt_bi_core** | Core spatial types (Polygon, SensingPolygon, AffinePolygon, TargetPolygon, Tracklet) |
| **rt_bi_emulator** | Test harness for emulating maps, sensors, and targets from YAML configurations |
| **rt_bi_eventifier** | Event generation via Connectivity Graphs and Metric I-Graphs with CTCD (Continuous-Time Collision Detection) |
| **rt_bi_behavior** | Behavior Automaton implementation with token-based state tracking and Graphviz rendering |
| **rt_bi_runtime** | Runtime environment, Cold Start Manager, RDF/SPARQL data dictionary (Apache Jena Fuseki interface) |

### Source Directory Layout

```
src/
├── rt_bi_interfaces/     # ROS 2 message/service definitions
├── rt_bi_commons/        # Shared utilities and base classes
├── rt_bi_core/           # Core spatial data structures
│   └── Spatial/          # Polygon types, Tracklets
├── rt_bi_emulator/       # Simulation nodes
│   ├── config/           # YAML configs for AVs, targets
│   └── launch/           # ROS 2 launch files
├── rt_bi_eventifier/     # Event processing
│   ├── Model/            # ConnectivityGraph, MetricIGraph, CTCD
│   └── config/           # RViz configurations
├── rt_bi_behavior/       # Behavior automaton
│   ├── Model/            # BehaviorAutomaton, TransitionStatement
│   └── config/           # BA specs, Lark grammar
└── rt_bi_runtime/        # Runtime & semantic data
    ├── rdf/              # RDF ontology (Turtle format)
    ├── sparql/           # SPARQL query templates
    └── config/           # Fuseki server configuration
```

## Prerequisites

- **ROS 2** (tested on Humble)
- **Python 3.10+**
- **colcon** build tool
- **Apache Jena Fuseki** (or any RDF store with SPARQL endpoints) for RDF triple store
- **RViz2** for debugging and visualization
- **rosbridge_server** for web-based behavior automaton visualization

### Python Dependencies

- `rclpy` - ROS 2 Python client library
- `networkx` - Graph data structures
- `shapely` - Geometric operations
- `lark` - Parsing library for transition grammar
- `requests` - HTTP client for Fuseki SPARQL endpoint
- `geometry_msgs`, `visualization_msgs` - ROS 2 message types

## Installation

1. **Build with colcon:**
   ```bash
   colcon build
   ```

2. **Source the workspace:**
   ```bash
   source install/local_setup.bash
   ```

## Configuration

### Apache Jena Fuseki Setup

1. Install and start Apache Jena Fuseki server
2. Create a dataset named `rt-bi-v2-1` (or update config)
3. Load the RDF ontology from [src/rt_bi_runtime/rdf/](src/rt_bi_runtime/rdf)
4. Update the Fuseki URL in [rdf.yaml](src/rt_bi_runtime/config/rdf.yaml):
   ```yaml
   rt_bi_runtime:
     dd_rdf_1:
       ros__parameters:
         fuseki_server: "http://<your-fuseki-host>:8090"
         rdf_store: "rt-bi-v2-1"
   ```

### Behavior Automaton Configuration

Define your behavior specification in `src/rt_bi_behavior/config/ba.yaml`:
```yaml
rt_bi_behavior:
  ba1:
    ros__parameters:
      states:
        - "Q0"
        - "Q1"
        - "Q2"
      transitions_from:
        - "Q0"
        - "Q1"
      transitions_predicate:
        - 'name == "B"'
        - 'name == "C"'
      transitions_to:
        - "Q1"
        - "Q2"
      start: "Q0"
      accepting:
        - "Q2"
```

## Usage

### Running the System

Launch the complete system using:
```bash
./scripts/debug.launch.sh
```

This starts:
- **rt_bi_runtime** (`all.launch.py`) - Cold Start Manager and RDF Data Dictionary
- **rt_bi_behavior** (`ba.launch.py`) - Behavior Automaton with web visualization
- **rt_bi_emulator** (`map.launch.py`) - Map and sensor emulation
- **rt_bi_eventifier** (`ev.launch.py`) - Eventifier

### VS Code Tasks

The project includes VS Code tasks for development:
- `rbc: build` - Build with colcon
- `rbc: source` - Source the workspace
- `rbc: launch` - Launch the system

### Web Visualization

The behavior automaton state can be visualized via a web interface at `http://localhost:5000` (Flask + rosbridge websocket) showing a Graphviz DOT rendering of the automaton with token positions.

## Key Concepts

### Polygon Types

| Type | Description |
|------|-------------|
| **StaticPolygon** | Fixed map regions that don't change over time |
| **DynamicPolygon** | Map regions that change shape/position over time |
| **AffinePolygon** | Map regions with affine transformations |
| **SensingPolygon** | Sensor field-of-view polygons that can observe targets |
| **TargetPolygon** | Moving targets/entities being tracked |

### Connectivity Graph

Represents spatial adjacency at a single time instant:
- **Shadows** - Unobserved regions (map minus sensor coverage)
- **Anti-Shadows** - Observed regions (intersection of map and sensor)
- **Edges** - Connectivity between adjacent regions

### Metric I-Graph (Information Graph)

A temporal stack of Connectivity Graphs connected by temporal edges, enabling reasoning about target movement possibilities across time.

## Development

### SPARQL Template System Overview

The system uses a template-based approach for generating SPARQL queries at runtime. Template files (`.rq`) in [src/rt_bi_runtime/sparql/](src/rt_bi_runtime/sparql/) contain placeholder comments that are dynamically replaced with query-specific content.

#### Template Files

| Template | Purpose |
|----------|---------|
| `sets.rq` | Discovers all SpaceTime regular sets and classifies them (static, dynamic, affine, temporal) |
| `geometry.rq` | Fetches polygon vertex coordinates for specific set IDs |
| `intervals.rq` | Fetches time interval bounds for dynamic/temporal sets |
| `channel.rq` | Fetches channel information for affine sets |
| `ids.rq` | Generic ID-based query template |

#### Placeholder Comments

Placeholders are special comment markers in the `.rq` files that get replaced at runtime:

| Placeholder | Config Key | Replaced With |
|-------------|------------|---------------|
| `# SELECT ####...` | `placeholder_select` | Additional SELECT variables derived from predicates |
| `# ID VALUES ####...` | `placeholder_ids` | List of IRI values like `<https://...>` for filtering |
| `# WHERE ####...` | `placeholder_where` | OPTIONAL clauses to extract predicate-related properties |
| `# BIND ####...` | `placeholder_bind` | BIND statements converting property values to boolean flags |
| `# ORDER ####...` | `placeholder_order` | Additional ORDER BY variables |
| `# FILTER_TRAVERSABILITY ...` | `placeholder_filter_traversability` | Target-specific traversability constraints |

#### Runtime Flow

1. **Predicate Parsing**: Behavior automaton predicates (e.g., `name == "BuildingA"`) are parsed using a Lark grammar (`transition.lark`)
2. **SPARQL Transformation**: The `PredicateToQueryStr` transformer converts predicates into:
   - SELECT variables (e.g., `?Name`)
   - WHERE clauses (e.g., `OPTIONAL { ?regularSetId property:material/property:name ?Name }`)
   - BIND statements (e.g., `BIND(?Name = "BuildingA" AS ?p_0)`)
3. **Template Filling**: `RdfStoreNode.__fillTemplate()` replaces all placeholders with the generated SPARQL fragments
4. **Query Execution**: The filled query is sent to Apache Jena Fuseki via HTTP

#### Example Transformation

Given predicate: `material.name == "cement"`

```
BA predicate: 'material.name == "cement"'
                    ↓
SparqlTransformer.transformPredicate()
                    ↓
┌─────────────────────────────────────────────────────────┐
│ whereClause: "OPTIONAL { ?regularSetId                  │
│               property:material/property:name           │
│               ?MaterialName }"                          │
│                                                         │
│ varBindings: "BIND (?MaterialName = "cement"            │
│               AS ?p_0)"                                 │
│                                                         │
│ variables:   "?p_0"                                     │
└─────────────────────────────────────────────────────────┘
                    ↓
__fillTemplate() replaces placeholders in sets.rq
```

The final SPARQL query becomes:
```sparql
SELECT ?regularSetId ?static ?dynamic ?affine ?temporal ?p_0
WHERE {
    ?regularSetId a/rdfs:subClassOf* class:SpaceTime .
    OPTIONAL { ?regularSetId property:material/property:name ?MaterialName }
    BIND (?MaterialName = "cement" AS ?p_0)
    FILTER (?p_0)
}
```

### Debugging

Enable verbose logging in node constructors:
```python
newKw = {
    "loggingSeverity": Ros.LoggingSeverity.DEBUG,
    ...
}
```

### Adding New Predicates

Transition predicates follow a Lark grammar defined in `transition.lark`. Example predicate:
```
name == "BuildingA"
```

SPARQL query templates in `src/rt_bi_runtime/sparql/` are dynamically populated with predicate-derived WHERE clauses.

### RViz Visualization

Configure rendering modules in `src/rt_bi_eventifier/config/ev.yaml`:
```yaml
renderModules:
  - c_graph    # Connectivity Graph
  - ctcd       # Continuous-Time Collision Detection
  - i_graph    # Information Graph
```

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)** License - see the [LICENSE](LICENSE) file for details.

---

**Maintainer:** Reza Teshnizi (reza.teshnizi@gmail.com)
**Version:** 0.9.0

# autoware_core_vehicle

## Overview

The `autoware_core_vehicle` package is a part of the Autoware Core system, responsible for interfacing with vehicle hardware and managing vehicle-specific operations. This package provides essential functionalities for vehicle control and communication within the Autoware ecosystem.

## Features

- Vehicle driver interface
- Loading URDF model

## Configuration

The package can be configured using the following parameters:

- `vehicle_model`: Specifies the package that contains urdf for vehicle model to be used.
- `sensor_model`: Specifies the package that contains urdf for sensor models to be used.

## Launch Files

- `vehicle.launch.xml`: Main launch file for vehicle operations.

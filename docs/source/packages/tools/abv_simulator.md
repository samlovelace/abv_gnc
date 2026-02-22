# abv_simulator

A numerical integration module for simulating the full state of the ABV.

## Purpose

The `abv_simulator` package is responsible for simulating how the ABV would move based on the thrust direction vector that was applied. This package is only needed for running/testing the GNC stack without running on the actual ABV.

## Usage

Listens for thruster commands via UDP from the `abv_controller`. Computes the theoretical thrust vector input based on the force of each thruster and which thrusters are fired. Additionally, has the ability to add noise to simulate a noisy sensor module.

## Topics

### Published

- `/abv/sim/state` - the simulated state of the ABV

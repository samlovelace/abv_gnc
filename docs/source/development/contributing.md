# Contributing

This document outlines some considerations and rules for contributing to the ABV software stack.

## Development Philosophy

The ABV stack is designed as a modular robotics software platform.

Core principles:

- Clear separation of responsibilities (navigation, guidance, control, tools)
- Strict interface boundaries defined in `abv_msgs`
- No cross-layer coupling
- Hardware-specific code isolated from algorithmic logic
- Simulation-first validation (when possible) before hardware testing

All contributions should preserve these principles.

---

## Workspace Structure

The repository follows a standard ROS 2 workspace layout:

- `src/` — ROS2 packages
- `docs/` — Documentation
- `tools/` — Supporting scripts and utilities

Each ROS2 package should:

- Own a single primary responsibility
- Declare dependencies explicitly
- Avoid unnecessary coupling to other packages

---

## Coding Standards

### C++ Guidelines

- Use modern C++ (C++17 or later)
- Prefer RAII and smart pointers
- Avoid raw `new` / `delete`
- Enforce const-correctness
- Use Eigen for linear algebra
- Separate headers and source files cleanly
- Avoid global state

### ROS2 Guidelines

- One node per executable
- Declare all parameters explicitly
- Topic names must be documented

---

## Adding a New Package

To add a new ROS 2 package:

1. Create the package inside `src/`
2. Use `ament_cmake`
3. Declare all dependencies in `package.xml`
4. Keep functionality focused and modular
5. Add documentation for the package
6. Update documentation index if necessary

New packages should not introduce circular dependencies.

---

## Adding or Modifying Messages

All shared interfaces are defined in `abv_msgs`.

When adding or modifying a message:

- Document all fields in the `.msg` file
- Specify units for numerical values
- Specify coordinate frame conventions
- Preserve backward compatibility when possible
- Update documentation if behavior changes

All frame conventions must be consistent across the stack.

---

## Testing

Little to no automated testing has been developed. If that is of interest to the reader, feel free to add unit and or functional tests to the repo.

## Pull Request Guidelines

All contributions should be submitted via pull request.

Recommended workflow:

- Create a feature branch
- Make focused commits with clear messages
- Ensure the workspace builds successfully
- Run tests (if applicable)
- Submit a pull request with a clear description

Avoid committing directly to main.

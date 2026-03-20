# abv_bridge

The `abv_bridge` is responsible for converting internal message types to the corresponding external type, as well as aggregating and
relaying information outward.

## Responsibilities

### Simple

- Navigation Data: the abv uses only a representation of its available degrees of freedom (x, y, yaw). The bridge will convert this to a full 12-dof state representation .

Steps to perform:

- Subscriber in bridge listening to /abv/state topic
- Translate data into full 12dof vector, assign zeros for unavailable DOFs
- Send translated data on outgoing interface, DDS, protobuf, UDP etc.

Assume outgoing navigation data is always full 12-dof state.

### Aggregate

- Controller Status: the

## Commands

- The bridge will take in 6DOF vehicle commands and convert to 3DOF. DOFs not available to the ABV will essentially be ignored.

## Interface

- The bridge will have a protocol agnostic incoming interface. This allows for reuse with different commanding modules ie, DDS, UDP, Protobuf.

## Classes

- Core: owns main loop, owns customer implementations
- INavigationSender: owned by Core. Customer implementation to convert ABV type to their type

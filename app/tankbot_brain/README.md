# TankBot app

The TankBot app (Flutter, iOS today): one app, three roles chosen at launch.

- **Mounted brain**: the phone rides on the robot (camera tracking, mapping, navigation, control server).
- **Brain in hand**: the same brain off the robot, tracking with the lidar alone.
- **Controller**: a remote for a brain on the network.

See the [project README](../../README.md), the [user guide](../../docs/user-guide.md) and the
[app architecture](../../docs/app-architecture.md). The Dart package is still named `tankbot_brain`
internally (folder, imports, bundle id `com.johnsonfarms.tankbotBrain`) so installed phones keep their data.

`flutter test` must pass before every install.

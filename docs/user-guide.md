# User guide

How to drive, map and explore with a TankBot-platform robot and a phone brain.

## Getting to the controls

| Address | What |
|---|---|
| `http://tankbot.local/` | the robot's own page: always works, even without a brain. Buttons, joystick, arrow keys / W A S D (Space stops), speed, trim, **Setup** |
| `http://tankbot.local/brain` | **bookmark this.** Takes you to the brain's full controls wherever the brain is |
| the app, Controller role | the same full controls on a phone |

The robot's page shows **Open full controls** whenever a brain is running.

## The full controls

The top bar on every page: robot name, status lights (**Brain**, **Robot**, **Lidar** scans/s,
**Position**, **Map** mode), the tabs, and a link to the robot's page. Green is good, amber means
working on it, red needs attention.

### Drive page

**The map**: drag to look around, scroll or pinch to zoom, **+ / -**, **(o)** to follow the robot again,
**Radar** for the robot's-eye view, **Key** for the colour key:

| On the map | Meaning |
|---|---|
| teal | walls (the map) |
| grey | open floor (the map); black is unexplored |
| yellow dots | what the lidar sees right now (should sit on the teal walls) |
| orange squares | low obstacles from the depth camera (phones with a depth sensor) |
| teal dots | ultrasonic / ToF readings right now |
| red lines | no-go lines |
| orange rings | bumps and stalls; dashed = temporary (fade in 15 min), solid = confirmed |
| red rings | drop-offs; dashed / solid as above |
| blue line and circle | the route and goal |
| purple diamond | the next place exploration will go |
| white circle with a line | the home spot (where the map started) |
| orange arrow | the robot (grey when it isn't sure where it is) |

**Status cards** (top left) say what the robot is doing or needs, with buttons: Stop, Try again,
Set position, I'm at home. Short confirmations appear at the bottom and fade.

**Panel**: joystick, **Stop**, max speed, **Go to...** (then click the map; you can drag and zoom
first), **Edit map** (move, eraser, no-go lines, undo), **Set position...**, **New map here**, and
**Mapping**: Explore / Maintain / Off.

Keyboard: arrows or W A S D drive, Space stops, Esc cancels. Leaving the page stops the robot.

### Mapping modes

| Mode | Use it for | What happens to the map |
|---|---|---|
| **Explore** | building a map, adding rooms, after moving furniture | everything you drive past is added. Saves a restore point when you switch to it |
| **Maintain** (default) | everyday driving | grows into unmapped areas; elsewhere a spot changes only after the change is seen on 3 separate passes over 10+ minutes (moved furniture yes, people and pets no) |
| **Off** | just driving | never changed (it is still used to know where the robot is) |

When the robot drives itself, Explore acts as Maintain, except during autonomous exploration.

### Exploring on its own

1. Set **Mapping** to **Explore**. **Explore on its own** appears below.
2. Without a cliff (drop-off) sensor you are asked to acknowledge that the robot can't detect drops and
   that you'll watch it. Block off stairs or draw no-go lines across them first.
3. It drives to the edges of what it has mapped (its own room first, then through doorways), pausing
   in new areas to let the map form, and drives home when nothing reachable is left. The card says why
   it finished (how many openings it found and why it skipped them).

Stop, the joystick, any drive key or leaving the page ends it. It also stops by itself if the robot
loses track of where it is (removing the last stretch of mapping), stops responding, or after 30 min.

### Set position

When the robot is lost or wrong: **Set position...**, press on the map where it really is and drag
toward the way it faces. The lidar fine-tunes the placement. A tap without dragging keeps its heading.

### Maps page

Save, rename, load, delete maps; **Find me again**, **I'm at home**; **Clear remembered drop-offs**;
**Restore an earlier version** (restore points from before Explore mode and before exploring).
The quality line shows lidar corrections, loop closures, straightening and what Maintain is doing.

### Bot page

The robot's size, drive type, how the phone is mounted (upright or on its side) and every sensor:
what it is plugged into, what it's used for, which way it faces, where it sits, its live reading.
Drag sensors in the top and side views. **Save to robot** sends sensor changes to the robot.

### Settings

Changes save as soon as you leave a field (a "Saved" note confirms it).

- **Driving**: steering trim, minimum power to move, cruise power.
- **Safety**: obstacle stop, stop and pass distances, depth camera stop distance and height.
- **Position tracking**: camera + lidar (lidar alone when the camera drifts) or lidar only.
- **Map straightening**: wall alignment; use GPS when better than N metres (outdoors).
- **Robot / This brain**: what is attached and what the phone can do.
- **Sensor log**: record GPS, compass and position for testing.

## Changing brain phones

The robot keeps what a new brain needs: its sensors (positions, roles, thresholds), its robot-level
settings (platform size, safety distances, power levels, mapping and tracking preferences) and a
compact copy of the most recent map (walls, open floor, no-go lines, edits). A new brain picks all of
it up when it connects and, if it has no map of its own yet, loads the robot's map and finds itself
on it. Measure the new phone's camera position on the Bot page; that one is per phone.

The robot's copy updates when the map changes meaningfully (at most every 5 minutes, never while the
robot drives itself). The Maps page shows what the robot holds. Full maps, with their raw scans for
straightening, stay on the brain.

## Troubleshooting

| You see | Try |
|---|---|
| **Robot not responding** / no lidar | check the battery and power; the brain reconnects by itself |
| yellow dots don't line up with the walls | **Set position** |
| "Camera tracking is drifting here" | normal in dim rooms or with plain walls; the lidar takes over. Turning the lights on helps |
| the map got messy | **Maps > Restore an earlier version** |
| Go to says there's no path | look for rings or no-go lines boxing it in; the eraser removes marks |
| exploring went home at once | read the reason on the card (openings found and why they were skipped); check its position |
| the robot keeps stopping in front of something | it backs up and goes around by itself; after 4 tries it gives up on that spot |
| it drives but doesn't move (caught on something low) | detected after ~2 s: marked and backed away from automatically |
| it can't find the brain | open `tankbot.local/brain`; on the brain phone check Wi-Fi and Settings > Privacy > Local Network |

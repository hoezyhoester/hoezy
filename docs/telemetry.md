# Motion telemetry

Xerenge can send where the player is in a race, sixty times a second, as UDP
packets for a motion simulator. The packets go to SpaceMonkey
(<https://github.com/PHARTGAMES/SpaceMonkey>), whose **Burnout Revenge**
provider turns them into accelerations, rates and angles, filters them, and
passes them on in Codemasters' custom UDP format (Dirt 4 / Dirt Rally 2.0) -
which SimTools, SimFeedback, Sim Racing Studio, FlyPT Mover and SimHub read.

## Turning it on

In `xerenge.conf`:

```
telemetry = on
telemetry_host = 127.0.0.1   # the PC SpaceMonkey runs on
telemetry_port = 20790
```

or start the game with `XERENGE_TELEMETRY=127.0.0.1:20790`. SpaceMonkey only
runs on Windows; the game can run on Linux, a Mac or Android and send to a
Windows PC on the same network - put that PC's address in `telemetry_host`.

In SpaceMonkey: **Burnout Revenge** > **Initialize!**, with the same port.
Then choose the output (Outputs) your motion software listens to, as for any
other SpaceMonkey game.

| Key | Default | What it does |
|---|---|---|
| `telemetry` | `off` | `on` to send |
| `telemetry_host` | `127.0.0.1` | IPv4 address to send to |
| `telemetry_port` | `20790` | UDP port |
| `telemetry_mirror` | `true` | RenderWare's world is right-handed, SpaceMonkey's left-handed. If the rig leans *into* corners instead of out of them, set `false`. |
| `telemetry_scale` | `1.0` | world units to metres |
| `telemetry_roll` | `1.0` | how much of the camera's sideways lean becomes roll (0 to 2) |

## Where the data comes from

The player car's physics object has not been found in this build yet, so the
motion is taken from the race camera, which Burnout's chase and bumper
cameras carry along behind the car (`rexglue/src/telemetry.cpp`):

- **position** - the camera's; surge, sway and heave come from its path;
- **heading** - where the camera looks, flattened;
- **pitch** - the road's slope along the direction of travel;
- **roll** - the camera's own lean.

So the motion is the car's as the camera follows it: a little smoothed and a
fraction late. Crash and takedown cameras are cuts; SpaceMonkey's teleport
handling eases the rig through them. Outside a race (menus) the game sends
"not in a race" and the rig settles to rest.

`burnout.log` says what was found, in lines starting `telemetry:` - the race
camera, which axis is up, or why a camera was not used.

Once the car's own object is known, a hook can fill the same sample from it
(true accelerations, throttle, brake, steering, rpm and gear have places in
the packet already) and nothing on SpaceMonkey's side changes.

## The packet

Little-endian, 128 bytes:

| Offset | Type | |
|---|---|---|
| 0 | char[4] | `XRTM` |
| 4 | u16 | version, 1 |
| 6 | u16 | flags: 1 in a race, 2 inputs known |
| 8 | u32 | sequence |
| 16 | f64 | time, seconds |
| 24 | f64[3] | position (metres, y up) |
| 48 | f64[3] | forward (unit) |
| 72 | f64[3] | up (unit) |
| 96 | f32 | speed, m/s |
| 100 | f32[5] | throttle, brake, steering, rpm, gear; -1 when not known |

# 2026: Rebuilt

1757 Rebuilt Robot

[Chief delphi form](https://www.chiefdelphi.com/t/frc-1757-wolverines-2022-2023-build-thread/416564)

[The Blue Alliance](https://www.thebluealliance.com/team/1757)

[Website](https://whsrobotics.org)

## Installation (for the average programmer)

### Visual Studio 2019 redistributable

[vc_redist.x64](https://aka.ms/vs/16/release/vc_redist.x64.exe)

### Python

[3.10+ amd64](https://www.python.org/downloads/release/python-31013/)

### VS Code

[VS Code](https://code.visualstudio.com)


### Install robotpy

1. **To save yourself from pain, run the following in your bash terminal:**
    ```bash
    py -3 --version #or python3 --version
    ```
    MAKE SURE IT IS **3.10** OR ABOVE (if not, use the link above to get it)

1. **Globally install uv (for the virual environment)**
[uv installation guide](https://docs.astral.sh/uv/getting-started/installation/)

```sh
uv sync
```
should deal with all the installation details

## Running

To simulate the robot, use the following:

```sh
uv run -- robotpy --main src sim
```

To run back a log file in replay mode, set the `LOG_PATH` environment variable and then run in simulation

An example is to run the following:

```sh
LOG_PATH=/path/to/log/file.wpilog uv run -- robotpy --main src sim
```

For replay watch, do the following:

```sh
LOG_PATH=/path/to/log/file.wpilog uv run -- robotpy --main src watch
```

## On real hardware

On real hardware, to deploy onto a robot use the following

```sh
uv run -- robotpy --main src deploy
```


### Steps to take when commiting

1. **run black from uv**

```bash
uv run -- black .
```

2. **check pylint from uv**

pylint is used for linting, or checking that code will work as expected

```bash
uv run -- pylint $(git ls-files '*.py')
```
3. **check mypy from uv**

mypy is used for type checking, or making sure that types are used correctly

```bash
uv run -- mypy $(git ls-files '*.py')
```

4.  **Make sure it starts in sim and works as expected**

- Should be obvious but make sure to do it

1. **CODE FORMATTING PRACTICES**
   - for python we are using [black](https://github.com/psf/black)
   - for json we are using [prettier](https://prettier.io)
   - all JSON files must be alphabetized, you can use the [following extension for prettier](https://www.npmjs.com/package/prettier-plugin-sort-json)
   

### Advanced User Stuff (With robot electronics)
### FRC Game Tools

[FRC Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#479842)

### CTRE Phoenix

Download Phoenix Tuner X from the Microsoft Store

## Setup

### roboRIO

1. Image the roboRIO
   [Imaging instructions](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/roborio2-imaging.html)
1. Configure the roboRio
   | Item | Value |
   | - | - |
   | Team number | `1757` |
   | Firmware | `6.0.0f1` |
   | Static IP | `10.17.57.2` |
   | Subnet Mask | `255.255.255.0` |

### Run Phoenix Tuner

#### Update device firmware

- PDH
- FalconFX
- CANCoder
- Pneumatics Hub

#### Configure CAN devices

| Device              | Class   | Range   | ID             |
| ------------------- | ------- | ------- | -------------- |
| robo_rio            | core    | 0 - 9   | master (no ID) |
| pdh                 | core    | 0 - 9   | 0              |
| front_left_drive    | motors  | 10 - 29 | 10             |
| front_left_steer    | motors  | 10 - 29 | 11             |
| front_right_drive   | motors  | 10 - 29 | 12             |
| front_right_steer   | motors  | 10 - 29 | 13             |
| back_left_drive     | motors  | 10 - 29 | 14             |
| back_left_steer     | motors  | 10 - 29 | 15             |
| back_right_drive    | motors  | 10 - 29 | 16             |
| back_right_steer    | motors  | 10 - 29 | 17             |
| front_left_encoder  | sensors | 40 - 59 | 40             |
| front_right_encoder | sensors | 40 - 59 | 41             |
| back_left_encoder   | sensors | 40 - 59 | 42             |
| back_right_encoder  | sensors | 40 - 59 | 43             |

#### Configure network devices

| Device                  | IP Address   | Subnet Mask       |
| ----------------------- | ------------ | ----------------- |
| VH-109 radio            | `10.17.57.1` | `???.???.???.???` |
| roboRIO                 | `10.17.57.2` | `255.255.255.000` |
| Driver Station (laptop) | `10.17.57.5` | `255.000.000.000` |
1. **Download python for roboRIO**

This is done automatically by robotpy when you deploy the code
If you do not have the packages downloaded locally for installation, it will prompt you how to download them. For a fresh installation, expect to need to download the packages.

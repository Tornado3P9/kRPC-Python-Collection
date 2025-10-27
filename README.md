# kRPC-Python-Collection

[https://krpc.github.io/krpc/index.html](https://krpc.github.io/krpc/index.html)

This repository contains simple python scripts for launching and controlling a rocket in the [KSP](https://store.steampowered.com/app/220200/Kerbal_Space_Program/) space simulator.

The scripts completely ignore the existence of SolidFuel Boosters but that's intentionally in order to account for different rocket types. Just tab the space key once your SRBs are empty or modify the script yourself.
The launch script solely checks for the existence of active thrust, without considering the source of that thrust. If there is no thrust being generated, the script will automatically initiate staging regardless.

## Usage:

***First download the KSP-Addon from Github and copy it to the KSP Game Folder:***

[https://github.com/krpc/krpc/releases/download/v0.5.4/krpc-0.5.4.zip](https://github.com/krpc/krpc/releases/download/v0.5.4/krpc-0.5.4.zip)

```bash
wget https://github.com/krpc/krpc/releases/download/v0.5.4/krpc-0.5.4.zip
```

Unzip it and copy the content from the "`GameData`" folder, which should be the `ModuleManager.4.2.2.dll` file and the `kRPC` directory, to the KSP Game Folder. If you installed KSP over Steam on Linux, the game folder is at:
`~/.steam/steam/steamapps/common/Kerbal Space Program/GameData/`.

Run KSP and verify that the Addon was installed correctly and can be activated. Then...

***Clone the repository (or download the scripts however you want):***
```bash
git clone https://github.com/Tornado3P9/kRPC-Python-Collection.git
```

***Install the necessary python libraries (also have python installed)***

***using `pip`***:

```bash
python3 -m venv ./.venv
source ./.venv/bin/activate
pip install -r requirements.txt
```

***or using `uv`*** (https://docs.astral.sh/uv/getting-started/installation/): 

```bash
uv venv
uv sync
```

***Run the launch script***

***using `pip`***:

```bash
deactivate &> /dev/null; source ./.venv/bin/activate
python launch.py --help
```

***or using `uv`***

```bash
uv run launch.py --help
uv run launch.py --target 90000
```

# kRPC-Python-Collection

You can find the official documentation for the Addon at [https://krpc.github.io/krpc/index.html](https://krpc.github.io/krpc/index.html).

This repository features straightforward Python scripts designed for launching and managing a rocket within the [Kerbal Space Program (KSP)](https://store.steampowered.com/app/220200/Kerbal_Space_Program/) space simulator. These are hobby scripts designed for a computer game, not a professional program requiring flawless documentation. Use them only if you're already familiar with KSP.

Note: The scripts completely ignore the existence of SolidFuel Boosters but that's intentionally in order to account for different rocket types. Just tab the space key once your SRBs are empty or modify the script yourself.
The launch script solely checks for the existence of active thrust, without considering the source of that thrust. If there is no thrust being generated, the script will automatically initiate staging regardless.

## *Usage:*

**1. Download the KSP-Addon from Github ([https://github.com/krpc/krpc/releases/download/v0.5.4/krpc-0.5.4.zip](https://github.com/krpc/krpc/releases/download/v0.5.4/krpc-0.5.4.zip)) and copy it to the KSP Game Folder:**

```bash
wget https://github.com/krpc/krpc/releases/download/v0.5.4/krpc-0.5.4.zip
```

Unzip it and copy the content from the "`GameData`" folder, which should be the "`ModuleManager.4.2.2.dll`" file and the "`kRPC`" directory, to the KSP Game Folder. If you installed KSP over Steam on Linux, the game folder is at:
`~/.steam/steam/steamapps/common/Kerbal Space Program/GameData/`, which can also be accessed via "`~/.local/share/Steam/steamapps/common/Kerbal Space Program/GameData`".

Start KSP and verify that the new Addon was installed correctly and works as intended. Then...

**2. Get the scripts by cloning this repository to your pc (or download the scripts however you want):**
```bash
git clone https://github.com/Tornado3P9/kRPC-Python-Collection.git
```

**3. Ensure Python is installed on your system, then use `pip` to install the required Python libraries for the scripts:**

```bash
python3 -m venv ./.venv
source ./.venv/bin/activate
pip install -r requirements.txt
```

**or use `uv` instead of pip (https://docs.astral.sh/uv/getting-started/installation/):**

```bash
uv venv
uv sync
```

**4. When KSP is running, the Addon is activated, and a rocket is positioned on the launch pad, initiate the launch script using `pip`:**

```bash
deactivate &> /dev/null; source ./.venv/bin/activate
python launch.py --help
python launch.py --target 90000
```

**or using `uv`**

```bash
uv run launch.py --help
uv run launch.py --target 90000
```

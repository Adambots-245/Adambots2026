# AdvantageScope Robot Assets

This folder contains 3D robot models for use with AdvantageScope 3D Field visualization.

## Available Models

### KitBot (Simple)
- **File:** `robots/KitBot.glb` (105 KB)
- **Config:** `robots/KitBot_config.json`
- **Description:** Generic FRC KitBot. Small and fast to load.
- **Source:** AdvantageScope bundled assets

### Presto (Flywheel Shooter)
- **Files:** `robots/Presto/model.glb` (31 MB) + `robots/Presto/model_0.glb` (23 MB)
- **Config:** `robots/Presto/config.json`
- **Description:** Team 6328's 2024 Crescendo robot with a flywheel shooter mechanism. Includes an articulated shooter/arm component.
- **Source:** [AdvantageScopeAssets](https://github.com/Mechanical-Advantage/AdvantageScopeAssets/releases)

## How to Use in AdvantageScope

1. Open AdvantageScope
2. Go to **Help > Show Assets Folder**
3. Copy the desired robot folder into `robots/` inside the assets folder
4. In a 3D Field tab, select the robot model from the dropdown

For Presto, copy the entire `Presto/` folder. For KitBot, create a folder `KitBot/` and put `KitBot.glb` as `model.glb` and `KitBot_config.json` as `config.json`.

## Getting More Models

See [AdvantageScopeAssets releases](https://github.com/Mechanical-Advantage/AdvantageScopeAssets/releases) for more robot models from various teams.

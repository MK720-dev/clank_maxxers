Maze Controller README

This controller is set up to do two different things depending on whether a valid maze_dump.json file already exists.

How it works:
1. If maze_dump.json does not exist, or FORCE_REMAP is set to True, the robot will run in mapping mode.
2. In mapping mode, the robot explores the maze, records the map, detects the green end wall, returns to the red start wall, and saves the maze data into maze_dump.json.
3. If maze_dump.json already exists and FORCE_REMAP is set to False, the robot will skip mapping and instead load the saved map and drive the shortest path to the goal.

How to run:
1. Put this controller file in the same controller folder as maze_dump.json.
2. Make sure the robot is assigned to this controller in Webots.
3. For a brand new maze or changed maze layout, set FORCE_REMAP = True and run the simulation once.
4. After the map has been saved successfully, set FORCE_REMAP = False and run the simulation again.
5. The robot should now read maze_dump.json and follow the optimal path to the green wall.

Backup option:
If the mapping controller does not work correctly, use the wallfollower controller located in the same folder instead.

Suggested workflow:
1. Try the mapping controller first.
2. If mapping fails, switch the robot controller to wallfollower in the same folder.
3. Run wallfollower as a fallback option.

Files:
- main maze controller file
- maze_dump.json
- wallfollower controller in the same folder

End of README

# Model-Based Reflex Agent Design of Pac-Man Strategies with Support of Pathfinding Algorithms in a Partially Observable Environment

### Abstract
This repository houses the model-based reflex agent design of my Pac-Man strategies in a partially observable environment. Limited visibility dictates that Pac-Man in such an environment can only see foods that are 5 steps ahead of it, and 1 step left or right of it, if there are no walls at these locations. In addition, Pac-Man can only detect ghosts that are 2 steps from it. The objective of this Pac-Man project is to maximize win rate. In order to accommodate such challenges and the objective, I utilizes Breath-First Search algorithm and A* search algorithm to support Pac-Man's pathfinding functionalities, and breaks down the overall strategies into four major functional components (ordered by priority): (1) map-building operation, (2) survival mode, (3) hungry mode, and (4) corner-seeking mode. Map-building operation scans nearby environment and store such information into Pac-Man's internal memory; survival mode detects imminent threats by using A*, and evades accordingly; hungry mode uses Breath-First Search to route the Pac-Man towards the nearest food; corner-seeking mode guides Pac-Man through unexplored areas in the maze by navigating towards different corners. With such strategies, after running a significant number of tests, Pac-Man's win rate stabilizes at around 90\%, with the highest win rate at 94\%.

### How to run
This version of Intelligent Pac-Man is running on <b>Python 2.7</b> environment. The visibility of the agent in this Pac-Man project is extremely limited, just like someone is running through the maze for real.<br/>
Run the following command to play the game:<br/>
```
python pacman.py --pacman PartialAgent --layout mediumClassic --numGames 50
```

### Design Document
<a href="https://github.com/xujiachang1024/Partial-Pac-Man/blob/master/cw1-xu-jiachang.pdf">*Model-Based Reflex Agent Design of Pac-Man Strategies with Support of Pathfinding Algorithms in a Partially Observable Environment*</a>

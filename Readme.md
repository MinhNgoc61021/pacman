# [Pacman](https://inst.eecs.berkeley.edu/~cs188/sp20/project1/)

## Introduction
- In this project, your Pacman agent will find paths through his maze world, both to reach a particular location and to collect food efficiently. You will build general search algorithms and apply them to Pacman scenarios.

### Files you'll edit:

- search.py	- **Where all of your search algorithms will reside.**
- searchAgents.py - **Where all of your search-based agents will reside.**
***Side Point**: In searchAgents.py, you’ll find a fully implemented SearchAgent, which plans out a path through Pacman’s world and then executes that path step-by-step. The search algorithms for formulating a plan are not implemented – that’s your job.

### Files you might want to look at:
- pacman.py	- **The main file that runs Pacman games. This file describes a Pacman GameState type, which you use in this project.**
- game.py - **The logic behind how the Pacman world works. This file describes several supporting types like AgentState, Agent, Direction, and Grid.**
- util.py - **Useful data structures for implementing search algorithms.**

### Supporting files you can ignore:
- graphicsDisplay.py - **Graphics for Pacman**
- graphicsUtils.py	- **Support for Pacman graphics**
- textDisplay.py - **ASCII graphics for Pacman**
- ghostAgents.py - **Agents to control ghosts**
- keyboardAgents.py	- **Keyboard interfaces to control Pacman**
- layout.py	- **Code for reading layout files and storing their contents**
- autograder.py	- **Project autograder**
- testParser.py	- **Parses autograder test and solution files**
- testClasses.py - **General autograding test classes**
- test_cases/ - **Directory containing the test cases for each question**
- searchTestClasses.py - **Project 1 specific autograding test classes**

### Grading
```
python autograder.py
```

### DFS
Tiny Maze
```
python pacman.py -l tinyMaze -p SearchAgent
```
Medium Maze
```
python pacman.py -l mediumMaze -p SearchAgent
```
Big Maze
```
python pacman.py -l bigMaze -z .5 -p SearchAgent
```

### BFS
Tiny Maze
```
python pacman.py -l tinyMaze -p SearchAgent -a fn=bfs
```
Medium Maze
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
```
Big Maze
```
python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5
```

### UCS
Medium Maze
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
```
Medium Dotted Maze
```
python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
```
Medium Scary Maze
```
python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
```

### A*S
```
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
```

### Corners Problem: Heuristic
Implement a non-trivial, consistent heuristic for the CornersProblem in cornersHeuristic
```
python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5
```
AStarCornersAgent is a shortcut
```
-p SearchAgent -a fn=aStarSearch,prob=CornersProblem,heuristic=cornersHeuristic
```

### Eating All The Dots
```
python pacman.py -l testSearch -p AStarFoodSearchAgent
```
```
-p SearchAgent -a fn=astar,prob=FoodSearchProblem,heuristic=foodHeuristic
```
```
python pacman.py -l trickySearch -p AStarFoodSearchAgent
```

### Suboptimal Search
```
python pacman.py -l bigSearch -p ClosestDotSearchAgent -z .5
```
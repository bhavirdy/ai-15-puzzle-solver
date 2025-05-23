# AI 15-Puzzle Solver

This project implements an AI-based solver for the 15-puzzle game using **Breadth-First Search (BFS)** and **A Star Search** algorithms.

## Introduction
The 15-puzzle is a sliding puzzle consisting of a 4x4 grid with 15 tiles labeled `A` to `O` and one blank space (`#`). The goal is to rearrange the tiles to match the target configuration:
A B C D E F G H I J K L M N O #

## How It Works
1. **State Representation:** The puzzle state is represented as a string (e.g., `ABCDEFGHIJKLMNO#`).
2. **Actions:** Possible moves are `UP`, `DOWN`, `LEFT`, and `RIGHT`.
3. **Heuristic:** The Manhattan distance is used to estimate the cost to reach the goal state.
4. **Search Algorithms:**
   - **BFS:** Explores all states in a breadth-first manner.
   - **A Star Search:** Combines the path cost and heuristic cost to prioritise states.

## Usage
1. Create an input file named `puzzles.txt` with each line representing a puzzle state.
2. Compile the program:
   g++ -o puzzle_solver puzzle_solver.cpp
3. Run the program:
    ./puzzle_solver

## Input Format
The input file puzzles.txt should contain one puzzle state per line. Each state is a 16-character string representing the tiles in row-major order. 

## Output
For each puzzle, the program outputs:
* Solution Cost: The number of moves in the solution.
* Nodes Generated: The total number of nodes generated during the search.
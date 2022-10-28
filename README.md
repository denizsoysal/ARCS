# ARCS
Repo for the ARCS course @KUL

# Problem Description
- description of the whiteboard erasing problem
- link to the industrial application of concrete scraping

# Minimum Viable Product Description

1. Erase a perfectly oriented whiteboard with bimanual coordination
  - classify a board as dirty
  - identify the dirty areas of the board
  - use orientation of dirty areas to decide between left or right dominant asymmetric motions
  - grasp the board on the top edge with one hand
  - move the over arm over the surface of the board to erase
  - confirm that board has been cleaned

## Possible Future Work

2. Start with a skewed board and orient in space
3. Introduce more functionality into erasing specific areas or characters on the board

# High Level Task State Machine

# World Model for the MVP

- what is in our world?

## Objects

1. Whiteboard
2. Markings on the whiteboard
3. Each Arm and End Effector
4. Humans

## Object Features

### Whiteboard
- surface which contains markings
- edge to grasp

## Sensor Features and Representation

### Whiteboard
- edges of whiteboard
- color of the whiteboard

# Lower Level Activity State Machines

- outline the "state collections" of the lower level activities

## Necessary Flags For Each Activity

# Petri Nets

# Physical Setup
- positions of arms
- sensor selection and orientation in space

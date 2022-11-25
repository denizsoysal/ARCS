# Sprint goals 

## 24 Nobember 2022

### Perception 

- Camera interface done
    - receive and save images from both cameras
    - try some openCV algorithm 

### Control

- Write the task state machine 
- Move control arm and stop it when arm contacts the whiteboard 

## 1 December 2022

### Perception 

- Detect the whiteboard
    - erosion + dilation (closing)
    - edge detection
    - color histogram 
    - other methods if needed 
    - background removal 
    - Thing of datastructure to store the whiteboard
        - store the left bottom as the reference of the frame
        - then store all three points 
            - [(3,5),(5,8)] 
- track the end effector
    - https://pyimagesearch.com/2015/09/21/opencv-track-object-movement/
    - or look at things that has moved between images 
- find boudings boxs for the words 


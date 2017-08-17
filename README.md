# phase-space-locomotion-planner

A hybrid phase space planner for rough terrain locomotion which includes the following key components: (i) a step transition solver that enables dynamically tracking non-periodic keyframe states over various types of terrains, (ii) a robust hybrid automaton to effectively formulate planning and control algorithms, (iii) a steering direction model to control the robot’s heading, (iv) a phase-space metric to measure distance to the planned locomotion manifolds, and (v) a hybrid control method based on the previous distance metric to produce robust dynamic locomotion under external disturbances。

This is a MATLAB algorithm implementation of the demos in the following paper:

Ye Zhao, Benito Fernandez and Luis Sentis. "[Robust Optimal Planning and Control of Non-Periodic Bipedal Walking with A Centroidal Momentum Model](https://dl.dropboxusercontent.com/u/54795992/IJRR-ROPSPlanning.pdf)," The International Journal of Robotics Research. Accepted. 2017.

A demo video is linked [here](https://youtu.be/eSqQS4z7EYA).

Please give a try on the demo scripts and feel free to open issues or contact us (yezhao@utexas.edu, lsentis@austin.utexas.edu).

License: "[https://githubusercontent.com/YeZhao/phase-space-planner-locomotion/master/LICENSE.TXT](https://raw.githubusercontent.com/YeZhao/phase-space-planner-locomotion/master/LICENSE?token=ACBwRzdFZ5yt134RxR6Nn1TJOMxmeGh0ks5Zn2nHwA%3D%3D)"
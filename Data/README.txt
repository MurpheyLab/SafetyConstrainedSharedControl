The Data folder contains all of the data presented in the results section of the corresponding paper.

Each PD controller has its own folder:
C1:  Kp = 10, Kd = 1
C2:  Kp = 12, Kd = 1
C3:  Kp = 14, Kd = 1
C4:  Kp = 16, Kd = 1
C5:  Kp = 18, Kd = 1
C6:  Kp = 20, Kd = 1
C7:  Kp = 10, Kd = 2
C8:  Kp = 12, Kd = 2
C9:  Kp = 14, Kd = 2
C10: Kp = 16, Kd = 2
C11: Kp = 18, Kd = 2
C12: Kp = 20, Kd = 2
C13: Kp = 10, Kd = 3
C14: Kp = 12, Kd = 3
C15: Kp = 14, Kd = 3
C16: Kp = 16, Kd = 3
C17: Kp = 18, Kd = 3
C18: Kp = 20, Kd = 3

Each folder contains 200 trial files with columns:
time (s), theta_x, thetadot_x, theta_y, thetadot_y, bowl_x, bowl_xdot, bowl_y, bowl_ydot, u_x, u_y, desired u_x, desired u_y, ball height, goal_x, goal_y, Kp, Kd

Each folder also contains a file of interventions in addition to a file of counterfactual trajectories.

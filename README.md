# q_learning_project
Team members: Riya Sahni and Julia Folkl
## Q-Learning Algorithm
### Executing the Q-learning algorithm:
* First, we will find our current state in the given Q-matrix and look up all the valid actions we can take from our current state (whose actions != -1). Then, we will calculate the Q-value from each of the valid actions we can take and pick the action with the highest Q to take.
* To test this component, we will introduce a situation with multiple possible actions to take, where there is an obvious optimal action we have already calculated from the origin (which we will make our starting path). Then we will use our algorithm to confirm that the result is the obvious optimal action given the possible actions to take.
### Determining the Q-matrix has converged:
* If the Q-value changes by some very small value for each of the valid actions that we take, i.e. if the Q-values all change by something < 0.005 or some designated value, then we will consider the Q-matrix to have converged.
* To test this component, we can compute and keep track of the change in Q-value for each valid action. We can confirm that the Q-value has converged when these delta-Q-values are all less than some defined value.
### Once Q-matrix has converged, which actions should the robot take to maximize expected reward?
* In each state, the turtlebot has a finite set of actions it can take. It will take the action with the highest Q-value following convergence.
* We can test this component by computing the Q-values for each of the valid actions, checking that it chooses the action with the highest computed Q-value, and also confirming that the Q-value associates with the most optimal action by creating a situation in which the optimal action is extremely obvious out of the set of all valid actions.
## Robot perception
### Determining identities and locations of the three colored objects
* The robot's camera can recognize the colors/identities of the objects if we define the colors using an RGB spectrum, and it can determine the locations of the colored objects by using the robot's sensor and doing math to find the angle and distance the object is from the robot. Then, the robot can travel at a certain speed for some amount of time until it is 'x' distance away from the object to be able to pick it up. We can pay attention to other objects nearby and lighting that might add noise to the robot's camera's ability to detect specific colors of the objects.
* We can test this component by placing an object some distance and angle away from the robot and printing out what color the robot recognizes the object as, as well as whether the robot is able to determine the relative location of the object and travel towards it.
### Determining the identities and locations of the three AR tags
* We can use the robot's camera to scan each tag and use the robot's sensor to measure the distance to each tag. Thus, we can determine the identities of the tags through the robot's camera, and we can determine the locations for each tag through the robot's sensor, finding the distance to the tag relative to the robot's position.
* To test this component, we can do something similar to how we test for the previous component: we can place the robot at some distance and angle away from one of the tags and have it search for the tag by moving its sensor, printing what tag it recognizes it to be, and them computing its relative location and we can see if it successfully navigates towards it or not.
## Robot manipulation and movement
### Picking up and putting down the colored objects with the OpenMANIPULATOR arm
* We can use the turtlebot3_manipulation_bringup node and the run move_group node to control the robot's arm movement for grabbing and releasing objects. We can experiment with parameters to find the proper "gripping" strength for the arm.
* To test this component, we can put an object within the robot's "grabbing distance" range and just have it grab and pick up an object that resembles the colored object and see if it is successfully able to complete the task. We can adjust parameters accordingly.
### Navigating to the appropriate locations to pick up and put down the colored objects
* We can use the robot's sensor to measure the distance that the robot is away from the colored object and the angle at which its arm approaches the object. Once lab F is published, we can study the GUIs used in it to find the approprioate joined angles for picking the object up and putting it down.
* We can test this component by simply running multiple scenarious where the robot has to navigate to a colored object and stop at an appropriate distance and angle from it to be able to pick it up. We can combine our test for this component with our test for the previously described component so that the robot does multiple trials of having to navigate to a colored object, stop at an appropriate angle and distance from the object, and successfully grab and grip onto the object to lift it up and travel with it.
## Brief timeline:
* We'll want to have the converged Q-matrix intermediate deliverable completed by Thursday (11/03) evening latest, with the best case scenario being completed by Wednesday (11/02)
* Next, we will complete the robot's functionality to navigate to a colored object that it locates by storing the colored object's location and having the robot stop and orient itself in an approriate way for it to be able to pick up the object. This should hopefully be completed by Saturday (11/05) or Sunday (11/06) at the latest.
* Next, we will complete the robot's functionality to grip and pick up the colored object and be able to travel with it successfully. This should hopefully be completed by Monday (11/07) or Tuesday at the latest (11/08).
* Finally, we should have everything integrated and working together with bugs worked out and parameters adjusted by Thursday evening hopefully, or possibly using our late hours, by Friday (11/11).

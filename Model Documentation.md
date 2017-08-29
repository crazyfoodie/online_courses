
### Half of the code is based on the content of walkthrough.
### To be honest, before I watch the walkthrough, I am not pretty sure how to make it work,there are too much things I need to consider.
### After watch the vedio everything become clear. Below is how my code works:

### With the help of walkthrough, my car can move smoothly without causing the jerk, this is mainlygained by using spline library which give a smooth path to go.
### Only issue not solved in the walkthrough is the how the car change the lane if the car ahead is too slow.
### My algorithm is write in the function "changeToLane", in that function I first check if it is safe to change the line and if it is safe then I will choose one of the lane to go (this part can be bettter if I write more logic on which line is better to go). 

### As you can see in the simulation it works pretty good, except one case, it will not response perfectly, the case is if the car is in the line 1 and there is no car in lane 2 detected by fusion data, while lane 0 have one car far away ahead of my car.
### in this case people will decide to go to line 2 while in my code it will go to lane 0,  most time it will be ok and later it will go back to lane 1 if the car in the lane 0 is too slow, however in rare case, the car in the lane 0 will suddenly decrease the velocity and car in lane 1 and lane0 drive in the same speed , in the case my car will wait for a long time until there is a chance to change line.

### By the way if it is not safe to change the line, my car will slow down to a little bit slower than the car aheadand wait until there is a good/safe time to change the lane.

## Let me know if you have any intersting idea on how to improve my current logic, I am looking forward to your response.


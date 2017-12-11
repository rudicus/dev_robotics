Partial code dump from my developmental robotics final project at Tufts.

Please note this code is *not* runnable as is - it requires the entire software stack for the menufacturing version of the Sawyer Robot to run.

components -> Contains code that runs on the robot and does the affordance mapping

image_listener -> Script that listens for the camera images over ros, runs them through the tensor flow classifer and publishes any detections out over ros

training_examples -> A few example images and labels from the training data I used

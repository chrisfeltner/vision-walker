# Smart-Walker Computer Vision
Related: <https://github.com/sharare90/Smart-Walker>

## Installation
We are using Virtual Enviroments to manage our dependencies.
To install the Virtual Enviroment package, use:
`pip install virtualenv`

You can set up your Virtual Enviroment with the correct version of Python using

`virtualenv python .`

Where python is the command used to access Python 2.7 on your machine, and . is the directory that you would like your virtual environment to be placed (leaving it as a dot places it in the root of your project).

Next, you'll want to install all required Python packages using
`pip install -r requirements.txt`

## Running
Ensure that you are connected to your virtual environment using:

`source environment_folder/bin/activate`

In the project's current state, we have 2 seperate Python files. `kinect.py` is used to collect the depth image from a Kinect camera. This project on GitHub does not support this, however, and is mainly here for reference.

The main file for this project is `test_detect.py`. To run this, use the command `python test_detect.py imageoutput.txt`. The program will output whether it detect an obstacle or not.

--------------------------------------

This branch has additional functions for running through test files, along with an experimental `detect2.py` file.

`detect2.py` requires a `answers.txt` file. The first line of this file is the number of test cases to run though. Each subsequent line will have a boolean True or False corresponding to whether or not the associated depth image contains an object, and an integer corresponding to the actual measured distance of the object, if there is one.

Each line in `answers.txt` requires a depth image saved as either a 1-d or 2-d array in a text file with the name `(test#).txt`.

`kinect2.py` requires libfreenect and its Python bindings to be installed. The files and instructions for libfreenect can be found here: https://github.com/OpenKinect/libfreenect


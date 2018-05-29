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


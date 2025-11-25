# Capstone Project

This repository creates all the code, neural networks, and output data created in the process of my senior capstone project.

## Contents

### adjusted_odometry_publisher

Defines the correct node to publish an offset odometry using a pre-trained neural network with sensor values obtained at the time of each collision.

### collision_data

Contains all of the data collected to train the neural network along with each version of the processed data.

### data_collector

Defines the collect node to faciliate the data collection process.

### data_processing

Contains the code used to split, normalize/standardize, and perform PCA on the data.

### networks

Contains the neural networks trained in the project (temp stores networks during parameter search).

### neural_network

Contains code to train neural networks and perform parameter searches.

### output_data

Contains the outputs from feeding testing data to trained neural networks.

### robot_driver

Defines drive node to control robot's movement and publish inputs.
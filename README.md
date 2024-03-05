# people_facts

## Overview

This node listens to various ROS4HRI topics, and stores semantic information
about humans into a knowledge base.

## ROS 2 API

### Topics

This package follows the ROS4HRI conventions ([REP-155](https://www.ros.org/reps/rep-0155.html)).
If the topic message type is not indicated, the ROS4HRI convention is implied.

#### Subscribed

- `/humans/persons/tracked`
- `/humans/persons/known`
- `/humans/persons/<person_id>/engagement_status`

### Services

The node relies on the `/kb/revise` service to be available to store facts in
the knowledge base.

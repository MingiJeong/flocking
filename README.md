

## Requirement
Gazebo 7

## include model
When launching the ros node, it might trigger an errror for loading a model.
In such a case, do the following command on the terminal. The cause of the error is from the model which the system is not recognizing.
The reference is based on the following link: https://answers.gazebosim.org//question/1940/problem-with-including-a-model/

```
export GAZEBO_PLUGIN_PATH=~/<path>/my_package_example/lib:${GAZEBO_PLUGIN_PATH}

export GAZEBO_MODEL_PATH=~/<path>/my_package_example/models:${GAZEBO_MODEL_PATH}

export GAZEBO_RESOURCE_PATH=~/<path>/my_package_example/models:${GAZEBO_RESOURCE_PATH}
``` 

avoid deadlock include random
<<<<<<< HEAD

### Instruction for Face_recognition package 

Tested in ubuntu 16.04

## 1. install dlib 
  
  ````
$ cd ~/
$ git clone https://github.com/davisking/dlib.git
$ cd dlib
$ mkdir build; cd build; cmake .. -DDLIB_USE_CUDA=0; cmake --build .
$ cd ..
$ sudo python setup.py install --no USE_AVX_INSTRUCTIONS --no DLIB_USE_CUDA
````
  

## 2. install pip (if you use python 3.3 > pip3)
  Cammand for installing pip 
  
  ```
  $ sudo apt-get install python-pip
  ```
    
  Check pip version
 
  ```
$ pip -V
  ```

## 3. face_recognition package
  
  ````
$ sudo pip install --upgrade scipy
$ sudo pip install face_recognition
$ sudo pip install git+https://github.com/ageitgey/face_recognition_models
  ````
  
## 4. install tensor_flow (pip install)
  
````
$ pip install tensorflow
````

## 5. Run examples.

````
$ python test.py known_faces/ new_faces/
````

## 6. Run with ROS
````
$ rosrun face_recognition_ros test_ros
````

## 7. ROS service and action are available
- face_service.py
- face_action_server.py
- take_face_photo_server.py
- take_photo_server.py

# Author
Minkyu Kim (steveminq@utexas.edu)

## Reference
  
 https://github.com/hcr-hsrproject/villa_perception/tree/master/face_recognition
  
=======
# deepgaze_ros
>>>>>>> 7f264fd3dce19e71dfed0fe4a1a8e0c648b7176f

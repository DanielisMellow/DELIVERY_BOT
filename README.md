# DELIVERY_BOT
Delivery Bot is a package delivering robot that is built on the Romi chassis. 

## Overview 
The project's core is a line-following robot with object detection used to make specific deliveries on a track. The Romi 32U4 handles the low-level operations of the robot and communicates with an ESP32 over UART. The ESP32 is used to host a web server using web sockets. The webserver serves as the GUI and displays the status of the robot in real-time. 

## Functional Decompostion: 
![DeliveryBot](https://user-images.githubusercontent.com/82124061/168692394-dbfa2c40-cbe9-4166-8a21-4b2a0275124c.png)

## Delivery Track:
![TRACKV1](https://user-images.githubusercontent.com/82124061/168695865-caa970cd-0dab-46ed-b836-f8ece6bb3135.png)
The robot was designed around this track. The robot is placed in the Starting positioned outline in the track diagram. After calibration, the user will input delivery details, and the robot will pick up a package and take it to a delivery zone. 

## Graphic User Interface:
![IMG_0140](https://user-images.githubusercontent.com/82124061/168697700-8cfed013-5813-4f25-9742-b856d8e01aa8.jpg)
The user interface requires the user to specify the number of deliveries, the drop-off location, which is a lowercase letter defined on the delivery track diagram, and a height, as the drop-off location can be a variety of heights.  

## Delivery Bot:
![IMG_0074](https://user-images.githubusercontent.com/82124061/168695502-3ca4066f-8d42-4fcb-865a-4f27404ebd35.jpg)


# DEMO 
https://user-images.githubusercontent.com/82124061/168700498-6470e1a8-da2c-4bdc-9119-ecf96eca9cb4.mp4

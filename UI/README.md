# RoboPorter UI
So this is where the Roboporter UI files are stored. It is broken down into two different sections:
1. Backend UI
2. Front End Software

## Backend UI
The Backend Software is written in python and is /backend.py This software runs on the second RPi and acts as the link between the UI and the server python code. It is mostly just a piece of software the forwards connections from one program to the other and handles web socket connections.

## Front End 
The front end is a web interface built using the Bootstrap framework. To get it working the following steps must be followed:
1. Install Apache Web Server
2. Install PHP 7 or greater
3. Install mysql 
4. Install PHPMyAdmin for database admin tasks
5. Upload the database stored in /database
6. Change /php/include.php to be the correct database details

## Implemented
* Control Page
* Mapping info and clickable map 

## Work to follow
* Finish mapping UI
* Finish home screen
* Finish settings
* Finish account details
* Secure password storage 
* Add status and debug infomation
<?php
session_start() ; 
include("connect.php");

$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 
$username = $_SESSION["loggedin"] ;
$password = $_POST['password'] ; 
$newpassword = $_POST['newpass'] ;
$newuser = $_POST['newuser'] ;  

$query = "SELECT ID FROM users WHERE Username = '$username' AND Password = '$password'" ;

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));
$row = mysqli_fetch_array($information,MYSQLI_ASSOC);
$numrows = mysqli_num_rows($information);

if(!$numrows == 1){
    echo "password_incorrect" ;
    exit;
}
$query = "SELECT ID FROM users WHERE Username = '$newuser'" ;

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));
$row = mysqli_fetch_array($information,MYSQLI_ASSOC);
$numrows = mysqli_num_rows($information);

if(!$numrows == 0){
   echo "username_in_use" ;
}
else{
    $query = "INSERT INTO users (Username , Password) VALUES('$newuser','$newpassword')" ;
    if(mysqli_query($connection, $query)){
        echo "success" ;
    }else{
        echo "error" ;
    } 
}
?>
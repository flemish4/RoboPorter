<?php
session_start() ; 
include("connect.php");

$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 
$username = $_POST['username'] ; 
$password = $_POST['password'] ; 

if(!$connection){
        die(mysqli_connect_errno());
    }

$query = "SELECT ID FROM users WHERE Username = '$username' AND Password = '$password'" ; 

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

$row = mysqli_fetch_array($information,MYSQLI_ASSOC);

$numrows = mysqli_num_rows($information);

if($numrows == 1){
    echo "loggedin";
    $_SESSION["loggedin"] = $username;
}
else{ echo "failed" ;}

?>
<?php
session_start() ; 
include("connect.php");

$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 
$username = $_SESSION["loggedin"] ;
$password = $_POST['password'] ; 
$newpassword = password_hash($_POST['newpass'], PASSWORD_DEFAULT) ;

$query = "SELECT * FROM users WHERE Username = '$username'" ;

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));
$row = mysqli_fetch_array($information,MYSQLI_NUM);
$numrows = mysqli_num_rows($information);

if(!$numrows == 1 || !password_verify($password,$row[2])){
    echo "password_incorrect" ;
    exit;
}else{
    $query = "UPDATE users SET Password = '$newpassword' WHERE id = '$row[0]'" ;
    if(mysqli_query($connection, $query)){
        echo "success" ;
    }else{
        echo "error" ;
    } 
}
?>
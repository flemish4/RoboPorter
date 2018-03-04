<?php
include("connect.php");

$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 

$image_id = $_POST["name"] ; 

if(!$connection){
        die(mysqli_connect_errno());
    }

$query = "DROP TABLE `".$image_id."`" ; 

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

$query = "DELETE FROM names WHERE Filename = '$image_id'" ; 

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

?>
<?php
include("connect.php");

$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 

$name = $_POST["name"] ; 

$query = "SELECT Filename FROM names WHERE Name = '$name'" ;

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

$onerow = mysqli_fetch_array($information) ;

echo $onerow[0] ; 

?>
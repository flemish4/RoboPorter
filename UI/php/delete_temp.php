<?php

include("connect.php");
$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 

$query = "DELETE FROM temp" ; 
$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

echo "Success" ; 
?>
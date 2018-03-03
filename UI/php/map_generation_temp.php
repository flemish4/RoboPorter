<?php

include("connect.php");

$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 

if(!$connection){
        die(mysqli_connect_errno());
    }

$query = "SELECT * FROM temp" ; 

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

$row = mysqli_fetch_array($information,MYSQLI_ASSOC);

$img = base64_decode($row['data']) ;

header('Content-Type:image/jpeg');

echo $img ; 

?>
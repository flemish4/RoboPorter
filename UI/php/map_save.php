<?php
include("connect.php");
$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 

sleep(2) ; 

$name = $_POST['name'] ; 

$query = "SELECT * FROM names WHERE Name = '$name' " ; 
$information = mysqli_query($connection, $query) or die(mysqli_error($connection));
$numrows = mysqli_num_rows($information);
if ($numrows == 1){
    echo "Error" ; 
    exit;
}else{

    $query = "SELECT max(Filename) FROM names" ; 
    $information = mysqli_query($connection, $query) or die(mysqli_error($connection));
    $rows = mysqli_fetch_array($information) ;
    $filename = intval($rows[0] + 1); 

    $query = "INSERT INTO names (Filename, Name) values ('$filename' , '$name' ) " ; 
    $information = mysqli_query($connection, $query) or die(mysqli_error($connection));

    $query = "CREATE TABLE `$filename` like temp" ; 
    $information = mysqli_query($connection, $query) or die(mysqli_error($connection));

    $query = "INSERT `$filename` SELECT * FROM temp" ; 
    $information = mysqli_query($connection, $query) or die(mysqli_error($connection));

    echo "Success" ; 
}

?>
<?php
include("connect.php");

$row = "" ; 

$connection = mysqli_connect($database_host,$dbuser,$dbpassword,$database) ; 

$query = "SELECT * FROM names" ;

$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

$numrows = mysqli_num_rows($information);

while($onerow = mysqli_fetch_array($information)) {
    $row .= "," ;
    $row .= $onerow["Name"] ;
}
echo trim($row, ",") ;
?>
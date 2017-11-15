<?php
include("connect.php");


$connection = mysqli_connect($database_host,$dbuser,$dbpassword,"map_data") ; 

if(!$connection){
        die(mysqli_connect_errno());
    }

$table = 1 ; 
$query = "SELECT * FROM `".$table."`" ; 
$information = mysqli_query($connection, $query) or die(mysqli_error($connection));

$xarray = Array() ;
$yarray = Array() ;
while($row = mysqli_fetch_array($information,MYSQLI_ASSOC)){
    $xarray[] = $row['X'] ; 
    $yarray[] = $row['Y'] ; 
}


$image = imagecreate(max($xarray),max($yarray)) ;

$background = imagecolorallocate($image,255,255,255) ;
$foreground = imagecolorallocate($image,0,0,0) ;

for($i = 0; $i < count($xarray); $i++){
    imagesetpixel($image,$xarray[$i],$yarray[$i],$foreground) ;

}

header("Content-type:image/jpeg") ;
imagejpeg($image) ;




?>
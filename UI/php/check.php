<?php

session_start() ; 

if(isset($_SESSION['loggedin'])){
echo "loggedin" ; 
}else {echo "failed" ;}


?>
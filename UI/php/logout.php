<?php
session_start() ; 
session_destroy() ; 
header("Location: ../login/?status=1");
exit() ; 

?>
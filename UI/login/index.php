<!doctype html>
<html lang="en">
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
<link rel="stylesheet" href="../resources/css/bootstrap.min.css"  crossorigin="anonymous">
<script type="text/javascript" src="../resources/js/jquery-3.2.1.min.js"></script> 
<script type="text/javascript" src="../resources/js/popper.js"></script> 
<script src="../resources/js/bootstrap.min.js"  crossorigin="anonymous"></script>
<link rel='shortcut icon' type='image/x-icon' href='../favicon.ico' />

<title>RoboPorter Login</title>
<style>
html, body {
  height: 100%;
}
.row{
  min-height: 100%;  /* Fallback for vh unit */
  min-height: 100vh;
}

</style>

<script>
$(document).ready(function(){
  <?php
    if(isset($_GET["status"])){
      echo '$("#loggedout").show();' ;
    }else{
      echo '$("#loggedout").hide();' ;
    };
  ?>

    $('#Modal').modal({
        backdrop: 'static',
        keyboard: false
    })
    $('#failed').hide() ;

    $('#submit').click(function(){
        var data = {
            'username' : $('input[name=username]').val() , 
            'password' : $('input[name=password]').val() ,
        } ;
        
        $.ajax({
            type:'POST',
            url:'../php/login.php',
            data: data ,

        }).done(function(msg){
            if(msg == "loggedin"){
              window.location = "../";
             }else{
              $('#failed').show() ;
             };
            });     
    });
});  
</script>
</head>

<body>

<div class="container-fluid">
  <div class="modal hide fade" tabindex="-1" role="dialog" id="Modal">
    <div class="modal-dialog" role="document">
      <div class="modal-content">
        <div class="modal-header">
          <h5 class="modal-title">Sign In</h5>
        </div>
        <div class="modal-body">
          <form>
            <div class="alert alert-success" role="alert" id="loggedout">
            Logged Out Successfully 
            </div>
            <div class="form-group">
            <label for="username"> Username:</label>
            <input type="text" class="form-control" name="username"></input>
            <label for="password">Password:</label>
            <input type="password" class="form-control" name="password"></input>
            </div>
          </form>
          <div class="alert alert-danger" role="alert" id="failed">
          Log in failed. Try again!
          </div>

        </div>
        <div class="modal-footer">
          <button type="button" class="btn btn-primary" id="submit">Sign In</button>
        </div>
      </div>
    </div>
  </div>
</div>

</body>
</html>

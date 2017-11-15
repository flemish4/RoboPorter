<!doctype html>
<html lang="en">
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
<link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0-beta.2/css/bootstrap.min.css" integrity="sha384-PsH8R72JQ3SOdhVi3uxftmaW6Vc51MKb0q5P2rRUpPvrszuE4W1povHYgTpBfshb" crossorigin="anonymous">
<script type="text/javascript" src="https://ajax.aspnetcdn.com/ajax/jQuery/jquery-3.1.1.min.js"></script> <script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.12.3/umd/popper.min.js" integrity="sha384-vFJXuSJphROIrBnz7yo7oB41mKfc8JzQZiCq4NCceLEaO4IHwicKwpJf9c9IpFgh" crossorigin="anonymous"></script>
<script src="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0-beta.2/js/bootstrap.min.js" integrity="sha384-alpBpkh1PFOepccYVYDB4do5UnbKysX5WZXm3XxPqe5iKTfUKjNkCk9SaVuEZflJ" crossorigin="anonymous"></script>


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

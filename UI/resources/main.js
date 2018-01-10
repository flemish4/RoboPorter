$(document).ready(function(){

    $("#noconnection").hide();
    $("#noconnection-modal").modal("show");
    $("#change_password_warning").hide() ;
    $("#change_password_success").hide() ;

    var timearray = [] ; 
    var s = new WebSocket("ws://192.168.1.2:5555");
    var canvas1 = document.getElementById('MapCanvas1');
    var context1 = canvas1.getContext('2d');
    var canvas2 = document.getElementById('MapCanvas2');
    var context2 = canvas2.getContext('2d');
    var canvas3 = document.getElementById('MapCanvas3');
    var context3 = canvas3.getContext('2d');
    var imageObj = new Image();
    var scale = null ;
    var realx = null ; 
    var realy = null ; 
    var x = null ;
    var y = null ;
    var div_width ;
    var wh_ratio ; 
    var new_height ; 

    var debug_canvas1 = document.getElementById('DebugCanvas1'); // Get Denug Canvas ID
    var debug_context1 = debug_canvas1.getContext('2d') ; 
    var csize = $("#DebugCanvasDiv").width() ;   // Get width of canvas div
    debug_canvas1.width = csize ; // set canvas width to size of div width
    debug_canvas1.height = csize ; // set height to div width i.e. make the canvas square
    var dscale = 0.35 ; // size of roboporter compared to width of div
    debug_context1.fillStyle="#595959" ; //roboporter color
    debug_context1.fillRect(((csize/2)-(dscale*csize/2)),(csize/2)-(dscale*csize/2), (csize*dscale) ,(csize*dscale)); // draw roboporter rectangle
    debug_context1.fillStyle="#ff0000" ; //ultrasonic bar color
    var maxbarlen = (csize - dscale*csize)/2 ; // The maximum debug bar length 
    var dh1 = maxbarlen;// length of ultrasonic bar 1
    var dh2 = 0.4*maxbarlen ;
    var dh3 = maxbarlen ;
    var dh4 = maxbarlen ;
    var dh5 = maxbarlen ;
    var dh6 = maxbarlen ; 
    var dh7 = maxbarlen ;
    var dh8 = maxbarlen ;
    var dh9 = maxbarlen ;
    var dh10 = maxbarlen ;
    var dh11 = maxbarlen ;
    var dh12 = maxbarlen ;
    var dw = dscale*csize/3 ; // ultrasonic bar width

    //fil rectangles for the top
    debug_context1.fillRect((csize/2)*(1-dscale),(csize/2)*(1-dscale)-dh1,dw,dh1) ;
    debug_context1.fillRect((csize/2)*(1-dscale)+dw,(csize/2)*(1-dscale)-dh2,dw,dh2) ;
    debug_context1.fillRect((csize/2)*(1-dscale)+(dw*2),(csize/2)*(1-dscale)-dh3,dw,dh3) ;

    //fill rectangles for the right side
    debug_context1.fillRect((csize/2)+(dscale*csize/2),(csize/2)*(1-dscale),dh4,dw) ;
    debug_context1.fillRect((csize/2)+(dscale*csize/2),(csize/2)*(1-dscale)+dw,dh5,dw) ;
    debug_context1.fillRect((csize/2)+(dscale*csize/2),(csize/2)*(1-dscale)+dw*2,dh6,dw) ;

    //fill rectangles for the base
    debug_context1.fillRect((csize/2)*(1-dscale),(csize/2)*(1-dscale)+(csize*dscale),dw,dh7) ;
    debug_context1.fillRect((csize/2)*(1-dscale)+dw,(csize/2)*(1-dscale)+(csize*dscale),dw,dh8) ;
    debug_context1.fillRect((csize/2)*(1-dscale)+(dw*2),(csize/2)*(1-dscale)+(csize*dscale),dw,dh9) ;

    //fill rectangles for the left side
    debug_context1.fillRect((csize/2)-(dscale*csize/2)-dh10,(csize/2)*(1-dscale),dh10,dw) ;
    debug_context1.fillRect((csize/2)-(dscale*csize/2)-dh11,(csize/2)*(1-dscale)+dw,dh11,dw) ;
    debug_context1.fillRect((csize/2)-(dscale*csize/2)-dh12,(csize/2)*(1-dscale)+dw*2,dh12,dw) ;
    

    setTimeout(() => { // Manages the connection timeout modal. After 21 seconds it fades in an alert saying no connection is available
        $("#connecting").fadeOut(1000, function(){
            $('#noconnection').fadeIn(1000) ;
            $('#pagereloadconn').prop('disabled', false); // Toggles the availability of the button
        });   
    }, 21000);

    $(".pagereload").click(function(){
        location.reload(true);
    });


    s.onopen = function(e) { $("#noconnection-modal").modal("hide");}
    s.onclose = function(e) { }
    s.onmessage = function(e) {
        data = JSON.parse(e.data);
        $("#debugdata").html("") ;
        $.each(data, function(i,j){
            $("#debugdata").append("<br>"+i+":"+j)
        })
    }
        
    $('div[id^="div-"]').hide();
    $("#div-home").show();
    $("#new_user_warning").hide();
    $("#new_user_success").hide();

    // When the new user form is submited. Run this function:
    $("#new_user").submit(function(e){
        e.preventDefault(); // Prevents any default form functions from happening
        if(!$("#current_password_input").val()){
            $("#new_user_warning").fadeOut();  
            $("#new_user_warning").html("Please complete all the form fields") ;
            $("#new_user_warning").fadeIn();
            
        }else if(!$("#new_user_input").val()){
            $("#new_user_warning").fadeOut();
            $("#new_user_warning").html("Please complete all the form fields") ;
            $("#new_user_warning").fadeIn();

        }else if(!$("#new_password_input").val()){
            $("#new_user_warning").fadeOut();
            $("#new_user_warning").html("Please complete all the form fields") ;
            $("#new_user_warning").fadeIn();
        }     
        else{
            $("#new_user_warning").hide();
            var data = {
            'password' : $("#current_password_input").val(), 
            'newuser' : $("#new_user_input").val(),  
            'newpass' : $("#new_password_input").val() ,
            }
            $.ajax({
                type:'POST',
                url:'php/newuser.php',
                data: data ,
        
            }).done(function(msg){
                if(msg == "success"){
                    $("#new_user_success").hide() ;
                    $("#new_user_success").fadeIn()
                }
                else if(msg == "username_in_use"){
                    $("#new_user_success").hide()
                    $("#new_user_warning").html("Username Already In Use") ;
                    $("#new_user_warning").fadeIn();
                } 
                else if(msg =="password_incorrect"){
                    $("#new_user_success").hide()
                    $("#new_user_warning").html("Current Password Incorrect") ;
                    $("#new_user_warning").fadeIn();
                }
                else{alert('An error occured --> '+msg)};
            })
        }  
    });

    $("#change_pass").submit(function(e){
        $("#change_password_success").hide() ;
        e.preventDefault(); // Prevents any default form functions from happening
        if(!$("#current_password_change").val()){
            $("#change_password_warning").fadeOut();  
            $("#change_password_warning").html("Please complete all the form fields") ;
            $("#change_password_warning").fadeIn();
            
        }else if(!$("#new_password_change").val()){
            $("#change_password_warning").fadeOut();
            $("#change_password_warning").html("Please complete all the form fields") ;
            $("#change_password_warning").fadeIn();
        }else if($("#new_password_change").val() != $("#new_password_change2").val()){
            $("#change_password_warning").fadeOut();
            $("#change_password_warning").html("Passwords Do Not Match") ;
            $("#change_password_warning").fadeIn();

        }     
        else{
            var data = {
                'password' : $("#current_password_change").val(),   
                'newpass' : $("#new_password_change").val() ,
                }
            $.ajax({
                type:'POST',
                url:'php/changepass.php',
                data: data ,
        
            }).done(function(msg){
                if(msg == "success"){
                    $("#change_password_warning").hide() ;
                    $("#change_password_success").fadeIn()
                }
                else if(msg =="password_incorrect"){
                    $("#change_password_warning").hide()
                    
                    $("#change_password_warning").html("Current Password Incorrect") ;
                    $("#change_password_warning").fadeIn();
                }
                else{alert('An error occured --> '+msg)};
            }) ;
        }
    }) ;

    $('#left-btn').click(function(){
        s.send("l")
    });

    $('#right-btn').click(function(){
        s.send("r")
    });

    $('#forward-btn').click(function(){
        s.send("f")
    });

    $('#back-btn').click(function(){
        s.send("b")
    });

    $('#auto-btn').click(function(){
        s.send("a")
    });

    $('#stop-btn').click(function(){
        s.send("x")
    });

    $('#man-control-send').click(function(){
        var left = $("#left-motor-amount").val()
        var right = $("#right-motor-amount").val()
        s.send("m"+left+","+right)
    });

    $("#navbar-toggle-id").click(function(){
        $('#navbarNav').toggle("collapse");
    })

    // The following JQuery Commands are for Showing and Hiding The Right Pane Content when a link is clicked
    $("#link-home").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-home").show();
        $(this).addClass("active");
        if($(window).width() < 991){ // if window width less than 991, ie the menu is shrank, collapse the menu item
            $('#navbarNav').toggle("collapse");
        }
    }) ;

    $("#link-control").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-control").show();
        $(this).addClass("active");
        if($(window).width() < 991){
            $('#navbarNav').toggle("collapse");
        }
    }) ;

    $("#link-nav").click(function(){
                $('div[id^="div-"]').hide();
                $('a[id^="link-"]').removeClass("active");
                $("#div-nav").show();
                $(this).addClass("active");
                if($(window).width() < 991){
                    $('#navbarNav').toggle("collapse");
                }
                imageObj.onload = function() {
                    div_width = $('#mapdiv').width() ; 
                    canvas1.width = div_width ; 
                    canvas2.width = div_width ; 
                    canvas3.width = div_width ; 
                    
                    wh_ratio = imageObj.width / imageObj.height ; 
                    new_height = div_width/wh_ratio ;
                    canvas1.height = new_height;
                    canvas2.height = new_height;
                    canvas3.height = new_height;
                    $("#mapdiv").height(new_height) ;
                    scale = imageObj.width/div_width ;
                    context1.drawImage(imageObj, 0, 0 , div_width ,new_height);
                };
                imageObj.src = 'php/map_generation.php';
            }) ;
      
    $("#mapdiv").click(function(e){
        context2.clearRect(0,0,div_width,new_height) ;
        var offset = $(this).parent().offset() ;
        x = e.pageX-offset.left ;
        y = e.pageY-offset.top ;
        context2.fillStyle="#FF0000" ;
        context2.fillRect(x-5,y-5,10,10) ;
        realx = Math.round(x*scale) ;
        realy = Math.round(y*scale) ; 
    });

    $("#mapdiv").mousemove(function(e){
        context3.clearRect(0,0,div_width,new_height) ; 
        var offset = $(this).parent().offset() ;
        x = e.pageX-offset.left ;
        y = e.pageY-offset.top ;
        context3.fillStyle="#FF0000" ;
        context3.fillRect(x-5,y-5,10,10) ;
        

    });

    $("#go-btn").click(function(){
        if(realx != null){
            alert("Sent") ;
        }
        else{
            alert("Please click a location on the Map")
        }
    }) ; 

    $("#link-debug").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-debug").show();
        $(this).addClass("active");
        if($(window).width() < 991){
            $('#navbarNav').toggle("collapse");
        }
    }) ;

    $("#link-status").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-status").show();
        $(this).addClass("active");
        if($(window).width() < 991){
            $('#navbarNav').toggle("collapse");
        }
    }) ;

    $("#link-settings").click(function(){
        $("#settings-modal").modal("show") ;
        if($(window).width() < 991){
            $('#navbarNav').toggle("collapse");
        }
        
    });

});
$(document).ready(function(){
    var timearray = [] ; 
    var s = new WebSocket("ws://192.168.1.154:5555");
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

    s.onopen = function(e) {}
    s.onclose = function(e) { }
    s.onmessage = function(e) {}
        
    $('div[id^="div-"]').hide();
    $("#div-home").show();

    //Check is existing Session Exists but calling check.php
    $.ajax({
        type:'POST',
        url:'php/check.php',

    }).done(function(msg){
        if(msg != "loggedin"){ 
            window.location = "login/"} ;
    })


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


    //when the login button is clicked login.php is called using AJAX
    
    // When the Sign out Button is clicked the logout.php script is called
    // $('#signout').click(function(){
    //     $.ajax(
    //         {
    //             type:'POST' ,
    //             url:'php/logout.php'
    //         }).done(function(){
    //             $("#loginfailed").hide() ;
    //             $('#username').val('');
    //             $('#password').val('');
    //             $(".content").fadeOut(1000,function(){
    //                 $(".login").fadeIn(1000)
    //             }) 

    //         })

    // });

    $("#navbar-toggle-id").click(function(){
        $('#navbarNav').toggle("collapse");
    })

    // The following JQuery Commands are for Showing and Hiding The Right Pane Content when a link is clicked
    $("#link-home").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-home").show();
        $(this).addClass("active");
        $('#navbarNav').toggle("collapse");
    }) ;

    $("#link-control").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-control").show();
        $(this).addClass("active");
        $('#navbarNav').toggle("collapse");
    }) ;

    $("#link-nav").click(function(){
                $('div[id^="div-"]').hide();
                $('a[id^="link-"]').removeClass("active");
                $("#div-nav").show();
                $(this).addClass("active");
                $('#navbarNav').toggle("collapse");
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


    // $("#link-map").click(function(){
    //     $('div[id^="div-"]').hide();
    //     $('a[id^="link-"]').removeClass("active");
    //     $("#div-map").show();
    //     $(this).addClass("active")
    // }) ;

    $("#link-debug").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-debug").show();
        $(this).addClass("active");
        $('#navbarNav').toggle("collapse");
    }) ;

    $("#link-status").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-status").show();
        $(this).addClass("active");
        $('#navbarNav').toggle("collapse");
    }) ;

});
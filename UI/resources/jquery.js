$(document).ready(function(){
    var timearray = [] ; 
    var s = new WebSocket("ws://localhost:5555");
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
        if(msg == "loggedin"){ 

        }else{
            window.location = "login/";
            
        } ; 
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
    $('#signout').click(function(){
        $.ajax(
            {
                type:'POST' ,
                url:'php/logout.php'
            }).done(function(){
                $("#loginfailed").hide() ;
                $('#username').val('');
                $('#password').val('');
                $(".content").fadeOut(1000,function(){
                    $(".login").fadeIn(1000)
                }) 

            })

    });


    // The following JQuery Commands are for Showing and Hiding The Right Pane Content when a link is clicked
    $("#link-home").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-home").show();
        $(this).addClass("active")
    }) ;

    $("#link-control").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-control").show();
        $(this).addClass("active")
    }) ;


    $("#link-nav").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-nav").show();
        $(this).addClass("active")
    }) ;

    $("#link-map").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-map").show();
        $(this).addClass("active")
    }) ;

    $("#link-debug").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-debug").show();
        $(this).addClass("active")
    }) ;

    $("#link-status").click(function(){
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-status").show();
        $(this).addClass("active")
    }) ;

});
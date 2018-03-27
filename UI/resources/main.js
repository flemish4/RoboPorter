$(document).ready(function () {

    $("#noconnection").hide();
    $("#noconnection-modal").modal("show");
    $("#change_password_warning").hide();
    $("#change_password_success").hide();
    $("#error-modal").modal("hide");
    $("#MapCanvas1").hide();
    $("#MapCanvas2").hide();
    $("#MapCanvas3").hide();
    $("#nav_command_sent").hide();



    //variables for WASD control
    var keycommandsent = null;
    var lastcommand;
    var map_reload;

    var curr_status = 0; 

    //Variables for Error Modal that shows when conection to roboporter is lost
    var num_debug_sent = 0;
    var prev_debug_sent = 0;
    var supress_error = false;

    // Creates websocket connection
    var s = new WebSocket("ws://192.168.0.1:5555");

    // Code below handles mapping canvas
    var canvas1 = document.getElementById('MapCanvas1');
    var context1 = canvas1.getContext('2d');
    var canvas2 = document.getElementById('MapCanvas2');
    var context2 = canvas2.getContext('2d');
    var canvas3 = document.getElementById('MapCanvas3');
    var context3 = canvas3.getContext('2d');
    var imageObj = new Image();
    var scale = null;
    var realx = null;
    var realy = null;
    var x = null;
    var y = null;
    var div_width;
    var wh_ratio;
    var new_height;
    var current_map ; 


    // Code below handles Debug Canvas
    var debug_canvas1 = document.getElementById('DebugCanvas1'); // Get Debug Canvas 1 ID i.e. the grey roboporter box
    var debug_context1 = debug_canvas1.getContext('2d');
    var debug_canvas2 = document.getElementById('DebugCanvas2'); // Get Debug Canvas 2 ID i.e. the bars
    var debug_context2 = debug_canvas2.getContext('2d');
    var csize = $("#DebugCanvasDiv").width();   // Get width of canvas div
    $("#DebugCanvasDiv").height(csize + 10); // Set height of div plus 10px for padding
    debug_canvas1.width = csize; // set canvas width to size of div width
    debug_canvas1.height = csize; // set height to div width i.e. make the canvas square
    debug_canvas2.width = csize;
    debug_canvas2.height = csize;

    //Draw Black Border Around Debug Canvas
    var borderwidth = 8;
    debug_context1.beginPath();
    debug_context1.lineWidth = borderwidth;
    debug_context1.strokeStyle = "black";
    debug_context1.rect(0.5, 0.5, csize - 0.5, csize - 0.5);
    debug_context1.stroke();

    // size of roboporter compared to width of div
    var dscale = 0.35;

    debug_context1.fillStyle = "#595959"; //roboporter color
    debug_context1.fillRect(((csize / 2) - (dscale * csize / 2)) + 0.5, (csize / 2) - (dscale * csize / 2) + 0.5, (csize * dscale), (csize * dscale)); // draw roboporter rectangle

    // Draw some text that shows when there is no debug data available
    debug_context2.fillStyle = "Red"; //Text Color
    debug_context2.font = "30px Arial"; // Font and 
    debug_context2.fillText("No Debug Data Available", 10, 50);
    debug_context2.fillStyle = "blue"; //ultrasonic bar color

    //Timer and time interval definitions
    setTimeout(function() { // Manages the connection timeout modal. After 21 seconds it fades in an alert saying no connection is available
        $("#connecting").fadeOut(1000, function () {
            $('#noconnection').fadeIn(1000);
            $('#pagereloadconn').prop('disabled', false); // Toggles the availability of the button
        });
    }, 21000);

    // A interval that checks new debug data has been sent. If prev value and current debug ID are the same. The connection modal is not shows and the error hasnt been supressed
    setInterval(function () {
        if ((prev_debug_sent == num_debug_sent) && ($('#noconnection-modal').is(':visible') == false) && (supress_error == false)) {
            $("#error-modal").modal("show");
        } else {
            //$("#error-modal").modal("hide");
            num_debug_sent = prev_debug_sent;
        }
    }, 2000);


    // Function definitions
    function map_names() {
        $.ajax({
            type: 'POST',
            url: 'php/map_names.php',

        }).done(function (msg) {
            $("#map_names_drop").html("");
            var num_maps = msg.split(",");
            $.each(num_maps, function (i, value) {
                $("#map_names_drop").append($('<option></option>').val(value).html(value));
            });
        })
    }

    function misccommand(command, setting, value) {
        var json = {
            'Type': 'MiscCommand',
            'Command': command,
            'Setting': setting,
            'Value': value,
        }
        var jsonstring = JSON.stringify(json);
        s.send(jsonstring + "$")
    }

    function userspeed(leftspeed, rightspeed) {
        var json = {
            'Type': 'UserSpeed',
            'Left': leftspeed,
            'Right': rightspeed,
        }

        var jsonstring = JSON.stringify(json);
        s.send(jsonstring + "$")
    }

    function mappingcommand() {

        var json = {
            'Type': 'CancelEnterUserCommand',
        }
        var jsonstring = JSON.stringify(json);
        s.send(jsonstring + "$")


        setTimeout(function() {
            var json = {
                'Type': 'MappingCommand',
            }
            var jsonstring = JSON.stringify(json);
            s.send(jsonstring + "$")
          },500);
        

    }

    function cancel() {
        var json = {
            'Type': 'Cancel',
        }
        var jsonstring = JSON.stringify(json);
        s.send(jsonstring + "$")
    }

    function navigationcommand(X, Y, map) {

        var json = {
            'Type': 'CancelEnterUserCommand',
        }
        var jsonstring = JSON.stringify(json);
        s.send(jsonstring + "$")

        setTimeout(function() {
            var json = {
                'Type': 'NavigationCommand',
                'X': X,
                'Y': Y,
                'Map_Filename' : map,
            }
            var jsonstring = JSON.stringify(json);
            s.send(jsonstring + "$")
          },500);


        
    }

    
    function testcommand(X, Y, map) {

        var json = {
            'Type': 'CancelEnterUserCommand',
        }
        var jsonstring = JSON.stringify(json);
        s.send(jsonstring + "$")

        setTimeout(function() {
            var json = {
                'Type': 'TestCommand',
                'X': X,
                'Y': Y,

            }
            var jsonstring = JSON.stringify(json);
            s.send(jsonstring + "$")
          },500);


        
    }

    function usercommand() {
        var json = {
            'Type': 'CancelEnterUserCommand',
        }
        var jsonstring = JSON.stringify(json);
        s.send(jsonstring + "$")
        
        setTimeout(function() {
            var json = {
                'Type': 'UserCommand',
            }
            var jsonstring = JSON.stringify(json);
            s.send(jsonstring + "$")
          },500);
    }

    function delete_map_radio() {
        $.ajax({
            type: 'POST',
            url: 'php/map_names.php',

        }).done(function (msg) {
            $("#delete-map-options").html("");
            var num_maps = msg.split(",");
            $.each(num_maps, function (i, value) {
                $("#delete-map-options").append(" <div class='form-check'><label class='form-check-label'><input class='form-check-input' type='radio' name='deleteradio' value='" + value + "'>" + value + "</label></div>");
            });
        })

    }



    function mapload() {
        var link = "/php/map_generation_temp.php?" + (new Date().getTime()) // Forces a reload not from the cache
        $(".current_map_image").attr("src", link);
    }


    //Manages socket send and recieve
    s.onopen = function (e) { }
    s.onclose = function (e) { //$("#error-modal").modal("show");
    }
    s.onmessage = function (e) {

        $("#noconnection-modal").modal("hide");
        data = JSON.parse(e.data); // parses the JSON string into a Javascript object

        $(".debugdata").html(""); // Clear previous values

        debug_context2.clearRect(0, 0, csize + 0.5, csize + 0.5); // Deletes all of the ultrasonic bars already drawn on the canvas
        // Write to ultrasonic debug screen
        try { //try to set bar lengths
            var ultrasonic_offset = 0.1; // offset to make sure dividing by zero never happens 
            var maxbarlen = (csize - dscale * csize) / 2 - borderwidth / 2; // The maximum ultrasonic bar length 
            var dh1 = ((parseInt(data['US1'], 10) + ultrasonic_offset) / 300) * maxbarlen;// length of ultrasonic bar 1
            var dh2 = ((parseInt(data['US2'], 10) + ultrasonic_offset) / 300) * maxbarlen;
            var dh3 = ((parseInt(data['US3'], 10) + ultrasonic_offset) / 300) * maxbarlen;
            var dh4 = ((parseInt(data['US4'], 10) + ultrasonic_offset) / 300) * maxbarlen;
            var dh5 = ((parseInt(data['US5'], 10) + ultrasonic_offset) / 300) * maxbarlen;
            var dh6 = ((parseInt(data['US6'], 10) + ultrasonic_offset) / 300) * maxbarlen;
            var dh7 = ((parseInt(data['US7'], 10) + ultrasonic_offset) / 300) * maxbarlen;
            var dh8 = ((parseInt(data['US8'], 10) + ultrasonic_offset) / 300) * maxbarlen;
            var dw = dscale * csize / 2; // ultrasonic bar width
            //fill rectangles for the top
            debug_context2.fillRect((csize / 2) * (1 - dscale) + 0.5, (csize / 2) * (1 - dscale) - dh3 + 0.5, dw, dh3);
            debug_context2.fillRect((csize / 2) * (1 - dscale) + dw + 0.5, (csize / 2) * (1 - dscale) - dh4 + 0.5, dw, dh4);

            //fill rectangles for the right side
            debug_context2.fillRect((csize / 2) + (dscale * csize / 2) + 0.5, (csize / 2) * (1 - dscale) + 0.5, dh5, dw);
            debug_context2.fillRect((csize / 2) + (dscale * csize / 2) + 0.5, (csize / 2) * (1 - dscale) + dw + 0.5, dh6, dw);

            //fill rectangles for the bottom
            debug_context2.fillRect((csize / 2) * (1 - dscale) + 0.5, (csize / 2) * (1 - dscale) + (csize * dscale) + 0.5, dw, dh8);
            debug_context2.fillRect((csize / 2) * (1 - dscale) + dw + 0.5, (csize / 2) * (1 - dscale) + (csize * dscale) + 0.5, dw, dh7);

            //fill rectangles for the left side
            debug_context2.fillRect((csize / 2) - (dscale * csize / 2) - dh1 + 0.5, (csize / 2) * (1 - dscale), dh1 + 0.5, dw);
            debug_context2.fillRect((csize / 2) - (dscale * csize / 2) - dh2 + 0.5, (csize / 2) * (1 - dscale) + dw, dh2 + 0.5, dw);

            // Delete ultrasonic data from javascript object. This allows any remaining data to be printed in a for loop
            // delete data["US1"];
            // delete data["US2"];
            // delete data["US3"];
            // delete data["US4"];
            // delete data["US5"];
            // delete data["US6"];
            // delete data["US7"];
            // delete data["US8"];
            delete data["Type"];

            num_debug_sent = parseInt(data["Debug Data Sent"], 10);

            $(".current-status").html(data["System Status"]);
            var batterywidth = parseInt(data["Battery"], 10) + "%";
            $("#battery-width").width(batterywidth);
            $("#battery-percentage").html(data["Battery"]);

            // Change colour of safety button to indicate active or not active 

            if (data["Safety ON"] == "True") {
                $("#safety-btn").removeClass("btn-secondary");
                $("#safety-btn").addClass("btn-success");
            } else {
                $("#safety-btn").removeClass("btn-success");
                $("#safety-btn").addClass("btn-secondary");
            }
            curr_status = data["System Status"] ;
            
            if ((data["System Status"] == "Mapping") || (data["System Status"] == "Navigation")) {
                $('.mancontrol').prop('disabled', true);
            }else{
                $('.mancontrol').prop('disabled', false);
            }

            if(curr_status == "UserCommand"){ // If the mode switched to user command hide the div that alerts the user that the robot is not in user command mode
                $("#usercontrol-modal").modal("hide") ;
            }

            //Print any remaining debug data to the debug data div
            $.each(data, function (i, j) {
                $(".debugdata").append("<br>" + i + " : " + j);
            })

            //Print data to home screen debug card
            $("#home-debug-data").html("Speed Vector Left: " + data["Speed Vector Left"] + "<br> Speed Vector Right: " + data["Speed Vector Right"])

        } catch (err) {
            alert(err)
            debug_context2.fillStyle = "Red"; //Text Color
            debug_context2.font = "30px Arial"; // Font and 
            debug_context2.fillText("No Debug Data Available", 10, 50);
        }
    }

    var wid = $("#stream-failed").width();
    $("#stream-failed").height(0.5625 * wid);

    window.onresize = function () {
        var wid = $("#stream-failed").width();
        $("#stream-failed").height(0.5625 * wid);
    }

    // detects keyboard input when the modal is shown
    $(document).keydown(function (e) {
        if (($('#keyinput-modal').is(':visible') == true) || ($('#map-3').is(':visible') == true)) {
            var keycode = e.which;
            switch (keycode) {
                case 87:
                    if (lastcommand != "f") {
                        $(".up-key").addClass("red");
                        lastcommand = "f"
                        userspeed(10, 10);
                    }
                    break;
                case 65:
                    if (lastcommand != "l") {
                        $(".left-key").addClass("red");
                        userspeed(-5, 5);
                        lastcommand = "l"
                    }
                    break;
                case 83:
                    if (lastcommand != "b") {
                        $(".down-key").addClass("red");
                        userspeed(-10, -10);
                        lastcommand = "b"
                    }
                    break;
                case 68:
                    if (lastcommand != "r") {
                        $(".right-key").addClass("red");
                        userspeed(5, -5);
                        lastcommand = "r"
                    }
                    break;
            }
            keycommandsent = true;
        }

    });

    // detects keyboard input when the modal is shown
    $(document).keyup(function (e) {
        if (($('#keyinput-modal').is(':visible') == true) || ($('#map-3').is(':visible') == true)) {
            var keycode = e.which;
            switch (keycode) {
                case 87:
                    $(".up-key").removeClass("red");
                    userspeed(0, 0);
                    break;
                case 65:
                    $(".left-key").removeClass("red");
                    userspeed(0, 0);
                    break;
                case 83:
                    $(".down-key").removeClass("red");
                    userspeed(0, 0);
                    break;
                case 68:
                    $(".right-key").removeClass("red");
                    userspeed(0, 0);
                    break;
            }
            lastcommand = "x";
            keycommandsent = false;
        }

    });

    // makes sure the last command sent from the keyboard modal is stop
    $("#keyclose").click(function () {
        if (keycommandsent != null) {
            userspeed(0, 0);
            keycommandsent = null;
        };

    });

    $('div[id^="div-"]').hide();
    $("#div-home").show();
    $("#new_user_warning").hide();
    $("#new_user_success").hide();

    // When the error close button is used set supress_error to true. This stops the modal reappearing. 
    $("#error_modal_close").click(function () {
        supress_error = true;
    })


    // reloads the page
    $(".pagereload").click(function () {
        location.reload(true);
    });


    //The following commands deal with manual control buttons
    $('#enter_user_mode').click(function(){
        usercommand() ; 
    })

    $('.left-btn').click(function () {
        userspeed(-20, 20);
    });

    $('.right-btn').click(function () {
        userspeed(20, -20);
    });

    $('.forward-btn').click(function () {
        userspeed(20, 20);
    });

    $('.back-btn').click(function () {
        userspeed(-20, -20);
    });

    $('#auto-btn').click(function () {
    });

    $('.stop-btn').click(function () {
        userspeed(0, 0)

    });

    $('#safety-btn').click(function () {
        misccommand("s", 0, 0)
    });

    $('#shutdown-btn').click(function () {
        misccommand("x", 0, 0)

    });

    $('#man-control-send').click(function () {
        var left = $("#left-motor-amount").val()
        var right = $("#right-motor-amount").val()
        userspeed(left, right);
    });

    // The following command colapses the nv bar when clicked
    $("#navbar-toggle-id").click(function () {
        $('#navbarNav').toggle("collapse");
    })


    // The following JQuery Commands are for Showing and Hiding The Content when a link is clicked
    $("#link-home").click(function () {
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-home").show();
        $(this).addClass("active");
        if ($(window).width() < 768) { // if window width less than 768, ie the menu is shrank, collapse the menu item
            $('#navbarNav').toggle("collapse");
        }
    });

    $("#link-control").click(function () {
        if(curr_status == "UserCommand"){
            $("#usercontrol-modal").modal("hide")
        }else{
            $("#usercontrol-modal").modal("show")
        }
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-control").show();
        $(this).addClass("active");
        if ($(window).width() < 768) {
            $('#navbarNav').toggle("collapse");
        }
    });

    $("#link-nav").click(function () {
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-nav").show();
        $(this).addClass("active");
        if ($(window).width() < 768) {
            $('#navbarNav').toggle("collapse");
        }


        // context1.fillStyle="Black" ; //Text Color
        // context1.font = "30px Arial"; // Font and 
        // context1.fillText("Please choose a map",10.5,50.5);
        map_names()

    });

    $("#link-debug").click(function () {
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-debug").show();
        $(this).addClass("active");
        if ($(window).width() < 768) {
            $('#navbarNav').toggle("collapse");
        }
    });

    $("#link-status").click(function () {
        $('div[id^="div-"]').hide();
        $('a[id^="link-"]').removeClass("active");
        $("#div-status").show();
        $(this).addClass("active");
        if ($(window).width() < 768) {
            $('#navbarNav').toggle("collapse");
        }
    });

    $("#link-settings").click(function () {
        $("#settings-modal").modal("show");
        if ($(window).width() < 768) {
            $('#navbarNav').toggle("collapse");
        }

    });


    // The following JQUERY commands are for managing the mapping canvas and changing the map based on the dropdown selection
    $("#mapdiv").click(function (e) {
        context2.clearRect(0, 0, div_width, new_height);
        var offset = $(this).parent().offset();
        x = e.pageX - offset.left;
        y = e.pageY - offset.top;
        context2.fillStyle = "#FF0000";
        context2.fillRect(x - 5, y - 5, 10, 10);
        realx = Math.round(x * scale);
        realy = Math.round(y * scale);
        $("#nav_command_sent").fadeOut();
    });

    $("#mapdiv").mousemove(function (e) {
        context3.clearRect(0, 0, canvas1.width, canvas1.height);
        var offset = $(this).parent().offset();
        x = e.pageX - offset.left;
        y = e.pageY - offset.top;
        context3.fillStyle = "#FF0000";
        context3.fillRect(x - 5, y - 5, 10, 10);
    });

    $("#go-btn").click(function () {
        if (realx != null) {
            var data = {
                'name': current_map,
            }
            $.ajax({
                type: 'POST',
                url: 'php/name_to_id.php',
                data: data,
            }).done(function (msg) {
                navigationcommand(realx,realy,parseInt(msg)) ; 
                $("#nav_command_sent").fadeIn();


            });
            
        }
        else {
            alert("Please click a location on the Map")
        }
    });

    $("#map_names_drop").change(function () {
        current_map = $("#map_names_drop").val()
        var dropdowndata = {
            'name': $("#map_names_drop").val(),
        }
        $.ajax({
            type: 'POST',
            url: 'php/name_to_id.php',
            data: dropdowndata,
        }).done(function (msg) {
            imageObj.onload = function () {
                $("#MapCanvas1").show();
                $("#MapCanvas2").show();
                $("#MapCanvas3").show();
                div_width = $('#mapdiv').width();

                realx = null;
                realy = null;
                canvas1.width = div_width;
                canvas2.width = div_width;
                canvas3.width = div_width;

                wh_ratio = imageObj.width / imageObj.height;
                new_height = div_width / wh_ratio;
                canvas1.height = new_height;
                canvas2.height = new_height;
                canvas3.height = new_height;
                $("#mapdiv").height(new_height);
                scale = imageObj.width / div_width;

                context1.drawImage(imageObj, 0.5, 0.5, div_width, new_height);

                
                context1.beginPath(); // draw border around debug canvas
                context1.lineWidth = borderwidth;
                context1.strokeStyle = "black";
                context1.rect(-0.5, -0.5, div_width + 0.5, new_height + 0.5);
                context1.stroke();
            };
            var imagesrc = "php/map_generation.php?ID=" + msg
            imageObj.src = imagesrc;
        })



    });


    // The following JQUERY commands are for handling the new mapping modal

    $("#Auto-map").click(function () {
        $("#map-1").fadeOut(function () {
            mappingcommand()
            $("#map-2").fadeIn();
        })
    });

    $("#Manual-map").click(function () {
        $("#map-1").fadeOut(function () {
            usercommand()
            $("#map-3").fadeIn();
        })
    });

    $("#new-map-btn").click(function () {
        $.ajax({
            type: 'POST',
            url: 'php/delete_temp.php',
        }).done(function (msg) {
            if(msg == "Success"){
                $("#map-2").hide();
                $("#map-3").hide();
                $("#map-4").hide();
                $("#map-1").show();
                map_reload = setInterval(mapload, 1000);
                $('#save-map-btn').prop('disabled', false); 
            }
        })
        
    });

    $(".end-mapping").click(function () {
        cancel()
        $("#map-saving").hide();
        $("#map-saving-error").hide();
        $("#map-saving-success").hide();
        clearInterval(map_reload);
        var link = "/php/map_generation_temp.php?" + (new Date().getTime()) // Forces a reload not from the cache
        $("#final_map").attr("src", link);
        $(this).parent().parent().parent().parent().fadeOut(function () {
            $("#map-4").fadeIn();
        })
    });

    $("#set-map-name").submit(function (e) {
        $("#map-saving-error").hide();
        e.preventDefault();

        if (!$("#map-name").val()) {
            $("#map-saving-error").fadeIn();
        }
        else {
            $("#map-saving").fadeIn();
            var newname = $("#map-name").val();
            var data = {
                'name': newname,
            }
            $.ajax({
                type: 'POST',
                url: 'php/map_save.php',
                data: data,
            }).done(function (msg) {
                if (msg == "Success") {
                    map_names();
                    $('#save-map-btn').prop('disabled', true);
                    $("#map-saving").fadeOut(function () {
                        $("#map-saving-success").fadeIn();
                    });
                } else {
                    $("#map-saving").fadeOut(function () {
                        $("#map-saving-error").fadeIn();
                    });
                }
            })

        }

    });

    $("#delete-map-btn").click(function () {
        delete_map_radio();
    })

    $("#delete-map-submit").click(function () {
        var data = {
            'name': $('input[name=deleteradio]:checked').val(),
        }
        $.ajax({
            type: 'POST',
            url: 'php/name_to_id.php',
            data: data,
        }).done(function (msg) {
            var data = {
                'name': msg,
            }
            $.ajax({
                type: 'POST',
                url: 'php/delete_map.php',
                data: data,
            }).done(function (msg2) {
                alert(msg2) ;
                delete_map_radio();
                map_names();
            });


        });

    })

    $("#cancel-map").click(function(){
        cancel() ; 
    })

    // Temp test code 
    $("#man-submit").click(function(){
        var manx = $("#x-manual").val() ; 
        var many = $("#y-manual").val() ; 
        testcommand(manx,many) ; 
    })
    // End of temp code 


    // The following code handles changing and creating users
    // When the new user form is submited. Run this function:
    $("#new_user").submit(function (e) {
        e.preventDefault(); // Prevents any default form functions from happening
        if (!$("#current_password_input").val()) {
            $("#new_user_warning").fadeOut();
            $("#new_user_warning").html("Please complete all the form fields");
            $("#new_user_warning").fadeIn();

        } else if (!$("#new_user_input").val()) {
            $("#new_user_warning").fadeOut();
            $("#new_user_warning").html("Please complete all the form fields");
            $("#new_user_warning").fadeIn();

        } else if (!$("#new_password_input").val()) {
            $("#new_user_warning").fadeOut();
            $("#new_user_warning").html("Please complete all the form fields");
            $("#new_user_warning").fadeIn();
        }
        else {
            $("#new_user_warning").hide();
            var data = {
                'password': $("#current_password_input").val(),
                'newuser': $("#new_user_input").val(),
                'newpass': $("#new_password_input").val(),
            }
            $.ajax({
                type: 'POST',
                url: 'php/newuser.php',
                data: data,

            }).done(function (msg) {
                if (msg == "success") {
                    $("#new_user_success").hide();
                    $("#new_user_success").fadeIn()
                }
                else if (msg == "username_in_use") {
                    $("#new_user_success").hide()
                    $("#new_user_warning").html("Username Already In Use");
                    $("#new_user_warning").fadeIn();
                }
                else if (msg == "password_incorrect") {
                    $("#new_user_success").hide()
                    $("#new_user_warning").html("Current Password Incorrect");
                    $("#new_user_warning").fadeIn();
                }
                else { alert('An error occured --> ' + msg) };
            })
        }
    });

    // When the change password button is submitted
    $("#change_pass").submit(function (e) {
        $("#change_password_success").hide();
        e.preventDefault(); // Prevents any default form functions from happening
        if (!$("#current_password_change").val()) {
            $("#change_password_warning").fadeOut();
            $("#change_password_warning").html("Please complete all the form fields");
            $("#change_password_warning").fadeIn();

        } else if (!$("#new_password_change").val()) {
            $("#change_password_warning").fadeOut();
            $("#change_password_warning").html("Please complete all the form fields");
            $("#change_password_warning").fadeIn();
        } else if ($("#new_password_change").val() != $("#new_password_change2").val()) {
            $("#change_password_warning").fadeOut();
            $("#change_password_warning").html("Passwords Do Not Match");
            $("#change_password_warning").fadeIn();

        }
        else {
            var data = {
                'password': $("#current_password_change").val(),
                'newpass': $("#new_password_change").val(),
            }
            $.ajax({
                type: 'POST',
                url: 'php/changepass.php',
                data: data,

            }).done(function (msg) {
                if (msg == "success") {
                    $("#change_password_warning").hide();
                    $("#change_password_success").fadeIn()
                }
                else if (msg == "password_incorrect") {
                    $("#change_password_warning").hide()

                    $("#change_password_warning").html("Current Password Incorrect");
                    $("#change_password_warning").fadeIn();
                }
                else { alert('An error occured --> ' + msg) };
            });
        }
    });
});
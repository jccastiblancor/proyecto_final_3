var cmdStart;
var robot_IP;
var ros;

var subscriptor;
var publisher;
var msj;

var rojo = 0;
var verde = 0;
var azul = 0;

var derecha = 0;
var izquierda = 0;
var slide = "slide_down"
var angulo = -135
var orientacion = 0


function refrescar(){
    location.reload();
}

function publish() {
    console.log('publicando')

    var x = document.getElementById("frm1");
    var text = "";
    var i;
    for (i = 0; i < x.length ;i++) {
        text += x.elements[i].value + " ; ";
    }
    publisher.data = text
    console.log(publisher.data)
    cmdStart.publish(publisher);
    setTimeout(refrescar, 2000);  
}

function iniciar(){
    robot_IP = "localhost";

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    ros.on('conection', function() {
        console.log("Connected to websocket server.");
    });
    
    ros.on('error', function(error) {
        console.log("Error connecting to websocket server: ", error);
    });
    
    ros.on('close', function() {
        console.log("Connection to websocket server closed.");
    });

    // ----------------------------
    // Iniciar listener y publisher.
    // ----------------------------

    this.subscriptorMovimiento = new ROSLIB.Topic({
        ros: ros,
        name: '/pioneer_motorsVel',
        messageType: 'std_msgs/Float32MultiArray'
    })

    this.subscriptorPosicion = new ROSLIB.Topic({
        ros: ros,
        name: '/pioneer_position',
        messageType: 'geometry_msgs/Twist'
    })

    this.subscriptorColores = new ROSLIB.Topic({
        ros: ros,
        name: '/colores',
        messageType: 'std_msgs/Float32MultiArray'
    })

    publisher = new ROSLIB.Message({
        data : "hello world"
    });

    // Init topic object
    cmdStart = new ROSLIB.Topic({
        ros: ros,
        name: '/start',
        messageType: 'std_msgs/String'
    });
    // Register publisher within ROS system
    cmdStart.advertise();

    // ---------------------------------
    // Iniciar variables a mostrar en UI.
    // --------------------------------

    document.getElementById("rojo").innerHTML=String(rojo);
    document.getElementById("verde").innerHTML=String(verde);
    document.getElementById("azul").innerHTML=String(azul);
    document.getElementById("rueda_derecha").innerHTML=String(this.derecha);
    document.getElementById("rueda_izquierda").innerHTML=String(this.izquierda);
    document.getElementById("orientacion").innerHTML=String(this.orientacion);

    refrescar(false)
}

function arrows_direction(p_slide){
    for (let i = 0; i < 4; i++) {

        var element = document.getElementById('flecha' + i);
        if(p_slide=='slide_der'){
            if (i==0){
                element.className = 'arrowSliding_der';
            } else{
                element.className = 'arrowSliding_der delay' + i;
            }
            
        }else if(p_slide=='slide_up'){
            if (i==0){
                element.className = 'arrowSliding_up';
            } else{
                element.className = 'arrowSliding_up delay' + i;
            }
        }else if(p_slide=='slide_down'){
            if (i==0){
                element.className = 'arrowSliding_down';
            } else{
                element.className = 'arrowSliding_down delay' + i;
            }
        }else{
            if (i==0){
                element.className = 'arrowSliding';
            } else{
                element.className = 'arrowSliding delay' + i;
            }
        }
    } 
    var element = document.getElementById('animate');
    

}

function refrescar(bono){

    let myElements = document.querySelectorAll(".arrow");
    for (let i = 0; i < myElements.length; i++) {
        myElements[i].style.transform = `rotate(${angulo}deg)`;
    }

    arrows_direction(slide)

    if(bono){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/bono.png?' + new Date().getTime()  alt='Card image'>";
    } else if (this.rojo > this.verde && this.rojo > this.azul){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/pez.png' alt='Card image'>";
    }else if (this.rojo < this.verde && this.verde > this.azul){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/gato.png' alt='Card image'>";
    }else if (this.azul > this.rojo && this.azul > this.verde){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/pato.png' alt='Card image'>";
    }else {
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/jirafa.png' alt='Card image'>";
    }

    setTimeout(refrescar, 1000);
}


window.onload = function () {

    this.iniciar()

    // this.updateImage()

    // ----------------------------
    // Usar listener.
    // ----------------------------    

    subscriptorMovimiento.subscribe(function(message){

        derecha = message.data[1].toFixed(2)
        izquierda = message.data[0].toFixed(2)

        document.getElementById("rueda_derecha").innerHTML=String(derecha);
        document.getElementById("rueda_izquierda").innerHTML=String(izquierda);
        
    })

    subscriptorPosicion.subscribe(function(message){

        orientacion = message.angular.z
        if (orientacion <0.78 && orientacion>-0.78){
            angulo = -135
            slide = "slide_down"
        } else if (orientacion >0.78 && orientacion<2.35){
            angulo = 135
            slide = "slide_der"
        } else if (orientacion <-0.78 && orientacion>-2.35){
            angulo = -45
            slide = "slide"
        } else {
            angulo = 45
            slide = "slide_up"
        }

        document.getElementById("orientacion").innerHTML=String((orientacion*180/3.14).toFixed(2));
        document.getElementById("posicion").innerHTML = `[ ${message.linear.x.toFixed(2)} m , ${message.linear.y.toFixed(2)} m ]`; 
    })

    subscriptorColores.subscribe(function(message){
        console.log(message.data)
        rojo = message.data[2].toFixed(0)
        verde = message.data[1].toFixed(0)
        azul = message.data[0].toFixed(0)

        document.getElementById("rojo").innerHTML=String(rojo);
        document.getElementById("verde").innerHTML=String(verde);
        document.getElementById("azul").innerHTML=String(azul);
    })
    
      

}
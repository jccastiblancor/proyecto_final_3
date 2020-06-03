var cmdStart;
var robot_IP;
var ros;

var subscriptor;
var publisher;
var msj;

var rojo = 1;
var verde = 2;
var azul = 3;

var derecha = 0;
var izquierda = 0;


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
        name: '/chatter',
        // messageType: 'geometry_msgs/Twist'
    })

    this.subscriptorColores = new ROSLIB.Topic({
        ros: ros,
        name: '/colores',
        // messageType: 'std_msgs/MultiArrayLayout'
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

    mostrar_dibujo(false)
}

function mostrar_dibujo(bono){

    if(bono){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/bono.png' alt='Card image'>";
    } else if (this.rojo > this.verde && this.rojo > this.azul){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/pez.png' alt='Card image'>";
    }else if (this.rojo < this.verde && this.verde > this.azul){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/gato.png' alt='Card image'>";
    }else if (this.azul > this.rojo && this.azul > this.verde){
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/pato.png' alt='Card image'>";
    }else {
        document.getElementById('dibujo').innerHTML="<img class='card-img-top' src='./img/jirafa.png' alt='Card image'>";
    }

}



window.onload = function () {

    this.iniciar()

    // ----------------------------
    // Usar listener.
    // ----------------------------    

    subscriptorMovimiento.subscribe(function(message){
        console.log('Info del mensaje: \n', message.data);
        msj = message.data

        //derecha = message.linear
        //izquierda = message.linear

        document.getElementById("rueda_derecha").innerHTML=String(this.derecha);
        document.getElementById("rueda_izquierda").innerHTML=String(this.izquierda);
    })

    subscriptorMovimiento.subscribe(function(message){
        console.log('Info del mensaje: \n', message.data);
        rojo = message.data[0]
        verde = message.data[1]
        azul = message.data[2]

        //derecha = message.linear
        //izquierda = message.linear

        document.getElementById("rojo").innerHTML=String(rojo);
        document.getElementById("verde").innerHTML=String(verde);
        document.getElementById("azul").innerHTML=String(azul);
    })

      

}
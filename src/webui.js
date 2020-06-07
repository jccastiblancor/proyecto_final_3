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

    console.log('DIBUJO');
    console.log(rojo, verde, azul);

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

    setTimeout(mostrar_dibujo, 1000);
}


window.onload = function () {

    this.iniciar()

    // this.updateImage()

    // ----------------------------
    // Usar listener.
    // ----------------------------    

    subscriptorMovimiento.subscribe(function(message){
        //console.log('Info del mensaje ruedas: \n', message.data);

        derecha = message.data[1].toFixed(2)
        izquierda = message.data[0].toFixed(2)

        //console.log('derecha: \n', derecha);
        //console.log('izquierda: \n', izquierda);
        rojo = derecha
        verde = izquierda

        document.getElementById("rueda_derecha").innerHTML=String(derecha);
        document.getElementById("rueda_izquierda").innerHTML=String(izquierda);

        document.getElementById("rojo").innerHTML=String(derecha);
        document.getElementById("verde").innerHTML=String(izquierda);

        
    })

    subscriptorColores.subscribe(function(message){
        console.log('Info del mensaje: \n', message.data);
        rojo = message.data[0]
        verde = message.data[1]
        azul = message.data[2]

        document.getElementById("rojo").innerHTML=String(rojo);
        document.getElementById("verde").innerHTML=String(verde);
        document.getElementById("azul").innerHTML=String(azul);
    })
    
      

}
const char MAIN_page[] PROGMEM = R"=====(



<!DOCTYPE html>
<html>
<head>
<meta http-equiv="content-type" content="text/html; charset=ISO-8859-1">
    <title>ESP8266 SBR</title>
  

<meta name="viewport" content="width=device-width, initial-scale=1">

<style>
#page{
  min-height:100%;
  max-height:1080px;
  max-width: 1920px;
}

#header {
    width: 100%;
    margin: 0;
    padding: 0;
}

#content {
    min-height: 48vh;
    float: left;
    width: 100%;
    margin: 0 0 20px 0;
    padding: 0;
}

#content, #bloc, #bloc2 {
  text-indent:center;
  text-align: center;
}


#footer {
    clear: both;
    width: 100%;
    margin: 0px;
    padding: 10px 20px;
  right: 0;
  bottom: 0;
  left: 0;
}

body {
    padding: 0px;
    margin:0px;
    font-family: Verdana, Geneva, sans-serif;
    line-height: 1.2em;
    font-size:1em;
}

h1 {
    line-height: 100%;
    font-size: 1.6em;
  text-align:center;
}

h2{
  font-size: 1.2em;
}
body {
    background-color: #cccccc;
  background-repeat: no-repeat;
    background-attachment: fixed;
    background-position: center; 
  max-height:1080;
  max-width: 1920;

}

#header, #navigation, #footer, h1, h2 {
    font-family: Verdana, Geneva, sans-serif;
}

#header {
  display:inline-block;
    color: #fff;
    background: rgba(50, 50, 50, 0.8);  
}

#content, #bloc, #bloc2 {
  padding:20px;
  color: #332;
}

.vertical{
  height:800;
  width:600;
  display: block;
    margin:0 auto;
}

.horizzontal{
  height:600;
  width:800;
    display: block;
    margin:0 auto;
}

#bloc , #bloc2 {
  color: #fff;
  background: rgba(176, 189, 202, 1);
  font-size: 85%;
} 

#navigation ul {
  list-style: none;
  text-align: center;
    color: #333;
    background:rgba(13, 17, 54,0.9);
    overflow: hidden;    
}

#navigation li {
    display:inline-block;
}

#navigation a {
    display: block;
    color: #fff;
    text-decoration: none;
    padding: 10px;
    font-weight: bold;
}

#navigation a:hover {
    background: rgb(150, 150, 150)
}

#footer { 
  color: #fff;
  background: rgba(50, 50, 50, 10);  
  margin-bottom:0;
}

#footer h3 {
  padding: 10px 20px;
}

.button , .buttonm , .buttonh , .buttont, .buttontm, .buttonth{

    color: white;
    text-align: center;
  text-indent:left;
    text-decoration: none;

}

.buttont{
    background-color: #4CAF50; /* Green */
  padding:5px 5px;
}

.buttontm{
    background-color: #FFA500; /* Orange */
  padding:5px 5px;
}

.buttonth{
    background-color: #f44336; /* Red */
  padding:5px 5px;
}

.button {
    background-color: #4CAF50; /* Green */
  padding:5px 17px;
}

.buttonm {
    background-color: #FFA500; /* Orange */
  padding:5px 13px;
}

.buttonh {
    background-color: #f44336; /* Red */
  padding:5px 10px;
}

</style>
 </head>
 
<body onLoad="getLoadData()">


<div id="header">
    <p style="padding: 20px 10px 20px 20px;"> 
    <h2>ESP8266 Self Balancing Robot</h2>
    </p>
</div>
  
<div id="page">  

<nav id="navigation">
      <ul>
        <li><a id="a" onClick="seeConfiguration()">Configure</a></li>
        <li><a id="b" onClick="seeControl()">Control</a></li>
      </ul>
</nav>  

<div id="content">
      ADC Battery Value is minimum is(800) : <span id="ADCValue">0</span><br>
  <div id="bloc">
        <h1>Configuration</h1>
  Reset Reason is : <span id="ResetReason">null</span><br>
   <div>
   <label>Proportional value</label><br>
   <input type="number" name="P" id="P"  value=""><br>
   <button type="button" onclick="sendP()">Set proportional value</button>
   </div> <br>
   <div>
   <label>Integrative value</label><br>
   <input type="number" name="I" id="I"  value=""><br>
   <button type="button" onclick="sendI()">Set integrative value</button>
   </div> <br>
   <div>
   <label>Derivative value</label><br>
   <input type="number" name="D" id="D"  value=""><br>
   <button type="button" onclick="sendD()">Set derivative value</button>
   </div> <br>
   <div>
   <label>Minimum Motor Voltage</label><br>
   <input type="number" name="mm" id="mm"  value=""><br>
   <button type="button" onclick="sendMm()">Set Minimum Voltage</button>
   </div><br>
   <div>
   <label>Maximum Motor Voltage</label><br>
   <input type="number" name="mM" id="mM"  value=""><br>
   <button type="button" onclick="sendMM()">Set Maximum Voltage</button>
   </div> <br>
   <div>
   <label>Zero Angle</label><br>
   <input type="number" name="za" id="za"  value=""><br>
   <button type="button" onclick="sendZa()">Set zero angle</button>
   </div><br>
   <div>
   <label>Turn Speed</label><br>
   <input type="number" name="ts" id="ts"  value=""><br>
   <button type="button" onclick="sendTs()">Set turn speed</button>
   </div>
   <br>
   <div>
   <label>Offset</label> <br>
   <button type="button" onclick="sendOffset(5)"> + </button>
   <span id="offset">0</span>
   <button type="button" onclick="sendOffset((0-5))"> - </button>
   </div>
   <br>
   <div>
   <input type="checkbox" id="OTA" onclick="setOTA()" checked>ToggleOTA 
   </div>  
   <br>
    <div>
   <label>Distance</label>
   <span id="distance">0</span>
   <button type="button" onclick="getDistance()">Get Distance</button>
   </div>   
   <br><br>
   <div>
   <button type="button" onclick="sendAD()">Set Adjust Angle</button>
   </div>
   <div>
   <button type="button" onclick="resetESP()">Reset the ESP8266</button>
   </div>
   <div>
   <button type="button" onclick="resetAndSave()">Reset And save</button>
   </div>
   </div>
         <div id="bloc2" >
      <h1>Control</h1>
   <table>
     <tr>
   <td></td>
   <td></td>
    <td></td>
    <td><button class="buttonh" type="button" onclick="RControl("3")">&#8593;&#8593;&#8593;</button></td>
   <td></td>
   <td></td>
    <td></td>
  </tr>
     <tr>
   <td></td>
   <td></td>
    <td></td>
    <td><button class="buttonm" type="button" onclick="RControl("2")">&#8593;&#8593;</button></td>
   <td></td>
   <td></td>
    <td></td>
  </tr>
     <tr>
   <td></td>
   <td></td>
    <td></td>
    <td><button class="button" type="button" onclick="RControl("1")"> &#8593; </button></td>
    <td></td>
  <td></td>
  <td></td>
  </tr>
  <tr>
      <td><button class="buttonth" type="button" onclick="RControl("t")">&#8592;&#8592;&#8592;</button></td>
      <td><button class="buttontm" type="button" onclick="RControl("r")">&#8592;&#8592;</button></td>
    <td><button class="buttont" type="button" onclick="RControl("4")">&#8592;</button></td>
    <td></td>
    <td><button class="buttont"  type="button" onclick="RControl("5")">&#8594;</button></td>
      <td><button class="buttontm" type="button" onclick="RControl("s")">&#8594;&#8594;</button></td>
        <td><button class="buttonth" type="button" onclick="RControl("z")">&#8594;&#8594;&#8594;</button></td>
  </tr>
    <tr>
   <td></td>
   <td></td>
    <td></td>
    <td><button class="button" type="button" onclick="RControl("6")"> &#8595; </button></td>
   <td></td>
   <td></td>
    <td></td>
  </tr>
    <tr>
   <td></td>
   <td></td>
    <td></td>
    <td><button class="buttonm" type="button" onclick="RControl("7")">&#8595;&#8595; </button></td>
   <td></td>
   <td></td>
    <td></td>
  </tr>
    <tr>
   <td></td>
   <td></td>
    <td></td>
    <td><button class="buttonh"  type="button" onclick="RControl("8")">&#8595;&#8595;&#8595;</button></td>
   <td></td>
   <td></td>
    <td></td>
  </tr>
</table>
<br>
<script>
var connection = new WebSocket('ws://192.168.4.1/ws');
connection.onopen = function () {
};
connection.onerror = function (error) {
  console.log('WebSocket Error ', error);
};
connection.onmessage = function (e) {
  console.log('Server: ', e.data);
};
connection.onclose = function () {
  console.log('WebSocket connection closed');
};

function RControl(number) {
  connection.send(number);
}
</script>
   </div>
    </div>  

<script>

function sendP() {
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("P");
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("P").placeholder =
      this.responseText;
    }
  };
  
  xhttp.open("GET", "getP?testo="+number.value, true);
  xhttp.send();
}
function sendI() {
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("I");
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("I").placeholder =
      this.responseText;
    }
  };
  
  xhttp.open("GET", "getI?testo="+number.value, true);
  xhttp.send();
}
function sendD() {
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("D");
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("D").placeholder =
      this.responseText;
    }
  };
  
  xhttp.open("GET", "getD?testo="+number.value, true);
  xhttp.send();
}

function sendMm() {
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("mm");
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("mm").placeholder =
      this.responseText;
    }
  };
  
  xhttp.open("GET", "getMm?testo="+number.value, true);
  xhttp.send();
}

function sendMM() {
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("mM");
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("mM").placeholder =
      this.responseText;
    }
  };
  
  xhttp.open("GET", "getMM?testo="+number.value, true);
  xhttp.send();
}

function sendOffset(number) {
  var xhttp = new XMLHttpRequest();
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("offset").innerHTML =
      this.responseText;
    }
  };

  xhttp.open("GET", "getOffset?testo="+number, true);
  xhttp.send();
}

function sendTs() {
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("ts");

  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ts").placeholder =
      this.responseText;
    }
  };

  xhttp.open("GET", "getTurnSpeed?testo="+number.value, true);
  xhttp.send();
}

function setOTA(){
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("OTA");

    xhttp.open("GET", "toggleOTA?testo="+number.checked.toString(), true);
  xhttp.send();
}

function getDistance() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("distance").innerHTML =
      this.responseText;
    }
  };
  
  
  xhttp.open("GET", "getDistance", true);
  xhttp.send();
}

function sendZa() {
  var xhttp = new XMLHttpRequest();
  var number = document.getElementById("za");
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("za").placeholder =
      this.responseText;
    }
  };
  
  xhttp.open("GET", "getZA?testo="+number.value, true);
  xhttp.send();
}

function sendAD() {
  var xhttp = new XMLHttpRequest();
  
  xhttp.open("GET", "getAD", true);
  xhttp.send();
}

function resetESP() {
  var xhttp = new XMLHttpRequest();
  
  xhttp.open("GET", "getReset", true);
  xhttp.send();
}

function resetAndSave() {
  var xhttp = new XMLHttpRequest();
  
  xhttp.open("GET", "getSaveAndReset", true);
  xhttp.send();
}

function getLoadData() {
  var xhttp = new XMLHttpRequest();
  document.getElementById("bloc2").style.display = "none";
  document.getElementById("a").style.display = "none";
  
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var obj = JSON.parse(this.responseText);
      document.getElementById("P").placeholder = obj.p;
      document.getElementById("I").placeholder = obj.i;
      document.getElementById("D").placeholder = obj.d;
      document.getElementById("mm").placeholder = obj.minimumVoltage;      
      document.getElementById("mM").placeholder = obj.maximumVoltage;
      document.getElementById("za").placeholder = obj.zeroAngle;
      document.getElementById("ts").placeholder = obj.turnSpeed;
      document.getElementById("offset").innerHTML = obj.offset;                  
      document.getElementById("ResetReason").innerHTML = obj.error
    }
  };
  
  xhttp.open("GET", "getLoad", true);
  xhttp.send();
}


setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getData();
}, 2000); //2000mSeconds update rate


 
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ADCValue").innerHTML = this.responseText;
      
    }
  };
  xhttp.open("GET", "getB", true);
  xhttp.send();
}


function seeControl(){
  document.getElementById("bloc2").style.display = "block";
  document.getElementById("b").style.display = "none";
  document.getElementById("a").style.display = "block";
  document.getElementById("bloc").style.display = "none";
}

function seeConfiguration(){
  document.getElementById("bloc").style.display = "block";
  document.getElementById("a").style.display = "none";
  document.getElementById("b").style.display = "block";
  document.getElementById("bloc2").style.display = "none";
}

</script>

  <div id="footer"> 
      <h3>SELF BALANCING ROBOT</h3>

  </div>
</body>
</html>

)=====";

var ros;  // pointer to the ROSLIB.Ros object
var canvas;  // html canvas
var context;  // 2d context of the canvas
var pixel;  // a single red pixel to draw a single lidar scan
var transparency;  // a 50% opaque white image the size of the screen to fade old images

var lidar_listener;  // subscriber to /scan
var currentScan;  // object to hold the most recent lidar scan
var max_range;  // maximum range of the lidar scanner

var odom_listener;  // subscriber to /odom
var currentOdom;  // object to hold the most recent odometry data

var robotWidth;  // physical width of the robot
var robotLength;  // physical length of the robot
var scale;  // canvas scale from meters to pixels

init();
animate();

// init is called first to setup the visualization
function init() {
  canvas = document.getElementById('canvas');  // connect canvas with html canvas
  context = canvas.getContext('2d');  // get 2d context from canvas
  pixel = context.createImageData(1, 1);  // create a 1x1 single pixel image for drawing individual scans
  transparency = context.createImageData(canvas.width, canvas.height);  // create an image the size of the screen for use in fading old images
  context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2);  // Put (0, 0) in the center of the canvas
  context.transform(1, 0, 0, -1, 0, 0);  // flip so the y+ is up

  currentOdom = new OdomData();   // allocate memory for odom structure
  currentScan = new LidarScan();  // allocate memory for lidar structure

  max_range = 10;  //TODO: change this to be not a hard set value
  robotWidth = 20 * .0254;   // 20 inches wide * .0254 inches/meter
  robotLength = 26 * .0254;  // 26 inches long * .0254 inches/meter
  scale = (canvas.height / 2) / max_range;  // scale for pixels per meter

  // ImageData is [r, g, b, a]
  pixel.data[0] = 255;  // set pixel to 100% red g and b default to 0
  pixel.data[3] = 255;  // set pixel to 100% alpha (full opacity)
  
  transparency.data[0] = 255;  // set transparency to white #FFFFFF
  transparency.data[1] = 255;
  transparency.data[2] = 255;
  transparency.data[3] = 128;  // set transparency to 50% alpha (half transparent)

  document.getElementById("start").addEventListener("click", connect);   // connect start button with html button, set click listener to connect
  document.getElementById("stop").addEventListener("click", terminate);  // connect stop button with html button, set click to terminate
}

// connect is called to start the rosbridge connection between the js and ROS
function connect() {
  ros = new ROSLIB.Ros({
    url : 'ws://35.2.220.201:9092'  // ip address of the raspberry pi on the magicbot which runs on port 9092
  });

  // on successful connection to the websocket
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    subscribeToTopics();  // subscribes the rosbridge to /scan and /odom
  });

  // on error connecting to the websocket (i.e. if a roscore is not running)
  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  // on close of the websocket connection
  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });
}

// called to subscribe to /scan and /odom
function subscribeToTopics() {
  // Subscribe to /scan to receive lidar scans
  lidar_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan',
    throttle_rate : 10
  });
  // the following function is called everytime a message is received from /scan
  lidar_listener.subscribe(function(message) {
    // sets the individual pieces of currentScan from the most recent message
    currentScan.ranges = message.ranges;
    currentScan.angle_min = message.angle_min;
    currentScan.angle_increment = message.angle_increment;
  });

  // Subscribe to /odom to receive odometry data
  odom_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry',
  });
  // the following function is called evertime a message is received from /odom
  odom_listener.subscribe(function(message) {
    // set the individual pieces of currentOdom from the most recent message
    currentOdom.x = message.pose.pose.position.x;
    currentOdom.y = message.pose.pose.position.y;
    var o = message.pose.pose.orientation;  // sets a temp variable for the messages quaternion representing the orientation
    currentOdom.theta = getYaw(o.x, o.y, o.z, o.w);  // calculates the yaw of the robot
  });
}

// called to animate the scene
function animate() {
  requestAnimationFrame(animate);  // request that animate be called before the next repaint
  draw();  // draw the current state and observations of the robot
}

// called to draw the current state and observations of the robot
function draw() {
  context.putImageData(transparency, 0, 0);  // place the transparency image at (0, 0) to fade the previously drawn images

  // Draw the Lidar Data
  var x;  // temporary variable for the x value of an individual lidar range
  var y;  // temporary variable for the y value of an individual lidar range
  var angle = currentScan.angle_min;  // temporary variable for the angle of the current individual lidar range, starts at the minimum angle
  var imgData = context.createImageData(canvas.width, canvas.height);  // creates new canvas sized image data
  for(var i = 0; i < currentScan.ranges.length; i++) {
    x = (canvas.width / 2) + (scale * currentScan.ranges[i] * Math.cos(angle + currentOdom.theta));  // x coordinate of scan with transform
    y = (canvas.height / 2) - (scale * currentScan.ranges[i] * Math.sin(angle + currentOdom.theta)); // y coordinate of scan with transform
    //var index = 4 * Math.round(canvas.width * y + x);
    var index = 4 * (canvas.width * Math.round(y) + Math.round(x));
    imgData.data[index] = 255;  // sets index-th pixel's red (0) to 255
    imgData.data[index+3] = 255;  // sets index-th pixels's alpha (3) to 255
    angle += currentScan.angle_increment;  // increment the current angle by angle increment
  }
  context.putImageData(imgData, 0, 0);  // places the imgData pixel buffer at the top left corner

  // Draw the robot's previous path
  // TODO: store previous state's somehow so this can be done

  // Draw the robot which is represented as a circle with a radius oriented at the current angle to show direction
  var radius = scale * robotLength / 2;  // radius set to half the robot's length (bigger side to give buffer) divided by two times the scale
  context.strokeStyle = "#000000";  // set stroke to black
  context.fillStyle = "#FFFFFF";  // set fill to white
  context.beginPath();
  context.arc(0, 0, radius, 0, 2*Math.PI, false);  // draw an arc at (0, 0) with radius as defined above from 0 to 2PI (full circle)
  context.fill();  // fill the drawn circle with white
  context.stroke();  // outline the circle in black
  context.moveTo(0, 0);  // set cursor at (0, 0)
  context.lineTo(radius*Math.cos(currentOdom.theta), radius*Math.sin(currentOdom.theta));  // create line from cursor position to outer edge of circle
  context.stroke();  // stroke to draw line in black
}

// called on stop button clicked, closes rosbridge connection
function terminate() {
  ros.close();  // closes rosbridge websocket connection
}

// Define OdomData Structure
// -------------------------
function OdomData() {
  this.x = 0;
  this.y = 0;
  this.theta = 0;
}

// Define LidarScan Structure
// --------------------------
function LidarScan() {
  this.ranges = [];
  this.angle_min = 0;
  this.angle_increment = 0;
}

// Helper Functions
// ----------------

function distanceSquared(p1, p2) {
  var sum = 0;
  sum += Math.pow(p2[0] - p1[0], 2);
  sum += Math.pow(p2[1] - p1[1], 2);
  return sum;
}

// Obtained pseudo-code from:
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_from_Quaternion
function quaternionToEuler(x, y, z, w) {
  var t0 = 2 * (w*x + y*z);
  var t1 = 1 - 2 * (x*x + y*y);
  var roll = Math.atan2(t0, t1);

  var t2 = 2 * (w*y - z*x);
  t2 = ((t2 > 1) ? 1 : t2);
  t2 = ((t2 < -1) ? -1 : t2);
  var pitch = Math.asin(t2);

  var t3 = 2 * (w*z + x*y);
  var t4 = 1 - 2 * (y*y + z*z);
  var yaw = Math.atan2(t3, t4);

  return [roll, pitch, yaw];
}

function getYaw(x, y, z, w) {
  var t3 = 2 * (w*z + x*y);
  var t4 = 1 - 2 * (y*y + z*z);
  var yaw = Math.atan2(t3, t4);

  return yaw;
}
var ros;  // pointer to the ROSLIB.Ros object
var canvas;  // html canvas
var context;  // 2d context of the canvas

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
  context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2);  // Put (0, 0) in the center of the canvas
  context.transform(1, 0, 0, -1, 0, 0);  // flip so the y+ is up

  currentOdom = new OdomData();   // allocate memory for odom structure
  currentScan = new LidarScan();  // allocate memory for lidar structure

  max_range = 10;  //TODO: change this to be not a hard set value
  robotWidth = 20 * .0254;   // 20 inches wide * .0254 inches/meter
  robotLength = 26 * .0254;  // 26 inches long * .0254 inches/meter
  scale = (canvas.height / 2) / max_range;  // scale for pixels per meter

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
    lidar_listener.unsubscribe();
    odom_listener.unsubscribe();
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
    throttle_rate : 100,  // messages throttled to a minimum of 100 millis between messages
    queue_length : 0,
    queue_size : 1,
    buff_size : 2**13  // buff_size is 2^13 bytes because my estimated size of a laser scan message is more than 2^12, but not yet 2^13
                       // my estimate comes from http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html note: consistently 1081 items in ranges
                       // idea for setting buff_size comes from https://github.com/ros/ros_comm/issues/536
  });
  // the following function is called everytime a message is received from /scan
  lidar_listener.subscribe(function(message) {
    // sets the individual pieces of currentScan from the most recent message
    for(i = 0; i < message.ranges.length; i++) {
      currentScan.ranges[i] = message.ranges[i];
    }
    currentScan.angle_min = message.angle_min;
    currentScan.angle_increment = message.angle_increment;
  });
  // Subscribe to /odom to receive odometry data
  odom_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry',
    queue_length : 0,
    queue_size : 1,
    buff_size : 2**10  // buff_size is 2^10 bytes because my estimated size of an odometry message is more than 2^9, but less than 2^10
                       // my estimate comes from http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
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
  context.fillStyle = "rgba(255, 255, 255, 0.3)";  // set fill style to 30% transparent white for fade
  context.fillRect(canvas.width / -2, canvas.height / -2, canvas.width, canvas.height);  //  fill the entire canvas with transparent white to fade old points

  var x;  // temporary variable for the x value of an individual lidar range
  var y;  // temporary variable for the y value of an individual lidar range
  var angle = currentScan.angle_min;  // temporary variable for the angle of the current individual lidar range, starts at the minimum angle
  for(var i = 0; i < currentScan.ranges.length; i++) {
    x = scale * currentScan.ranges[i] * Math.cos(angle + currentOdom.theta);  // find x from the current range and angle
    y = scale * currentScan.ranges[i] * Math.sin(angle + currentOdom.theta);  // find y from the current range and angle

    context.fillStyle = "#FF0000";  // set fill style to red
    context.fillRect(x, y, 1, 1);  // draw a 1x1 pixel at (x, y)

    angle += currentScan.angle_increment;  // increment the current angle by angle increment
  }

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

// caluclates the square of the distance between 2 points
function distanceSquared(p1, p2) {
  var sum = 0;
  sum += Math.pow(p2[0] - p1[0], 2);
  sum += Math.pow(p2[1] - p1[1], 2);
  return sum;
}

/*  Roll and Pitch are unused so I extracted the yaw portion as a seperate method
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
*/

// retrieves the euler yaw from the quaternion's x, y, z, w
function getYaw(x, y, z, w) {
  var t3 = 2 * (w*z + x*y);
  var t4 = 1 - 2 * (y*y + z*z);
  var yaw = Math.atan2(t3, t4);

  return yaw;
}
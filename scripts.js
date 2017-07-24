/*
 * Odom_Viz3.2 | ZombieBot Controller and Visualizer
 *
 * Lidar scans and odometry data is visualized
 * Simultaenously a virtual joystick controls the motors
 * Communication handled through javascript RosBridge
 *
 * Brian May | Summer 2017 Internship
 * Laboratory4Progress
 */

var ros;  // pointer to the ROSLIB.Ros object
var canvas;  // html canvas
var context;  // 2d context of the canvas

var connectivity;  // html text displaying connectivity status
var mobile;  // boolean that describes whether or not the user is on a mobile device

var controls;  // html canvas for the virtual joystick
var rect;  // the rect defining the controls canvas
var conCtx;  // the 2d context of the controls canvas
var mouseDown;  // a variable for tracking when the mouse button is down

var lidar_listener;  // subscriber to /scan
var currentScan;  // object to hold the most recent lidar scan
var max_range;  // maximum range of the lidar scanner

var odom_listener;  // subscriber to /odom
var currentOdom;  // object to hold the most recent odometry data

var velocity_publisher;  // publisher to /cmd_vel
var publish_counter;  // counter for when velocities should be published to /cmd_vel
var publish_freq;  // how many animation loops should pass before another velocity message is published
var publish_ready;
var twist;  // twist message for publishing current velocity

var robotWidth;  // physical width of the robot
var robotLength;  // physical length of the robot 
var joystick_radius;
var radius;  // the radius of the circle to draw that depicts the robot
var scale;  // canvas scale from meters to pixels
var states;  // linked list for storing a set amount of previous states of the robot

init();
animate();

// init is called first to setup the visualization
function init() {
  canvas = document.getElementById('canvas');  // connect canvas with html canvas
  context = canvas.getContext('2d');  // get 2d context from canvas
  canvas.width = window.innerWidth;  // set the canvas to the width of the window (full page)
  canvas.height = window.innerHeight;  // set the canvas to the height of the window (full page)
  context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2);  // Put (0, 0) in the center of the canvas
  context.transform(1, 0, 0, -1, 0, 0);  // flip so the y+ is up
  
  connectivity = document.getElementById("connectivity");
  mobile = navigator.userAgent.match(/(iPhone|Android|IEMobile)/);

  controls = document.getElementById('controls');  // connect controls canvas with html
  conCtx = controls.getContext('2d');  // get 2d context from controls
  controls.width = window.innerWidth;  // set the controls canvas to the width of the window (full page)
  controls.height = window.innerHeight;  // set the controls canvas to the height of the window (full page)
  conCtx.strokeStyle = "#0000FF";  // set controls stroke style to this color

  controls.addEventListener("mousedown", stickDown);  // add a listener to the controls canvas for mousedown events
  controls.addEventListener("mouseup", stickUp);  // add a listener to the controls canvas for mouseup events
  controls.addEventListener("mousemove", movement);  // add a listener to the controls canvas for mousemove events
  controls.addEventListener("touchstart", stickDownT);  // handles touch events on the controls canvas
  controls.addEventListener("touchend", stickUp);  // adds a listener for when touches are removed
  controls.addEventListener("touchmove", movementT);  // adds a listener for when touches move
  mouseDown = null;

  publish_ready = false;
  publish_counter = 0;
  publish_freq = 10;  // every 10 draw frames, a new velocity message is published
  twist = new ROSLIB.Message({  // creates a new twist message for publishing current velocity
    linear : {
      x : 0,  // all values 0 so that the bot doesn't take off on startup
      y : 0,
      z : 0
    },
    angular : {
      x : 0,  // all values 0 so that the bot doesn't spin on startup
      y : 0,
      z : 0
    }
  });

  currentOdom = new OdomData();   // allocate memory for odom structure
  currentScan = new LidarScan();  // allocate memory for lidar structure
  states = new World();  // allocate memory for the world structure
  states.append(currentScan, currentOdom);  // append the default odom and lidar data to avoid null errors

  max_range = 10;  //TODO: change this to be not a hard set value
  robotWidth = 20 * .0254;   // 20 inches wide * .0254 inches/meter
  robotLength = 26 * .0254;  // 26 inches long * .0254 inches/meter
  scale = (canvas.height / 2) / max_range;  // scale for pixels per meter
  joystick_radius = mobile ? 200 : 100;

  document.getElementById("start").addEventListener("click", connect);   // connect start button with html button, set click listener to connect
  document.getElementById("stop").addEventListener("click", terminate);  // connect stop button with html button, set click to terminate
  //document.getElementById("start").addEventListener("touch", connect);
  //document.getElementById("stop").addEventListener("touch", terminate);
}

// connect is called to start the rosbridge connection between the js and ROS
function connect() {
  connectivity.innerHTML = "Connecting...";
  var loc = mobile ? 'ws://192.168.42.1:9092' : 'ws://zombie.local:9092';
  ros = new ROSLIB.Ros({
    url : loc  // local host of the raspberry pi access point, rosbridge connects via port 9092
  });

  // on successful connection to the websocket
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    connectivity.innerHTML = "Connected";  // display connection status
    connectivity.style.color = "#00FF00";  // make text green
    subscribeToTopics();  // subscribes the rosbridge to /scan and /odom
  });

  // on error connecting to the websocket (i.e. if a roscore is not running)
  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    connectivity.innerHTML = "Error Connecting";  // display connection status
    connectivity.style.color = "#FF0000";  // make text red
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
    throttle_rate : 500,  // messages throttled to a minimum of 500 millis between messages
    queue_size : 1
  });

  // the following function is called everytime a message is received from /scan
  lidar_listener.subscribe(function(message) {
    currentScan = JSON.parse(JSON.stringify(message));  // deep copy's message into currentScan (stupid way of deep copying but it is the best js offers)
    currentScan.angle_max = null;  // set unnecessary properties to null to conserve memory
    currentScan.time_increment = null;
    currentScan.scan_time = null;
    currentScan.range_min = null;
    currentScan.range_max = null;
    currentScan.intensities = null;
    message = null;  // set message to null so that it can be garbage collected
  });

  // Subscribe to /odom to receive odometry data
  odom_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry',
    queue_size : 1
  });

  // the following function is called evertime a message is received from /odom
  odom_listener.subscribe(function(message) {
    // set the individual pieces of currentOdom from the most recent message
    currentOdom = JSON.parse(JSON.stringify(message));  // deep copies the incoming message
    currentOdom.x = currentOdom.pose.pose.position.x;  // sets the x position to an easier to access variable
    currentOdom.y = currentOdom.pose.pose.position.y;  // sets the y position to an easier to access variable
    var o = currentOdom.pose.pose.orientation;  // sets a temp variable for the messages quaternion representing the orientation
    currentOdom.theta = getYaw(o.x, o.y, o.z, o.w);  // calculates the yaw of the robot
    message = null;  // set message to null so that it can be garbage collected

    // append the current state to the states variable if it has moved or turned more than a set threshold
    if(distanceSquared([states.tail.odom.x, states.tail.odom.y], [currentOdom.x, currentOdom.y]) > .04 || 
    (Math.abs(states.tail.odom.theta - currentOdom.theta) > 0.03)) {  // run if bot has moved a specified distance or turned TODO: find ideal values for thresholds
      var tempLidar = JSON.parse(JSON.stringify(currentScan));  // deep copy currentScan (not a pretty way to deep copy, but js offers nothing better)
      var tempOdom = JSON.parse(JSON.stringify(currentOdom));  // deep copy currentOdom
      states.append(tempLidar, tempOdom);  // add current scan and odom to the world linked list
    }
  });

  // Publisher Initialization
  // ------------------------
  velocity_publisher = new ROSLIB.Topic({
    ros : ros,
    name : 'cmd_vel',
    messageType : 'geometry_msgs/Twist',
    queue_size : 1  // set queue_size to 1 so that the topic does not accumulate old messages
  });
  publish_ready = true;
}

// called to animate the scene
function animate() {
  requestAnimationFrame(animate);  // request that animate be called before the next repaint
  draw();  // draw the current state and observations of the robot
}

// called to draw the current state and observations of the robot
function draw() {
  // clear/fade previous drawings
  context.fillStyle = "#FFFFFF";  // set fill style to opaque white for clear
  //context.fillStyle = "rgba(255, 255, 255, .3)";  // set fill style to 30% transparent white for fade
  context.fillRect(canvas.width / -2, canvas.height / -2, canvas.width, canvas.height);  //  fill the entire canvas with transparent white to fade old points
  
  // TODO: draw old lidar scans at their appropriate location

  // Draw the current lidar scan
  context.fillStyle = "#FF0000";  // set fill style to red
  drawLidar(currentScan.ranges, currentScan.angle_min, currentScan.angle_increment, currentOdom.theta);

  // Draw the robot's previous path
  context.strokeStyle = "#00FF00";  // set stroke style to this color
  context.beginPath();
  pointer = states.head;  // point at states head
  context.moveTo(scale*(pointer.odom.x - currentOdom.x), scale*(pointer.odom.y - currentOdom.y));  // move to the oldest position relative to the robot
  if(pointer != null) pointer = pointer.next;  // advance the pointer if appropriate
  while(pointer != null) {  // loop through entire linked list
    context.lineTo(scale*(pointer.odom.x - currentOdom.x), scale*(pointer.odom.y - currentOdom.y));  // draw a line to the next (x, y) position relative to robot
    pointer = pointer.next;  // advance the pointer
  }
  context.stroke();  // stroke to draw the line
  
  // Draw the robot which is represented as a circle with a radius oriented at the current angle to show direction
  radius = scale * robotLength / 2;  // radius set to half the robot's length (bigger side to give buffer) divided by two times the scale
  context.strokeStyle = "#000000";  // set stroke to black
  context.fillStyle = "#FFFFFF";  // set fill to white
  context.beginPath();
  context.arc(0, 0, radius, 0, 2*Math.PI, false);  // draw an arc at (0, 0) with radius as defined above from 0 to 2PI (full circle)
  context.fill();  // fill the drawn circle with white
  context.stroke();  // outline the circle in black
  context.moveTo(0, 0);  // set cursor at (0, 0)
  context.lineTo(radius*Math.cos(currentOdom.theta), radius*Math.sin(currentOdom.theta));  // create line from cursor position to outer edge of circle
  context.stroke();  // stroke to draw line in black

  publish_counter++;  // increment publish counter
  if(publish_ready && publish_counter >= publish_freq) {  // if the publish frequency has been met and the previous publish has been completed
    velocity_publisher.publish(twist);  // publish the current twist message to /cmd_vel
    publish_counter = 0;  // reset publish counter
  }
}

// used to draw the ranges of a lidar scan, method extracted to avoid near duplicate code
function drawLidar(ranges, angle, increment, robot_angle) {
  var x;  // temporary variable for the x value of an individual lidar range
  var y;  // temporary variable for the y value of an individual lidar range
  for(var i = 0; i < ranges.length; i++) {  // loop through all ranges in array
    x = scale * ranges[i] * Math.cos(angle + robot_angle);  // find x from the range and angle
    y = scale * ranges[i] * Math.sin(angle + robot_angle);  // find y from the range and angle

    context.fillRect(x, y, 1, 1);  // draw a 1x1 pixel at (x, y)

    angle += increment;  // increment the angle by the angle increment
  }
}

// called on stop button clicked, closes rosbridge connection
function terminate() {
  ros.close();  // closes rosbridge websocket connection
  connectivity.innerHTML = "Not Connected";  // display connection status
  connectivity.style.color = "#000000";  // make text black
}

// Controller Methods
// ------------------
// called when the mouse button is pressed
function stickDown(event) {
  event.preventDefault();
  rect = controls.getBoundingClientRect();  // retrieves the current rect defining the controls canvas
  mouseDown = {x : event.clientX - rect.left, y : event.clientY - rect.top};  // sets mouseDown to [x, y] of the click relative to top left of controls canvas
  drawStick();  // draws the bounding circle without a joystick circle
}

// called when the mouse button is released
function stickUp() {
  mouseDown = null;  // resets the mouseDown variable to nulll
  conCtx.clearRect(0, 0, controls.width, controls.height);  // clears all current drawings on the controls canvas
  publishVelocity(0, 0);  // publishes 0 linear and 0 angular velocities to stop the robot
}

// called when the mouse button is moved within the controls canvas
function movement(event) {
  event.preventDefault();  // avoids moving the mouse or touch from selecting elements on the screen
  if(mouseDown != null) {  // only if the mouse button is currently pressed
    var relX = event.clientX - rect.left - mouseDown.x;  // the x location relative to where the button was originally clicked (max 100)
    var relY = mouseDown.y - (event.clientY - rect.top);  // the y location relative to where the button was originally clicked (max 100)
    if(distanceSquared([0, 0], [relX, relY]) > joystick_radius*joystick_radius) {  // if the distance from the mouse position to the original click location is greater than 100:
      var arctan = Math.atan2(relY, relX);  // finds the angle that the mouse is at
      relX = joystick_radius * Math.cos(arctan);  // sets the x to the maximum radius at the correct angle
      relY = joystick_radius * Math.sin(arctan);  // sets the y to the maximum radius at the correct angle
    }
    drawStick(relX, relY);  // draws the bounding circle with a joystick circle at the current mouse location
    publishVelocity(relY/joystick_radius, relX/joystick_radius);  // publishes a linear velocity correlated with the y position of the joystick and angular correlated with x
  }
}

// same method as stickDown() but handles touch events
function stickDownT(event) {
  event.preventDefault();
  rect = controls.getBoundingClientRect();  // retrieves the current rect defining the controls canvas
  mouseDown = {x : event.touhes[0].clientX - rect.left, y : event.touches[0].clientY - rect.top};  // sets mouseDown to [x, y] of the click relative to top left of controls canvas
  drawStick();  // draws the bounding circle without a joystick circle
}

// same method as movement() but handles touch events
function movementT(event) {
  event.preventDefault();
  if(mouseDown != null) {  // only if the mouse button is currently pressed
    var relX = event.touches[0].clientX - rect.left - mouseDown.x;  // the x location relative to where the button was originally clicked (max 100)
    var relY = mouseDown.y - (event.touches[0].clientY - rect.top);  // the y location relative to where the button was originally clicked (max 100)
    if(distanceSquared([0, 0], [relX, relY]) > joystick_radius*joystick_radius) {  // if the distance from the mouse position to the original click location is greater than 100:
      var arctan = Math.atan2(relY, relX);  // finds the angle that the mouse is at
      relX = joystick_radius * Math.cos(arctan);  // sets the x to the maximum radius at the correct angle
      relY = joystick_radius * Math.sin(arctan);  // sets the y to the maximum radius at the correct angle
    }
    drawStick(relX, relY);  // draws the bounding circle with a joystick circle at the current mouse location
    publishVelocity(relY/joystick_radius, relX/joystick_radius);  // publishes a linear velocity correlated with the y position of the joystick and angular correlated with x
  }
}

// draws the virtual joystick, parameter mousePos is an array either empty to not draw joystick or [x, y] to draw joystick
function drawStick(mousePosX, mousePosY) {
  conCtx.clearRect(0, 0, controls.width, controls.height);  // clears old drawings
  conCtx.beginPath();
  conCtx.arc(mouseDown.x, mouseDown.y, joystick_radius, 0, 2 * Math.PI);  // draws a bounding circle where the button was originally pressed with radius 100
  conCtx.stroke();
  if(mousePosX != null || mousePosY != null) {  // if the mouse has been moved from the original location
    conCtx.beginPath();
    conCtx.arc(mousePosX + mouseDown.x, mouseDown.y - mousePosY, joystick_radius/10, 0, 2 * Math.PI);  // draws the joystick at the current mouse location
    conCtx.stroke();
  }
}

// sets the twist object to the current velocities to be published periodically
function publishVelocity(forward, turn) {
  twist = {  // sets values of the twist message
    linear : {
      x : forward * 2,  // sets linear x velocity to the forward parameter
      y : 0,  // never linear y velocity because the robot cannot strafe sideways
      z : 0  // never linear z velocity because the robot cannot jump or fly
    },
    angular : {
      x : 0,  // never angular x velocity because the robot has no roll
      y : 0,  // never angular y velocity because the robot has no pitch
      z : turn  // sets linear z, the yaw, to the turn parameter
    }
  };
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

// Define World Class
// ------------------
function World() {
  this.head = null;  // oldest element
  this.tail = null;  // most recent element
  this.length = 0;  // current length of the linked list
  this.max_length = 20;  // maximum length of the linked list

  this.append = function(l, o) {  // used to append new elements to the linked list
    const node = {lidar : l, odom : o};  // create a new node with the given attributes
    node.next = null;  // this is going on the end of the list so it has a null next attribute
    if(this.head == null && this.tail == null) {  // if the list is empty
      this.head = node;  // set head and tail to this element
      this.tail = node;
    } else {  // if the linked list has at least one element (is not empty)
      this.tail.next = node;  // link this node to the current tail
      this.tail = node;  // set the tail to point at this node
      this.length++;  // increment length
      if(this.length >= this.max_length) {  // if the append makes the length exceed the maximumum length
        this.head = this.head.next;  // advance the head pointer to chop off the oldest element (nothing points to it anymore)
        this.length--;  // decrement the length to account for the lost node
      }
    }
  }
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
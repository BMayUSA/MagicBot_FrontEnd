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

var ros;                    // pointer to the ROSLIB.Ros object
var canvas;                 // html canvas
var context;                // 2d context of the canvas

var connectivity;           // html text displaying connectivity status
var mobile;                 // boolean that describes whether or not the user is on a mobile device

var controls;               // html canvas for the virtual joystick (on top of other canvas)
var rect;                   // the rect defining the controls canvas
var conCtx;                 // the 2d context of the controls canvas
var mouseDown;              // a variable for tracking when the mouse button is down

var lidar_listener;         // subscriber to /scan
var currentScan;            // object to hold the most recent lidar scan
var max_range;              // maximum range of the lidar scanner

var odom_listener;          // subscriber to /odom
var currentOdom;            // object to hold the most recent odometry data

var velocity_publisher;     // publisher to /cmd_vel
var publish_counter;        // counter for when velocities should be published to /cmd_vel
var publish_freq;           // how many animation loops should pass before another velocity message is published
var publish_ready;          // boolean for when the initialization of the publisher is done
var twist;                  // twist message for setting and publishing current velocity

var robotWidth;             // physical width of the robot in meters
var robotLength;            // physical length of the robot in meters
var joystick_radius;        // the radius to be displayed for the virtual joystick
var radius;                 // the radius of the circle to draw that depicts the robot
var scale;                  // canvas scale from meters to pixels
var states;                 // linked list for storing a set amount of previous states of the robot

init();
animate();

// init is called first to set everything up
function init() {
  // initialize the main canvas
  canvas = document.getElementById('canvas');
  context = canvas.getContext('2d');                               // get 2d context from canvas
  canvas.width = window.innerWidth;                                // set the canvas to the width of the window (full page)
  canvas.height = window.innerHeight;                              // set the canvas to the height of the window (full page)
  context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2);  // Put (0, 0) in the center of the canvas
  context.transform(1, 0, 0, -1, 0, 0);                            // flip so the y+ is up

  // initialize the controls canvas
  controls = document.getElementById('controls');
  conCtx = controls.getContext('2d');                              // get 2d context from controls
  controls.width = window.innerWidth;                              // set the controls canvas to the width of the window (full page)
  controls.height = window.innerHeight;                            // set the controls canvas to the height of the window (full page)
  conCtx.strokeStyle = "#0000FF";

  connectivity = document.getElementById("connectivity");
  mobile = navigator.userAgent.match(/(iPhone|Android|IEMobile)/); // check to see if the user is on a mobile device

  // add event listeners for mouse clicks and touch events
  controls.addEventListener("mousedown", stickDown);
  controls.addEventListener("mouseup", stickUp);
  controls.addEventListener("mousemove", movement);
  controls.addEventListener("touchstart", stickDownT);
  controls.addEventListener("touchend", stickUp);
  controls.addEventListener("touchmove", movementT);
  mouseDown = null;

  // initialize the velocity publisher
  publish_ready = false;
  publish_counter = 0;
  publish_freq = 10;             // every 10 animate frames, a new velocity message is published
  twist = new ROSLIB.Message({   // creates a new twist message for publishing current velocity
    linear : {
      x : 0,     // all values 0 so that the bot doesn't take off on startup
      y : 0,
      z : 0
    },
    angular : {
      x : 0,     // all values 0 so that the bot doesn't spin on startup
      y : 0,
      z : 0
    }
  });

  currentOdom = new OdomData();             // allocate memory for odom structure
  currentScan = new LidarScan();            // allocate memory for lidar structure
  states = new World();                     // allocate memory for the world structure
  states.append(currentScan, currentOdom);  // append the default odom and lidar data to avoid null errors

  max_range = 10;                           //TODO: find actual value for this, 10 is an estimation that looked good
  robotWidth = 20 * .0254;                  // 20 inches wide * .0254 meters/inch
  robotLength = 26 * .0254;                 // 26 inches long * .0254 meters/inch
  scale = (canvas.height / 2) / max_range;  // scale for pixels per meter
  joystick_radius = mobile ? 200 : 100;     // mobile devices have a bigger joystick so it can be seen under fingers

  document.getElementById("start").addEventListener("click", connect);
  document.getElementById("stop").addEventListener("click", terminate);
}

// connect is called to start the rosbridge connection between the js and ROS
function connect() {
  connectivity.innerHTML = "Connecting...";
  var loc = mobile ? 'ws://192.168.42.1:9092' : 'ws://zombie.local:9092';  // mobile can't route through .local because it circles back
  ros = new ROSLIB.Ros({
    url : loc // local host of the raspberry pi access point, rosbridge connects via port 9092
  });

  // on successful connection to the websocket
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    connectivity.innerHTML = "Connected";
    connectivity.style.color = "#00FF00";
    subscribeToTopics();  // subscribes the rosbridge to /scan and /odom and /cmd_vel
  });

  // on error connecting to the websocket (i.e. if a roscore is not running)
  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    connectivity.innerHTML = "Error Connecting";
    connectivity.style.color = "#FF0000";
  });

  // on close of the websocket connection
  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });
}

// called to subscribe to /scan and /odom and /cmd_vel
function subscribeToTopics() {
  // Subscribe to /scan to receive lidar scans
  lidar_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan', 
    messageType : 'sensor_msgs/LaserScan',
    throttle_rate : 100,  // messages throttled to a minimum numbe of millis between messages
    queue_size : 1
  });

  // the following function is called everytime a message is received from /scan
  lidar_listener.subscribe(function(message) {
    currentScan = JSON.parse(JSON.stringify(message));  // deep copy's message into currentScan (stupid way of deep copying but it is the best js offers)
    currentScan.angle_max = null;                       // set unnecessary properties to null to conserve memory
    currentScan.time_increment = null;
    currentScan.scan_time = null;
    currentScan.range_min = null;
    currentScan.range_max = null;
    currentScan.intensities = null;
    message = null;                                     // set message to null so that it can be garbage collected
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
    currentOdom.x = currentOdom.pose.pose.position.x;   // sets the x position to an easier accessible variable
    currentOdom.y = currentOdom.pose.pose.position.y;   // sets the y position to an easier accessible variable
    var o = currentOdom.pose.pose.orientation;          // temp variable for the messages quaternion representing the orientation
    currentOdom.theta = getYaw(o.x, o.y, o.z, o.w);
    message = null;                                     // set message to null so that it can be garbage collected

    // append the current state to the states variable if it has moved or turned more than a set threshold
    if(distanceSquared([states.tail.odom.x, states.tail.odom.y], [currentOdom.x, currentOdom.y]) > .04 || 
    (Math.abs(states.tail.odom.theta - currentOdom.theta) > 0.03)) {  // TODO: find ideal values for thresholds
      var tempLidar = JSON.parse(JSON.stringify(currentScan));        // deep copy currentScan (not a pretty way to deep copy, but js offers nothing better)
      var tempOdom = JSON.parse(JSON.stringify(currentOdom));         // deep copy currentOdom
      states.append(tempLidar, tempOdom);
    }
  });

  // Publisher Initialization
  // ------------------------
  velocity_publisher = new ROSLIB.Topic({
    ros : ros,
    name : 'cmd_vel',
    messageType : 'geometry_msgs/Twist',
    queue_size : 1
  });
  publish_ready = true;
}

// called to animate the scene
function animate() {
  requestAnimationFrame(animate);  // request that animate be called before the next repaint
  draw();
}

// called to draw the current state and observations of the robot
function draw() {
  // clear previous drawings
  context.fillStyle = "#FFFFFF";
  context.fillRect(canvas.width / -2, canvas.height / -2, canvas.width, canvas.height);
  
  // TODO: draw old lidar scans at their appropriate location?

  // Draw the current lidar scan
  context.fillStyle = "#FF0000";
  drawLidar(currentScan.ranges, currentScan.angle_min, currentScan.angle_increment, currentOdom.theta);

  // Draw the robot's previous path
  context.strokeStyle = "#00FF00";
  context.beginPath();
  var pointer = states.head;                   // temp pointer for looping through states
  context.moveTo(scale*(pointer.odom.x - currentOdom.x), scale*(pointer.odom.y - currentOdom.y));  // move to the oldest position relative to the robot
  if(pointer != null) pointer = pointer.next;  // advance the pointer if appropriate
  while(pointer != null) {                     // loop through entire linked list
    context.lineTo(scale*(pointer.odom.x - currentOdom.x), scale*(pointer.odom.y - currentOdom.y));
    pointer = pointer.next;                    // advance the pointer
  }
  context.stroke();
  
  // Draw the robot which is represented as a circle with a radius oriented at the current angle to show direction
  radius = scale * robotLength / 2;  // radius set to half the robot's length (bigger side to give buffer) divided by two times the scale
  context.strokeStyle = "#000000";
  context.fillStyle = "#FFFFFF";
  context.beginPath();
  context.arc(0, 0, radius, 0, 2*Math.PI, false);
  context.fill();
  context.stroke();
  context.moveTo(0, 0);
  context.lineTo(radius*Math.cos(currentOdom.theta), radius*Math.sin(currentOdom.theta));
  context.stroke();

  // handle velocity publisher
  publish_counter++;
  if(publish_ready && publish_counter >= publish_freq) {
    velocity_publisher.publish(twist);
    publish_counter = 0;
  }
}

// used to draw the ranges of a lidar scan, method extracted to avoid near duplicate code
function drawLidar(ranges, angle, increment, robot_angle) {
  var x;  // temp x variable
  var y;  // temp y variable
  for(var i = 0; i < ranges.length; i++) {
    x = scale * ranges[i] * Math.cos(angle + robot_angle);  // find x from the range and angle
    y = scale * ranges[i] * Math.sin(angle + robot_angle);  // find y from the range and angle

    context.fillRect(x, y, 1, 1);  // draw a 1x1 pixel at (x, y)

    angle += increment;
  }
}

// called on stop button clicked, closes rosbridge connection
function terminate() {
  ros.close();                               // closes rosbridge websocket connection
  currentScan = new LidarScan();             // reset data structures
  currentOdom = new OdomData();
  states = new World();
  states.append(currentScan, currentOdom);
  connectivity.innerHTML = "Not Connected";
  connectivity.style.color = "#000000";
}

// Controller Methods
// ------------------
// called when the mouse button is pressed
function stickDown(event) {
  event.preventDefault();
  rect = controls.getBoundingClientRect();                                    // retrieves the current rect defining the controls canvas
  mouseDown = {x : event.clientX - rect.left, y : event.clientY - rect.top};  // sets mouseDown to (x, y) of the click relative to top left of controls canvas
  drawStick();                                                                // draws the bounding circle without a joystick circle b/c no parameters included
}

// called when the mouse button is released
function stickUp() {
  mouseDown = null;
  conCtx.clearRect(0, 0, controls.width, controls.height);  // clears all current drawings on the controls canvas
  publishVelocity(0, 0);                                    // stops the robot
}

// called when the mouse button is moved within the controls canvas
function movement(event) {
  event.preventDefault();  // avoids moving the mouse or touch from selecting elements on the screen
  if(mouseDown != null) {  // only if the mouse button is currently pressed
    var relX = event.clientX - rect.left - mouseDown.x;   // the x location relative to where the button was originally clicked
    var relY = mouseDown.y - (event.clientY - rect.top);  // the y location relative to where the button was originally clicked
    if(distanceSquared([0, 0], [relX, relY]) > joystick_radius*joystick_radius) {  // if the current mouse position is outside of the joystick:
      var arctan = Math.atan2(relY, relX);                                         // finds the angle that the mouse is at in the typical fashion
      relX = joystick_radius * Math.cos(arctan);                                   // sets the x to the maximum radius at the correct angle
      relY = joystick_radius * Math.sin(arctan);                                   // sets the y to the maximum radius at the correct angle
    }
    drawStick(relX, relY);                                        // draws the bounding circle with a joystick circle at the current mouse location
    publishVelocity(relY/joystick_radius, relX/joystick_radius);  // publishes a linear velocity correlated with the y position of the joystick and angular correlated with x
  }
}

// same method as stickDown() but handles touch events
function stickDownT(event) {
  event.preventDefault();
  rect = controls.getBoundingClientRect();                                                         // retrieves the current rect defining the controls canvas
  mouseDown = {x : event.touches[0].clientX - rect.left, y : event.touches[0].clientY - rect.top};  // sets mouseDown to (x, y) of the click relative to top left of controls canvas
  drawStick();                                                                                     // draws the bounding circle without a joystick circle
}

// same method as movement() but handles touch events
function movementT(event) {
  event.preventDefault();  // prevents moving touch points from selecting elements on screen
  if(mouseDown != null) {  // only if screen is being touched is currently pressed
    var relX = event.touches[0].clientX - rect.left - mouseDown.x;   // the x location relative to where the screen was originally touched
    var relY = mouseDown.y - (event.touches[0].clientY - rect.top);  // the y location relative to where the screen was originally touched
    if(distanceSquared([0, 0], [relX, relY]) > joystick_radius*joystick_radius) {  // if the touch has moved outside the joystick:
      var arctan = Math.atan2(relY, relX);                                         // finds the angle that the touch is at in typical fashion
      relX = joystick_radius * Math.cos(arctan);                                   // sets the x to the maximum radius at the correct angle
      relY = joystick_radius * Math.sin(arctan);                                   // sets the y to the maximum radius at the correct angle
    }
    drawStick(relX, relY);                                        // draws the bounding circle with a joystick circle at the current mouse location
    publishVelocity(relY/joystick_radius, relX/joystick_radius);  // publishes a linear velocity correlated with the y position of the joystick and angular correlated with x
  }
}

// draws the virtual joystick
function drawStick(mousePosX, mousePosY) {
  conCtx.clearRect(0, 0, controls.width, controls.height);
  conCtx.beginPath();
  conCtx.arc(mouseDown.x, mouseDown.y, joystick_radius, 0, 2 * Math.PI);
  conCtx.stroke();
  if(mousePosX != null || mousePosY != null) {  // if the mouse has been moved from the original location
    conCtx.beginPath();
    conCtx.arc(mousePosX + mouseDown.x, mouseDown.y - mousePosY, joystick_radius/10, 0, 2 * Math.PI);
    conCtx.stroke();
  }
}

// sets the twist object to the current velocities to be published periodically
function publishVelocity(forward, turn) {
  twist = {             // sets values of the twist message
    linear : {
      x : forward,  // sets linear x velocity to the forward parameter
      y : 0,            // never linear y velocity because the robot cannot strafe sideways
      z : 0             // never linear z velocity because the robot cannot jump or fly
    },
    angular : {
      x : 0,            // never angular x velocity because the robot has no roll
      y : 0,            // never angular y velocity because the robot has no pitch
      z : turn          // sets linear z, the yaw, to the turn parameter
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
  this.head = null;                               // oldest element
  this.tail = null;                               // most recent element
  this.length = 0;                                // current length of the linked list
  this.max_length = 20;                           // maximum length of the linked list (maybe should be set in init()?)

  this.append = function(l, o) {                  // used to append new elements to the linked list
    const node = {lidar : l, odom : o};           // create a new node with the given attributes
    node.next = null;                             // makes sure a while loop will stop at this node
    if(this.head == null && this.tail == null) {  // if the list is empty
      this.head = node;                           // set head and tail to this element
      this.tail = node;
    } else {                                      // if the linked list has at least one element (is not empty)
      this.tail.next = node;                      // link this node to the current tail
      this.tail = node;                           // set the tail to point at this node
      this.length++;
      if(this.length >= this.max_length) {        // if the append makes the length exceed the maximumum length
        this.head = this.head.next;               // advance the head pointer to chop off the oldest element (nothing points to it anymore)
        this.length--;
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
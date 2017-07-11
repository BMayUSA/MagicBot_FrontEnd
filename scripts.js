var ros;  // pointer to the ROSLIB.Ros object
var canvas;  // html canvas
var context;  // 2d context of the canvas

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
var twist;  // twist message for publishing current velocity

var robotWidth;  // physical width of the robot
var robotLength;  // physical length of the robot 
var scale;  // canvas scale from meters to pixels
var states;  // linked list for storing up to 10 previous states of the robot

init();
animate();

// init is called first to setup the visualization
function init() {
  canvas = document.getElementById('canvas');  // connect canvas with html canvas
  context = canvas.getContext('2d');  // get 2d context from canvas
  context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2);  // Put (0, 0) in the center of the canvas
  context.transform(1, 0, 0, -1, 0, 0);  // flip so the y+ is up

  controls = document.getElementById('controls');  // connect controls canvas with html
  conCtx = controls.getContext('2d');  // get 2d context from controls
  conCtx.strokeStyle = "#0000FF";  // set controls stroke style to this color

  controls.addEventListener("mousedown", stickDown);  // add a listener to the controls canvas for mousedown events
  controls.addEventListener("mouseup", stickUp);  // add a listener to the controls canvas for mouseup events
  controls.addEventListener("mousemove", movement);  // add a listener to the controls canvas for mousemove events
  mouseDown = null;

  publish_freq = 10;
  twist = new ROSLIB.Message({  // creates a new twist message for publishing current velocity
    linear : {
      x : 0,  // all values 0 so that the bot doesn't take off on startup
      y : 0,
      z : 0
    },
    angular : {
      x : 0,  // all values 0 so that the bot doesn't start a tornado on startup
      y : 0,
      z : 0
    }
  })

  currentOdom = new OdomData();   // allocate memory for odom structure
  currentScan = new LidarScan();  // allocate memory for lidar structure
  states = new World();  // allocate memory for the world structure
  states.append(new State(currentScan, currentOdom));  // append the default odom and lidar data to avoid null errors

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
    // Note: unsubscribe here instead of in terminate() so that in case of error termination subscribers are still unsubscribed
    lidar_listener.unsubscribe();  // unsubscribe from lidar subscriber
    odom_listener.unsubscribe();  // unsubscribe from odom subscriber
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
    for(i = 0; i < message.ranges.length; i++) {  // deep copy of ranges so that message can be garbage collected
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

    // append the current state to the states variable if it has moved or turned more than a set threshold
    if(distanceSquared([states.head.odom.x, states.head.odom.y], [currentOdom.x, currentOdom.y]) > 2 ||
       Math.abs(states.head.odom.theta - currentOdom.theta) > 0.03) {  // TODO: find ideal values for thresholds
      states.append(new State(currentScan, currentOdom));  // add current scan and odom to the world linked list in the form of a State
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
}

// called to animate the scene
function animate() {
  requestAnimationFrame(animate);  // request that animate be called before the next repaint
  draw();  // draw the current state and observations of the robot
  publish_counter++;  // increment publish counter
  if(publish_counter % publish_freq == 0) {
    velocity_publisher.publish(twist);  // publish the current twist message to /cmd_vel
  }
}

// called to draw the current state and observations of the robot
function draw() {
  // clear/fade previous drawings
  context.fillStyle = "#FFFFFF";  // set fill style to opaque white for clear
  //context.fillStyle = "rgba(255, 255, 255, .3)";  // set fill style to 30% transparent white for fade
  context.fillRect(canvas.width / -2, canvas.height / -2, canvas.width, canvas.height);  //  fill the entire canvas with transparent white to fade old points

  // Draw the past lidar scans
  context.fillStyle = "#00FFF0";  // set fill style to this color
  var pointer = states.head;  // variable pointing at the current location in the states linked list
  // loop to go through entire linked list
  while(pointer != null) {
    drawLidar(pointer.lidar.ranges, pointer.lidar.angle_min, pointer.lidar.angle_increment, pointer.odom.theta);
    pointer = pointer.next;  // advance the pointer along the linked list
  }

  // Draw the current lidar scan
  context.fillStyle = "#FF0000";  // set fill style to red
  drawLidar(currentScan.ranges, currentScan.angle_min, currentScan.angle_increment, currentOdom.theta);

  // Draw the robot's previous path
  context.strokeStyle = "#00FF00";  // set stroke style to this color
  context.beginPath();
  context.moveTo(0, 0);  // move context cursor to (0, 0), the position that the robot will be drawn at
  pointer = states.head;  // point at states head
  while(pointer != null) {  // loop through entire linked list
    context.lineTo(pointer.odom.x, pointer.odom.y);  // draw a line to the next (x, y) position
    pointer = pointer.next;  // advance the pointer
  }
  context.stroke();  // stroke to draw the line

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
}

// Controller Methods
// ------------------
// called when the mouse button is pressed
function stickDown(event) {
  rect = controls.getBoundingClientRect();  // retrieves the current rect defining the controls canvas
  mouseDown = [event.clientX - rect.left, event.clientY - rect.top];  // sets mouseDown to [x, y] of the click relative to top left of controls canvas
  drawStick([]);  // draws the bounding circle without a joystick circle
}

// called when the mouse button is released
function stickUp() {
  mouseDown = null;  // resets the mouseDown variable to nulll
  conCtx.clearRect(0, 0, controls.width, controls.height);  // clears all current drawings on the controls canvas
  publishVelocity(0, 0);  // publishes 0 linear and 0 angular velocities to stop the robot
}

// called when the mouse button is moved within the controls canvas
function movement(event) {
  if(mouseDown != null) {  // only if the mouse button is currently pressed
    var relX = event.clientX - rect.left - mouseDown[0];  // the x location relative to where the button was originally clicked (max 100)
    var relY = mouseDown[1] - (event.clientY - rect.top);  // the y location relative to where the button was originally clicked (max 100)
    if(distanceSquared([0, 0], [relX, relY]) > 10000) {  // if the distance from the mouse position to the original click location is greater than 100:
      var arctan = Math.atan2(relY, relX);  // finds the angle that the mouse is at
      relX = 100 * Math.cos(arctan);  // sets the x to the maximum radius at the correct angle
      relY = 100 * Math.sin(arctan);  // sets the y to the maximum radius at the correct angle
    }
    drawStick([relX, relY]);  // draws the bounding circle with a joystick circle at the current mouse location
    publishVelocity(relY/100, relX/100);  // publishes a linear velocity correlated with the y position of the joystick and angular correlated with x
    //console.log(Math.round(Math.sqrt(distanceSquared([0, 0], [relX, relY]))) + "% @" + Math.atan2(relY, relX));  // prints radial % and angle of mouse
  }
}

// draws the virtual joystick, parameter mousePos is an array either empty to not draw joystick or [x, y] to draw joystick
function drawStick(mousePos) {
  conCtx.clearRect(0, 0, controls.width, controls.height);  // clears old drawings
  conCtx.beginPath();
  conCtx.arc(mouseDown[0], mouseDown[1], 100, 0, 2 * Math.PI);  // draws a bounding circle where the button was originally pressed with radius 100
  conCtx.stroke();
  if(mousePos.length == 2) {  // if the mouse has been moved from the original location
    conCtx.beginPath();
    conCtx.arc(mousePos[0] + mouseDown[0], mouseDown[1] - mousePos[1], 10, 0, 2 * Math.PI);  // draws the joystick, a circle of raidus 10, at the current mouse location
    conCtx.stroke();
  }
}

// sets the twist object to the current velocities to be published periodically
function publishVelocity(forward, turn) {
  twist = {  // sets values of the twist message
    linear : {
      x : forward,  // sets linear x velocity to the forward parameter
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

// Define State Structure
// ----------------------

function State(lidar, odom) {
  this.lidar = lidar;
  this.odom = odom;
}

// Define World Class
// ------------------

function World() {
  this.max_length = 10;
  this.length = 0;
  this.head = null;
  this.tail = null;

  this.append = function(state) {
    // if the list is empty both head and tail point to the state being added
    if(this.head == null && this.tail == null) {
      this.head = state;
      this.tail = state;
    } else {  // if the list is not empty the old tail now points to the state being added
      this.tail.next = state;
      this.tail = state;
    }
    this.length++;
    // if the length has exceeded max_length advance the head pointer to remove the head
    if(this.length > this.max_length) {
      this.head = this.head.next;
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
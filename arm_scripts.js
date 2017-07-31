/*
 * Arm_Control | ZombieBot Arm Controller
 *
 * ZombieBot arm's joints controlled by sliders
 * Can update one joint at a time or the whole
 *   configuration at once.
 *
 * Brian May | Summer 2017 Internship
 * Laboratory4Progress
 */

var ros;                    // pointer to the ROSLIB.Ros object
var connectivity;           // html text displaying connectivity status
var mobile;                 // boolean indicating whether user is on mobile or not

var joints;                 // dictionary of pointers to html sliders and names of joints
var joint_publisher;        // publishes joint trajectories
var gripper_open;           // boolean for the state of the gripper
var pts, msg;               // rosbridge messages, pts - JointTrajectoryPoints[], msg - JointTrajectory
var full;                   // boolean for current mode, send full configuration or not

init();

// init is called first to set everything up
function init() {
  connectivity = document.getElementById("connectivity");
  mobile = navigator.userAgent.match(/(iPhone|Android|IEMobile)/); // check to see if the user is on a mobile device

  joints = {shoulder : document.getElementById("shoulder"),
            elbow_pitch : document.getElementById("elbow_pitch"),
            elbow_roll : document.getElementById("elbow_roll"),
            wrist_pitch : document.getElementById("wrist_pitch"),
            wrist_roll : document.getElementById("wrist_roll"),
            gripper_roll : document.getElementById("gripper_roll"),
            gripper : document.getElementById("gripper")
  };

  pts = [new ROSLIB.Message({             // allocate memory for pts and msg
    positions : []
  })];
  msg = new ROSLIB.Message({
    joint_names : [],
    points : pts
  });

  for(key in joints) {
    joints[key].addEventListener("change", sendJoint);  // add event listeners to each slider
  }

  full = false;
  gripper_open = true;
  joints.gripper.addEventListener("click", gripper_change);
  
  document.getElementById("full").addEventListener("change", function() {  // change mode when the checkbox gets changed
    full = !full;
  });
  document.getElementById("send").addEventListener("click", sendConfig);  // listener on send button to submit a full configuration
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

// initializes the publisher for magic_arm_joints
function subscribeToTopics() {
  joint_publisher = new ROSLIB.Topic({
    ros : ros,
    name : 'magic_arm_joints',
    messageType : 'trajectory_msgs/JointTrajectory'
  });
}

// called when a slider is changed
function sendJoint() {
  if(!full) {  // if in full config mode, do nothing
    pts[0].positions = [this.value * Math.PI/180];  // sets positions value to the slider's value converted to radians
    msg.joint_names = [this.id];
    msg.points = pts;

    joint_publisher.publish(msg);
  }
}

// called when the send button is clicked
function sendConfig() {
  var i = 0;
  for(key in joints) {
    pts[0].positions[i] = joints[key].value * Math.PI/180;  // get values from all sliders and convert to radians
    msg.joint_names[i] = key;
    i++;
  }
  pts[0].positions[joints.length-1] = gripper_open ? Math.PI : 0;
  msg.points = pts;

  joint_publisher.publish(msg);
}

// called to open or close the gripper
function gripper_change() {
  msg.joint_names = [this.id];
  pts[0].positions = gripper_open ? [0] : [Math.PI];  // 0 closes gripper, 180 opens gripper
  msg.points = pts;
  joint_publisher.publish(msg);
  joints.gripper.innerHTML = gripper_open ? "Open" : "Close";
  gripper_open = !gripper_open;
}

// called on stop button clicked, closes rosbridge connection
function terminate() {
  ros.close();                               // closes rosbridge websocket connection
  connectivity.innerHTML = "Not Connected";
  connectivity.style.color = "#000000";
}

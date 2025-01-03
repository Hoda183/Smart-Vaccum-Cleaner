"use strict";
// ros configuration
const ros = new ROSLIB.Ros({
  url: "ws://10.42.0.1:9090",
});

ros.on("connection", () => {
  document.body.style.backgroundColor = "#213555";
  vaccumButton.style.backgroundColor = "#adff2f";
  console.log("Ros Master connected");
});
ros.on("error", () => {
  document.body.style.backgroundColor = "#C62E2E";
});

const topic = new ROSLIB.Topic({
  ros: ros,
  name: "/ultrasonic/range",
  messageType: "std_msgs/Int16",
});

const controlModeService = new ROSLIB.Service({
  ros: ros,
  name: "/control_mode",
  serviceType: "control_mode_pkg/ControlMode",
});

const ultrasonicTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/ultrasonic/range",
  message: "std_msgs/Int32",
});

const motorsSpeedTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/motors_speed",
  messageType: " Int32MultiArray",
});

//New state topic
const vacuumStateTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/vaccum_state",
  messageType: "std_msgs/Bool",
});
// Function to publish vacuum state
function publishVacuumState(state) {
  const message = new ROSLIB.Message({
    data: state, // Publish true for ON and false for OFF
  });
  vacuumStateTopic.publish(message);
  console.log(`Vacuum state published: ${state ? "ON" : "OFF"}`);
}

ultrasonicTopic.subscribe((message) => {
  ultrasonicReadings.textContent = `${message.data} cm`;
});
// checking if epsnode is connected
setInterval((_) => {
  timeout = setTimeout(function () {
    if (connected) {
      vacState = false;
      vaccumButton.style.backgroundColor = "#ffffff";
      vaccumButton.textContent = "OFF";
      renderMenu(0);
      console.log("esp disconnected");
      connected = false;
    }
  }, 1500);
}, 1500);
ultrasonicTopic.subscribe(function () {
  clearTimeout(timeout);
  if (!connected) {
    connected = true;
    console.log("esp reconnected");
    renderMenu(1);
    if (vacState) vaccumButton.style.backgroundColor = "#adff2f";
    else vaccumButton.style.backgroundColor = "#c62300";
  }
});
// ros api
const serviceModeControl = function (mode) {
  // Create a request object with the mode parameter set to mode value
  const request = new ROSLIB.ServiceRequest({
    mode,
  });
  controlModeService.callService(
    request,
    (result) => {
      console.log("Service Response:", result);
    },
    (error) => {
      console.error("Service Call Failed:", error);
    }
  );
};

const publishMotorSpeed = function (motorsData) {
  let motorsSpeedData = new ROSLIB.Message({
    data: motorsData,
  });
  motorsSpeedTopic.publish(motorsSpeedData);
};
// elements storage
const vaccumButton = document.querySelector(".vaccum_button");
const content = document.querySelector(".content");
const modesMenu = document.querySelector(".modes");
const manualControlMenu = content.querySelector(".manual_control");
const loadingMenu = content.querySelector(".loading_div");
const menus = content.querySelectorAll(".menu");
const ultrasonicReadings = document.querySelector(".readings--value");

//  variables
let connected = false;
let leftSpeed, rightSpeed, motorsInterval;
let motorsCount = 0;
let vacState = true;
let timeout;

// functions
const hideMenus = function () {
  menus.forEach((e) => e.classList.add("hidden"));
};
const renderMenu = function (mode = 0) {
  hideMenus();
  switch (mode) {
    case 0: // default menu
      loadingMenu.classList.remove("hidden");
      break;
    case 1: // modes menu
      modesMenu.classList.remove("hidden");
      break;
    case 2: //keyboard
      if (window.screen.width < 400)
        manualControlMenu.classList.remove("hidden");
      else modesMenu.classList.remove("hidden");
      break;
  }
};

// handlers
modesMenu.addEventListener("click", function (e) {
  const targetEl = e.target.closest(".mode");
  if (!targetEl) return;
  document
    .querySelectorAll(".mode")
    .forEach((e) => e.classList.remove("active"));

  const mode = +targetEl.dataset.mode;
  if (mode === 1) renderMenu(2);
  if (mode != 0) targetEl.classList.add("active");
  serviceModeControl(mode);
});

vaccumButton.addEventListener("click", function (e) {
  if (!connected){
    console.log("Not connected to ROS. Cannot toggle vacuum state.");
    return;
  }
  
  if (vacState) {
    vaccumButton.style.backgroundColor = "#c62300";
    vaccumButton.textContent = "OFF";
    vacState = false;
  } 
  else {
    vaccumButton.style.backgroundColor = "#adff2f";
    vaccumButton.textContent = "ON";
    vacState = true;
  }
  publishVacuumState(vacState);
});
manualControlMenu.addEventListener("touchstart", (e) => {
  const el = e.target.closest(".arrow--key");
  if (!el) return;
  clearInterval(motorsInterval);
  if (el.classList.contains("arrow--up")) {
    leftSpeed = 255;
    rightSpeed = 255;
  }
  if (el.classList.contains("arrow--down")) {
    leftSpeed = -255;
    rightSpeed = -255;
  }
  if (el.classList.contains("arrow--left")) {
    leftSpeed = 0;
    rightSpeed = 255;
  }
  if (el.classList.contains("arrow--right")) {
    leftSpeed = 255;
    rightSpeed = 0;
  }
  motorsInterval = setInterval(
    (_) => publishMotorSpeed([leftSpeed, rightSpeed]),
    100
  );
});

manualControlMenu.addEventListener("touchend", (e) => {
  const el = e.target.closest(".arrow--key");
  if (!el) return;
  clearInterval(motorsInterval);
  motorsInterval = setInterval((_) => {
    publishMotorSpeed([0, 0]);
    if (++motorsCount == 50) {
      clearInterval(motorsInterval);
      motorsCount = 0;
    }
  }, 100);
});

manualControlMenu.addEventListener("click", (e) => {
  const el = e.target.closest(".back");
  if (!el) return;
  renderMenu(1);
});

// start the app
renderMenu(0);


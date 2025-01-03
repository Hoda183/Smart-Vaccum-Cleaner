const ros = new ROSLIB.Ros({
  // url: "ws://192.168.1.20:9090", // Replace with your ROSBridge server's address
  url: "ws://10.42.0.1:9090", // Replace with your ROSBridge server's address
  // url: "ws://0.0.0.0.:9090", // Replace with your ROSBridge server's address
});

ros.on("connection", () => {
  document.body.style.backgroundColor = "#213555";
  console.log("Ros Master connected");
});
ros.on("error", (error) => {
  document.body.style.backgroundColor = "#C62E2E";
});

const topic = new ROSLIB.Topic({
  ros: ros,
  name: "/ultrasonic/range", // Replace with your topic name
  messageType: "std_msgs/Int16", // The message type published on the topic
});
let connection = false;
const content = document.querySelector(".content");
const modesMenu = document.querySelector(".modes");
const manualControlMenu = content.querySelector(".manual_control");
const loadingMenu = content.querySelector(".loading_div");
const menus = content.querySelectorAll(".menu");
const ultrasonicReadings = document.querySelector(".readings--value");
console.log(content, modesMenu, manualControlMenu, loadingMenu);
modesMenu.addEventListener("click", function (e) {
  const targetEl = e.target.closest(".mode");
  if (!targetEl) return;
  document
    .querySelectorAll(".mode")
    .forEach((e) => e.classList.remove("active"));

  const mode = +targetEl.dataset.mode;
  if (mode === 1) renderMenu(2);
  if (mode != 0) targetEl.classList.add("active");
  const controlModeService = new ROSLIB.Service({
    ros: ros,
    name: "/control_mode", // The name of the service
    serviceType: "control_mode_pkg/ControlMode", // The type of the service
  });

  // Create a request object with the mode parameter set to 1
  console.log(mode);
  const request = new ROSLIB.ServiceRequest({
    mode, // Set mode to 1
  });
  controlModeService.callService(
    request,
    (result) => {
      console.log("Service Response:", result); // Log the response
    },
    (error) => {
      console.error("Service Call Failed:", error); // Log any error
    }
  );
});
const hideMenus = function () {
  menus.forEach((e) => e.classList.add("hidden"));
};
const renderMenu = function (mode = 0) {
  console.log(mode);
  hideMenus();
  switch (mode) {
    case 0:
      loadingMenu.classList.remove("hidden");
      break;
    case 1: // modes menu
      modesMenu.classList.remove("hidden");
      break;
    case 2: //keyboard
      manualControlMenu.classList.remove("hidden");
      break;
  }
};
const ultrasonicTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/ultrasonic/range",
  message: "std_msgs/Int32",
});
ultrasonicTopic.subscribe((message) => {
  ultrasonicReadings.textContent = `${message.data} cm`;
  console.log("hello");
});
let leftSpeed, rightSpeed, motorsInterval;
manualControlMenu.addEventListener("touchstart", (e) => {
  const el = e.target.closest(".arrow--key");
  if (!el) return;
  clearInterval(motorsInterval);
  if (el.classList.contains("arrow--up")) {
    leftSpeed = 255;
    rightSpeed = 255;
    console.log("up");
  }
  if (el.classList.contains("arrow--down")) {
    leftSpeed = -255;
    rightSpeed = -255;
    console.log("down");
  }
  if (el.classList.contains("arrow--left")) {
    leftSpeed = 0;
    rightSpeed = 255;
    console.log("left");
  }
  if (el.classList.contains("arrow--right")) {
    leftSpeed = 255;
    rightSpeed = 0;
    console.log("right");
  }
  motorsInterval = setInterval(
    (_) => publishMotorSpeed([leftSpeed, rightSpeed]),
    100
  );
});
let motorsCount = 0;
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
const motorsSpeedTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/motors_speed",
  messageType: " Int32MultiArray", // Change message type if needed
});
const publishMotorSpeed = function (motorsData) {
  let motorsSpeedData = new ROSLIB.Message({
    data: motorsData, // Replace with your message content
  });
  motorsSpeedTopic.publish(motorsSpeedData);
};

// checking if epsnode is connected
let timeout;
let connected = false;
ultrasonicTopic.subscribe(function (message) {
  clearTimeout(timeout);
  if (!connected) {
    renderMenu(1);
    connected = true;
  }
});

setInterval((_) => {
  timeout = setTimeout(function () {
    if (connected) {
      renderMenu(0);
      console.log("esp disconnected");
      connected = false;
    }
  }, 1500);
}, 1500);
// console.log(timeout);
// start excution
// renderLoading();
renderMenu(0);

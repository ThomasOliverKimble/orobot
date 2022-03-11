import RobotWindow from 'https://cyberbotics.com/wwi/R2022b/RobotWindow.js';

window.onload = function() {
  window.robotWindow = new RobotWindow();
  window.robotWindow.setTitle('Orobot Camera Window');
  window.robotWindow.send("Camera Window Loaded");
  window.robotWindow.receive = function(message, robot) {
    let added_div = document.createElement('p');
    added_div.innerHTML = message;
    document.getElementById("test").appendChild(added_div);
  }
}

function update_images(data) {
  if (data.devices.camera != null) {
      document.getElementById('camera-left').src = data.devices['camera_left']['image'] + '#' + new Date().getTime();;
      document.getElementById('camera-right').src = data.devices['camera_right']['image'] + '#' + new Date().getTime();;
  }
}
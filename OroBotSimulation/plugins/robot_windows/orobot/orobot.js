import RobotWindow from 'https://cyberbotics.com/wwi/R2022b/RobotWindow.js';

window.onload = function() {
  window.robotWindow = new RobotWindow();
  window.robotWindow.setTitle('Orobot Robot Window');
  window.robotWindow.send("Robot window loaded");
};

window.updateValue  = function(name, value) {
  document.getElementById('parameter_' + name + '_display').innerHTML = value;
  window.robotWindow.send("parameter: " + name + "=" + value);
}

window.restartPressed = function() {
  window.robotWindow.send("restart");
}
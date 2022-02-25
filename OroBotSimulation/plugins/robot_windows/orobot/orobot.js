window.onload = function() {
  window.robotWindow = webots.window("orobot");
  window.robotWindow.setTitle("Orobot Robot Window");
}

function updateValue(name, value) {
  document.getElementById('parameter_' + name + '_display').innerHTML = value;
  window.robotWindow.send("parameter: " + name + "=" + value);
}

function restartPressed() {
  window.robotWindow.send("restart");
}

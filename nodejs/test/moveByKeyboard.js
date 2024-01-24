import ROSLIB from "roslib";
import { Movement } from "../src/DogController.js";

const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
const movement = new Movement(ros);

// movement.move_forward();

//console input
const stdin = process.openStdin();
stdin.setRawMode(true);
stdin.resume();
stdin.setEncoding("utf8");
stdin.on("data", function (key) {
  if (key === "\u0003") {
    process.exit();
  }
  // write the key to stdout all normal like
  // process.stdout.write( key );
  console.log(key);
  switch (key) {
    case "w":
      movement.move_forward();
      break;
    case "s":
      movement.move_backward();
      break;
    case "a":
      movement.move_left();
      break;
    case "d":
      movement.move_right();
      break;
    case "q":
      movement.turn_left();
      break;
    case "e":
      movement.turn_right();
      break;
    default:
      movement.stop();
      break;
  }
  // movement.initTwist();
});

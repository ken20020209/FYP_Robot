import ROSLIB from "roslib";
import { Camera } from "../src/DogController.js";

const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
const camera = new Camera(ros);


camera.subCamCapture(0, (message) => {
  console.log(message);
});

#!/usr/bin/env node

// ROS node to link to Google Cloud Function

// ROS setup
const ros = require('rosnodejs');
ros.initNode('smarthome_gateway')
  .then(nh => {
    const std_msgs = ros.require('std_msgs');
    const smarthome_msgs = ros.require('smarthome');
  });

// Web interfaces
const ngrok = require('ngrok');
const express = require('express');
const axios = require('axios');
const app = express();

// Command line interface
const { exec } = require('child_process');
const { Http2ServerRequest } = require('http2');

// Handle SYNC requests, posted to
app.post('/', (req, res) => {
  ros.log.info('SYNC request recieved');
  res.status(202).json({
    user: "1836.15267389",
    devices: [{
      id: "123",
      type: "action.devices.types.OUTLET",
      traits: [
        "action.devices.traits.OnOff"
      ],
      name: {
        defaultNames: ["My Outlet 1234"],
        name: "Night light",
        nicknames: ["wall plug"]
      },
      willReportState: false,
      roomHint: "kitchen",
      deviceInfo: {
        manufacturer: "lights-out-inc",
        model: "hs1234",
        hwVersion: "3.2",
        swVersion: "11.4"
      },
      otherDeviceIds: [{
        deviceId: "local-device-id"
      }],
      customData: {
        fooValue: 74,
        barValue: true,
        bazValue: "foo"
      }
    }]
  })
});

// Function to store randomly generated url with cloud
async function send_url(public_url) {
  /*
  exec("curl http://localhost:4040/api/tunnels", (error, stdout, stderr) => {
    let data = JSON.parse(stdout);
    public_url = data.tunnels[0].public_url;
    console.log(public_url);
  */
  ros.log.info('Logging public url with cloud functions');

  axios({
    method: 'post',
    url: "https://us-central1-decent-booster-285122.cloudfunctions.net/connection_refresh",
    data: {
      url: public_url,
      client_id: '12955530',
      secret: "209j1ncncsc8w0010nijsb0q"
    }
  })
    .then((res) => {
      ros.log.info('Response from cloud recieved');
    })
    .catch((e) => {
      ros.log.error('Error hitting cloud functions: ');
      console.error(e);
    });
};



// Ngrok configuration object
const ngrokOpt = {
  proto: 'http',
  onStatusChange: status => {
    // Restart ngrok when it expires (every 2 hours)
    if (status == 'closed') {
      ros.log.info('Ngrok has expired.');
      restart_ngrok()
        .then((url) => {
          send_url(url);
        });
    }
  }
};
// Function to restart ngrok
async function restart_ngrok() {
  ros.log.info('Restarting ngrok');
  let url = await ngrok.connect(ngrokOpt)

  return new Promise((resolve, reject) => {
    if(url != undefined){
      ros.log.info('Restarted ngrok at: '+url);
      resolve(url);
    }
    else{
      reject(undefined);
    }
  })
}

// Make initial connection 
ngrok.connect(ngrokOpt)
  .then((url) => {
    ros.log.info('Opened https gateway at ' + url + '. Forwarding to localhost:5050');
    send_url(url);
  })
  .catch((e) => {
    ros.log.error('Error in ngrok: ');
    console.error(e);
  });

// Start app
ros.log.info('Fulfilment sever listening on port 5050');
app.listen(5050);
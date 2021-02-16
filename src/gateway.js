#!/usr/bin/env node

// ROS node to link to Google Cloud Function

// ROS setup
const ros = require('rosnodejs');
var sync_srv;
ros.initNode('smarthome_gateway')
  .then(nh => {
    const std_msgs = ros.require('std_msgs');
    const smarthome_msgs = ros.require('smarthome');

    sync_srv = nh.serviceClient('sync', 'smarthome/Sync');
  });

// Web interfaces
const ngrok = require('ngrok');
const axios = require('axios');
const express = require('express');
const app = express();

// Function to store randomly generated url with cloud
async function send_url(public_url) {
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
  addr: 5050,
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
    if (url != undefined) {
      ros.log.info('Restarted ngrok at: ' + url);
      resolve(url);
    }
    else {
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

// Handle SYNC requests
app.post('/smarthome/fulfillment/sync', (req, res) => {
  ros.log.info('SYNC request recieved');
  var devices = [];
  // Call sync service
  sync_srv.call({ key: 'squirrel' })
    .then((response) => {
      ros.log.info('SYNC data recieved from device manager');
      let i = 0;
      // Add each device to response array
      response.devices.forEach(element => {
        let device = {
          id: element.id,
          type: element.type,
          traits: element.traits,
          name: {
            name: element.name
          },
          willReportState: element.will_report_state
        }
        devices.push(device);
        i++;
      });
      // Send SYNC response to cloud
      res.status(202).json({
        user: "1836.15267389",
        devices: devices
      })
    })
    .catch((e) => {
      ros.log.error('Error processing SYNC data from device manager');
      console.error(e);
    });
});

// Handle QUERY requests
app.post('/smarthome/fulfillment/query', (req, res) => {

});

// Handle EXECUTE requests
app.post('/smarthome/fulfillment/execute', (req, res) => {

});

// Start app
ros.log.info('Fulfilment sever listening on port 5050');
app.listen(5050);
#!/usr/bin/env node

// ROS node to link to Google Cloud Function

const mapParser = require('../include/map_parser');

// ROS setup
const ros = require('rosnodejs');
var sync_srv;
var nodeHandle;
ros.initNode('smarthome_gateway')
  .then(nh => {
    nodeHandle = nh;
    const std_msgs = ros.require('std_msgs');
    const smarthome_msgs = ros.require('smarthome');

    sync_srv = nodeHandle.serviceClient('sync', 'smarthome/Sync');
  });

// Web interfaces
const ngrok = require('ngrok');
const axios = require('axios');
const express = require('express');
const app = express();
const bodyParser = require('body-parser');
app.use(bodyParser.json());


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
  console.log(req.body);
  var devices = [];
  // Call sync service
  sync_srv.call({ key: req.body.key })
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
  ros.log.info('QUERY request recieved');
  let devices = req.body.devices;
  var n_dev = devices.length;
  var i = 0;
  let payload = {
    errorCode: "",
    devices: []
  };
  devices.forEach(dev => {
    ros.log.info('Querying '+dev.id);
    let query_srv = nodeHandle.serviceClient(dev.id+'/query', 'smarthome/Query');
    // TODO: set timeout function, to be called if device is offline
    query_srv.call({key: "we0ri23m"})
    .then(response => {
      ros.log.info('Recieved query response from '+dev.id);
      // TODO: cancel timeout function
      let devStatus = mapParser.parse(response.param_names, response.param_values);
      devStatus.online = true;
      devStatus.status = "SUCCESS",
      devStatus.errorCode = response.error_code
      payload.devices[dev.id] = devStatus;
    })
    .catch(e => {
      ros.log.error('Error querying '+dev.id);
      console.log(e);
      // TODO: cancel timeout function
      let devStatus = {
        online: false,
        status: "ERROR",
        errorCode: ""
      };
      payload.devices[dev.id] = devStatus;
    });
    i++;
  });

  new Promise((resolve, reject) => {
    if(i == n_dev){
      res.status(202).send({payload: payload});
      resolve();
    }
    else {
      reject();
    }
  });
});

// Handle EXECUTE requests
app.post('/smarthome/fulfillment/execute', (req, res) => {
  ros.log.info('EXECUTE request recieved.');
  const commands = req.body.commands;
  const n_cmd = commands.length;
  var i = 0;
  let cmd_result = {
    commands: []
  };
  
  commands.forEach(command => {
    // Execute each command group
    const devices = command.devices;
    let command_res = {
      ids: [],
      status: "SUCCESS",
      errorCode: ""
    };
    // Call each device in this command group
    devices.forEach(device => {
      let dev_id = device.id;
      ros.log.info('Executing command on '+ dev_id);
      let exe_srv = nodeHandle.serviceClient(dev_id+'/execute', 'smarthome/Execute');
      exe_srv.call({
        command: command.execution[0].command,
        param_names: ["on"],
        param_values: [command.execution[0].params.on ? "true": "false"]
      })
      .then(response => {
        // Add device response to this command group response
        command_res.ids.push(dev_id);
        if(response.error_code != ""){
          command_res.errorCode = response.error_code;
        }
      });
    });
    // Add this command group response to the execute result
    cmd_result.commands.push(command_res);
    i++;
  });

  new Promise((resolve, reject) => {
    if(i == n_cmd){
      // loop complete
      res.status(202).json({
        user: "1836.15267389",
        cmd_result: cmd_result
      });
      resolve();
    }
    else{
      reject();
    }
  });
});

// Start app
ros.log.info('Fulfilment sever listening on port 5050');
app.listen(5050);